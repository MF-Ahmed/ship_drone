from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

DRONES = ["drone1", "drone2", "drone3"]


def _mkdir_for_file(path: str):
    d = os.path.dirname(path)
    if d:
        os.makedirs(d, exist_ok=True)


def _setup_dirs(context, *args, **kwargs):
    fusion_log = LaunchConfiguration("fusion_log_path").perform(context)
    assign_log = LaunchConfiguration("assignment_csv_path").perform(context)

    event_log = LaunchConfiguration("event_log_path").perform(context)
    stats_log = LaunchConfiguration("stats_log_path").perform(context)
    prune_log = LaunchConfiguration("prune_log_path").perform(context)

    fusion_metrics_csv = LaunchConfiguration("fusion_metrics_csv_path").perform(context)

    _mkdir_for_file(fusion_log)
    _mkdir_for_file(assign_log)
    _mkdir_for_file(event_log)
    _mkdir_for_file(stats_log)
    _mkdir_for_file(prune_log)
    _mkdir_for_file(fusion_metrics_csv)

    return []


def generate_launch_description():
    # Defaults
    assign_log_default = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_assignment/assignment_log.csv"
    fusion_log_default = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fused_obstacles.csv"
    event_log_default = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fusion_events.csv"
    prune_log_default = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/prune_events.csv"
    stats_log_default = "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fusion_stats.csv"
    fusion_metrics_csv_default = "/home/user/data/drones_ship_ws/metrics/fusion_metrics.csv"

    # Launch arguments
    fusion_log_arg = DeclareLaunchArgument(
        "fusion_log_path",
        default_value=fusion_log_default,
        description="CSV file path for fusion logging"
    )

    assignment_log_arg = DeclareLaunchArgument(
        "assignment_csv_path",
        default_value=assign_log_default,
        description="CSV file path for assignment logging"
    )

    event_log_arg = DeclareLaunchArgument(
        "event_log_path",
        default_value=event_log_default,
        description="CSV file path for event logging"
    )

    prune_log_arg = DeclareLaunchArgument(
        "prune_log_path",
        default_value=prune_log_default,
        description="CSV file path for prune logging"
    )

    stats_log_arg = DeclareLaunchArgument(
        "stats_log_path",
        default_value=stats_log_default,
        description="CSV file path for stats logging"
    )

    fusion_metrics_arg = DeclareLaunchArgument(
        "fusion_metrics_csv_path",
        default_value=fusion_metrics_csv_default,
        description="CSV file path for fusion comparison metrics (CI vs FTCI)"
    )

    nodes = []

    # Create dirs before starting nodes
    nodes.append(OpaqueFunction(function=_setup_dirs))

    # --- CI fusion node (your existing multi_drone_fusion_node) ---
    nodes.append(
        Node(
            package="crazyflie_yolo",
            executable="multi_drone_fusion_node",
            name="multi_drone_fusion_node",
            output="screen",
            parameters=[{
                "drone_names": DRONES,
                "topic_format": "/%s/tracked_obstacles_array",
                "association_threshold": 8.0,

                "prune_time_s": 0.8,
                "prune_cov_logdet_max": 1.0,
                "prune_cov_max_std_m": 1.3,
                "mahal_prune_threshold": 1.5,
                "prune_done_enabled": False,
                "prune_done_logdet": -0.2,
                "mahal_max_misses": 3,

                "aquabot_frame": "aquabot/base_link",

                # logs
                "log_path": LaunchConfiguration("fusion_log_path"),
                "event_log_path": LaunchConfiguration("event_log_path"),
                "prune_log_path": LaunchConfiguration("prune_log_path"),
                "stats_log_path": LaunchConfiguration("stats_log_path"),
                "log_append": False,

                "debug": False,
                "use_sim_time": True
            }]
        )
    )

    # --- FTCI fusion node (separate baseline) ---
    nodes.append(
        Node(
            package="crazyflie_yolo",
            executable="ftci_fusion_node",
            name="ftci_fusion_node",
            output="screen",
            parameters=[{
                "input_topics": [f"/{d}/tracked_obstacles_array" for d in DRONES],
                "output_topic": "/fused_tracked_obstacles_array_ftci",

                # keep association identical to CI for fairness
                "association_threshold": 8.0,

                # FTCI parameters
                "chi2_thresh": 11.345,   # DOF=3, 99% chi-square
                "union_scale": 1.0,
                "omega_min": 0.05,
                "omega_max": 0.95,

                "fuse_rate_hz": 10.0,
                "use_sim_time": True,
            }]
        )
    )

    # --- Fusion metrics node (CI vs FTCI) ---
    nodes.append(
        Node(
            package="crazyflie_yolo",
            executable="fusion_metrics_node",
            name="fusion_metrics_node",
            output="screen",
            parameters=[{
                "gt_container_count": 5,
                "dist_thresh": 8.0,
                "eval_rate_hz": 5.0,
                "world_frame": "world",
                "tf_timeout_sec": 0.05,

                "ci_topic": "/fused_tracked_obstacles_array",
                "ftci_topic": "/fused_tracked_obstacles_array_ftci",

                "csv_path": LaunchConfiguration("fusion_metrics_csv_path"),
                "csv_append": False,

                "use_sim_time": True,
            }]
        )
    )

    # --- Assignment node ---
    nodes.append(
        Node(
            package="crazyflie_yolo",
            executable="assignment_node",
            name="assignment_node",
            output="screen",
            parameters=[{
                "drone_odom_topics": [
                    "/drone1/ekf/odom",
                    "/drone2/ekf/odom",
                    "/drone3/ekf/odom"
                ],
                "alloc_rate_hz": 1.0,

                "eta": 1.0, "beta": 0.2, "gamma": 0.3, "rho": 0.2, "kappa": 1000.0,
                "r_safe": 0.5, "d_max": 25.0,
                "drone_capacity": 2,

                "hover_l": 12,
                "hover_radius": 1.5,
                "hover_height": 10.0,
                "min_alt": 9.5,
                "max_alt": 10.0,

                "hover_inside_tol_m": 0.3,
                "hover_z_tol_m": 0.5,
                "best_margin_logdet": 0.05,

                "tau_logdet": -2.0,
                "tau_dJ": 7.0,
                "tmax_meas": 2.0,

                "print_term": True,
                "verbose": False,

                "csv_enable": True,
                "csv_path": LaunchConfiguration("assignment_csv_path"),

                "gt_enable": True,
                "gt_max_age_s": 5.0,
                "gt_odom_topics": [
                    "/container1/odometry",
                    "/container2/odometry",
                    "/container3/odometry",
                    "/container4/odometry",
                    "/container5/odometry",
                ],

                "use_sim_time": True,
                "print_enable": True,
                "print_throttle_ms": 200,
            }],
            remappings=[
                ("fused_tracked_obstacles_array", "/fused_tracked_obstacles_array"),
            ],
        )
    )

    # --- qstar nodes ---
    for dn in DRONES:
        nodes.append(
            Node(
                package="crazyflie_yolo",
                executable="qstar_goal_node",
                name=f"qstar_goal_node_{dn}",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "ns": dn,
                    "odom_topic": f"/{dn}/ekf/odom",
                    "qstar_topic": f"/{dn}/assigned_pose",
                    "max_step": 0.10,
                    "min_step": 0.02,
                    "alpha": 0.25,
                    "max_jump_world": 1.0,
                    "publish_dxdy": True,
                }],
            )
        )

    return LaunchDescription([
        fusion_log_arg,
        assignment_log_arg,
        event_log_arg,
        prune_log_arg,
        stats_log_arg,
        fusion_metrics_arg,
        *nodes
    ])