# multi_drone_explore_track.launch.py
# ROS 2 Jazzy

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def make_drone_group(
    ns: str,
    default_alt: float = 10.0,
    leg_distance: float = 20.0,
    forward_speed: float = 0.25,
    use_low_tracker=False,
    low_tracker_action: str = "track_target",
    odom_topic: str = "ekf/odom",
    cmd_vel_topic: str = "cmd_vel",
    enable_path_tracer: bool = True,
):
    """Create nodes for one drone under a namespace."""

    # Low-level action servers
    hover_srv = Node(
        package="crazyflie_servers",
        executable="navigate_to_hover_server",
        name="navigate_to_hover_server",
        namespace=ns,
        output="screen",
        parameters=[{
            "odom_topic": odom_topic,
            "cmd_vel_topic": cmd_vel_topic,
        }],
        # <<< Key fix: rewrite any absolute/stray names to namespaced topics >>>
        remappings=[
            # velocity
            ("cmd_vel",         cmd_vel_topic),
            ("/cmd_vel",        cmd_vel_topic),
            ("drone1/cmd_vel",  cmd_vel_topic),
            ("/drone1/cmd_vel", cmd_vel_topic),
            # odometry
            ("ekf/odom",         odom_topic),
            ("/ekf/odom",        odom_topic),
            ("drone1/ekf/odom",  odom_topic),
            ("/drone1/ekf/odom", odom_topic),
        ],
    )

    move_forward_srv = Node(
        package="crazyflie_servers",
        executable="move_forward_server",
        name="move_forward_server",
        namespace=ns,
        output="screen",
        parameters=[{
            "odom_topic": odom_topic,
            "cmd_vel_topic": cmd_vel_topic,
            "cmd_frame": "body",
            "hold_yaw": True,
            "yaw_kp": 0.6,
            "yaw_rate_max": 0.3,
            "forward_ramp_rate": 0.005,
            "speed_limit": 0.4,
            "brake_margin": 6.0,
            "slow_radius": 1.0,
            "stop_tolerance": 0.15,
            "stop_hold_time": 0.2,
            "yaw_gate_radius": 0.6,
            "yaw_min_speed": 0.05,
        }],
        remappings=[
            ("cmd_vel",         cmd_vel_topic),
            ("/cmd_vel",        cmd_vel_topic),
            ("drone1/cmd_vel",  cmd_vel_topic),
            ("/drone1/cmd_vel", cmd_vel_topic),

            ("ekf/odom",         odom_topic),
            ("/ekf/odom",        odom_topic),
            ("drone1/ekf/odom",  odom_topic),
            ("/drone1/ekf/odom", odom_topic),
        ],
    )

    controller = Node(
        package="crazyflie_servers",
        executable="explore_and_track_controller",
        name="explore_and_track_controller",
        namespace=ns,
        output="screen",
        parameters=[{
            "default_ascend_altitude": default_alt,
            "default_leg_distance": leg_distance,
            "default_forward_speed": forward_speed,
            # absolute is fine here, but relative would also work since we’re in ns
            "navigate_action": f"/{ns}/navigate_to_hover",
            "forward_action":  f"/{ns}/move_forward",
            "use_low_tracker": use_low_tracker,
            "low_tracker_action": f"/{ns}/{low_tracker_action}",
        }],
    )

    nodes = [hover_srv, move_forward_srv, controller]

    if enable_path_tracer:
        path_tracer = Node(
            package="crazyflie_servers",
            executable="path_tracer",
            name="path_tracer",
            namespace=ns,
            output="screen",
            parameters=[{
                "odom_topic": odom_topic,
                "path_topic": "trajectory",
                "max_points": 5000,
            }],
            remappings=[
                ("ekf/odom",         odom_topic),
                ("/ekf/odom",        odom_topic),
                ("drone1/ekf/odom",  odom_topic),
                ("/drone1/ekf/odom", odom_topic),
                # publish path inside the namespace even if absolute slipped in
                ("/trajectory",       "trajectory"),
                ("drone1/trajectory", "trajectory"),
                ("/drone1/trajectory","trajectory"),
            ],
        )
        nodes.append(path_tracer)

    return nodes


def generate_launch_description():
    use_low_tracker_arg = DeclareLaunchArgument(
        "use_low_tracker", default_value="false",
        description="Delegate /track goals to /<ns>/track_target"
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic", default_value="ekf/odom",
        description="Relative or absolute odometry topic per namespace"
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        "cmd_vel_topic", default_value="cmd_vel",
        description="Relative or absolute cmd_vel topic per namespace"
    )
    enable_path_tracer_arg = DeclareLaunchArgument(
        "enable_path_tracer", default_value="true",
        description="Publish nav_msgs/Path at /<ns>/trajectory for RViz"
    )

    use_low_tracker = LaunchConfiguration("use_low_tracker")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    enable_path_tracer = LaunchConfiguration("enable_path_tracer")

    nodes = []
    for ns in ["drone1", "drone2", "drone3"]:
        nodes += make_drone_group(
            ns=ns,
            use_low_tracker=use_low_tracker,
            odom_topic=odom_topic,
            cmd_vel_topic=cmd_vel_topic,
            enable_path_tracer=enable_path_tracer,
        )

    return LaunchDescription([
        use_low_tracker_arg,
        odom_topic_arg,
        cmd_vel_topic_arg,
        enable_path_tracer_arg,
        *nodes
    ])

