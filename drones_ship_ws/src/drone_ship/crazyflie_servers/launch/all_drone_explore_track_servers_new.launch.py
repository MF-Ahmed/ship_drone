# multi_drone_explore_track.launch.py (ROS 2 Jazzy)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def _to_bool(s): return str(s).lower() in ("1","true","yes","on")

def make_drone_group(ns, odom_topic, cmd_vel_topic,
                     default_alt=10.0, leg_distance=20.0, forward_speed=0.25,
                     use_low_tracker=False, low_tracker_action="track_target",
                     enable_path_tracer=True):

    hover_srv = Node(
        package="crazyflie_servers",
        executable="navigate_to_hover_server",
        name="navigate_to_hover_server",
        namespace=ns,
        output="screen",
        parameters=[{"odom_topic": odom_topic, "cmd_vel_topic": cmd_vel_topic}],
        remappings=[
            ("cmd_vel", cmd_vel_topic), ("/cmd_vel", cmd_vel_topic),
            ("drone1/cmd_vel", cmd_vel_topic), ("/drone1/cmd_vel", cmd_vel_topic),
            ("ekf/odom", odom_topic), ("/ekf/odom", odom_topic),
            ("drone1/ekf/odom", odom_topic), ("/drone1/ekf/odom", odom_topic),
        ],
    )

    move_forward_srv = Node(
        package="crazyflie_servers",
        executable="move_forward_server",
        name="move_forward_server",
        namespace=ns,
        output="screen",
        parameters=[{
            "odom_topic": odom_topic, "cmd_vel_topic": cmd_vel_topic,
            "cmd_frame": "body", "hold_yaw": True, "yaw_kp": 0.6, "yaw_rate_max": 0.3,
            "forward_ramp_rate": 0.005, "speed_limit": 0.4, "brake_margin": 6.0,
            "slow_radius": 1.0, "stop_tolerance": 0.15, "stop_hold_time": 0.2,
            "yaw_gate_radius": 0.6, "yaw_min_speed": 0.05,
        }],
        remappings=[
            ("cmd_vel", cmd_vel_topic), ("/cmd_vel", cmd_vel_topic),
            ("drone1/cmd_vel", cmd_vel_topic), ("/drone1/cmd_vel", cmd_vel_topic),
            ("ekf/odom", odom_topic), ("/ekf/odom", odom_topic),
            ("drone1/ekf/odom", odom_topic), ("/drone1/ekf/odom", odom_topic),
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
            "use_low_tracker": ParameterValue(use_low_tracker, value_type=bool),
            "navigate_action": f"/{ns}/navigate_to_hover",
            "forward_action":  f"/{ns}/move_forward",
            "low_tracker_action": f"/{ns}/{low_tracker_action}",
        }],
    )

    nodes = [hover_srv, move_forward_srv, controller]

    if enable_path_tracer:
        nodes.append(Node(
            package="crazyflie_servers",
            executable="path_tracer",
            name="path_tracer",
            namespace=ns,
            output="screen",
            parameters=[{"odom_topic": odom_topic, "path_topic": "trajectory", "max_points": 5000}],
            remappings=[
                ("ekf/odom", odom_topic), ("/ekf/odom", odom_topic),
                ("drone1/ekf/odom", odom_topic), ("/drone1/ekf/odom", odom_topic),
                ("/trajectory", "trajectory"), ("drone1/trajectory", "trajectory"),
                ("/drone1/trajectory","trajectory"),
            ],
        ))
    return nodes

def _build_drone_list(drones_csv, num_drones, prefix):
    drones_csv = (drones_csv or "").strip()
    if drones_csv:
        return [d.strip() for d in drones_csv.split(",") if d.strip()]
    # fallback: generate prefix1..prefixN
    try:
        n = int(num_drones)
    except Exception:
        n = 3
    return [f"{prefix}{i+1}" for i in range(n)]

def launch_setup(context, *args, **kwargs):
    # resolve args
    drones_csv = LaunchConfiguration("drones").perform(context)
    num_drones  = LaunchConfiguration("num_drones").perform(context)
    prefix      = LaunchConfiguration("prefix").perform(context)
    use_low     = _to_bool(LaunchConfiguration("use_low_tracker").perform(context))
    enable_path = _to_bool(LaunchConfiguration("enable_path_tracer").perform(context))
    odom_topic  = LaunchConfiguration("odom_topic").perform(context)
    cmd_vel     = LaunchConfiguration("cmd_vel_topic").perform(context)

    drones = _build_drone_list(drones_csv, num_drones, prefix)

    nodes = []
    for ns in drones:
        nodes += make_drone_group(
            ns=ns,
            odom_topic=odom_topic,
            cmd_vel_topic=cmd_vel,
            use_low_tracker=use_low,
            enable_path_tracer=enable_path,
        )
    return nodes

def generate_launch_description():
    return LaunchDescription([
        # Primary way: explicit names
        DeclareLaunchArgument(
            "drones",
            default_value="drone1,drone2,drone3",
            description="Comma-separated drone namespaces (overrides num_drones/prefix)"
        ),
        # Fallbacks if 'drones' is empty:
        DeclareLaunchArgument("num_drones", default_value="3",
            description="How many drones to auto-generate if 'drones' is empty"),
        DeclareLaunchArgument("prefix", default_value="drone",
            description="Prefix for auto-generated names (e.g., 'uav' -> uav1,uav2,...)"),
        # Other args you already had
        DeclareLaunchArgument("use_low_tracker", default_value="false"),
        DeclareLaunchArgument("odom_topic", default_value="ekf/odom"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel"),
        DeclareLaunchArgument("enable_path_tracer", default_value="true"),
        OpaqueFunction(function=launch_setup),
    ])
