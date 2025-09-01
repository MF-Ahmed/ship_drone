import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Locate packages
    pkg_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Resolve model.sdf from GZ_SIM_RESOURCE_PATH
    gz_model_paths = os.getenv('GZ_SIM_RESOURCE_PATH', '').split(':')
    sdf_file = None
    for path in gz_model_paths:
        candidate = os.path.join(path, 'crazyflie', 'model.sdf')
        if os.path.exists(candidate):
            sdf_file = candidate
            break

    if sdf_file is None:
        raise FileNotFoundError("Could not find crazyflie/model.sdf in GZ_SIM_RESOURCE_PATH")

    with open(sdf_file, 'r') as f:
        robot_description = f.read()

    # Launch Gazebo with a world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': os.path.join(pkg_gazebo, 'worlds', 'crazyflie_world.sdf') + ' -r'
        }.items(),
    )

    # Spawn Crazyflie model
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_crazyflie',
        arguments=[
            '-name', 'crazyflie',
            '-x', '0', '-y', '0', '-z', '1',
            '-file', sdf_file
        ],
        output='screen'
    )

    # Publish robot state
    state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='crazyflie_robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Static transform
    tf_static = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '1', '0', '0', '0', 'world', 'crazyflie/odom'],
        output='screen'
    )

    # Bridge topics
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_bringup, 'config', 'ros_gz_crazyflie_bridge.yaml'),
        }],
        output='screen'
    )

    # Drone control node
    control = Node(
        package='ros_gz_crazyflie_control',
        executable='control_services',
        output='screen',
        parameters=[
            {'hover_height': 0.5},
            {'robot_prefix': '/crazyflie'},
            {'incoming_twist_topic': '/cmd_vel'},
            {'max_ang_z_rate': 0.4},
        ]
    )

    return LaunchDescription([
        gz_sim,
        spawn_entity,
        state_pub,
        tf_static,
        bridge,
        control,
    ])

