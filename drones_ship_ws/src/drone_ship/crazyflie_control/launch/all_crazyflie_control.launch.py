from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('crazyflie_control'),
        'config',
        'config.yaml'
    )

    drones = ['drone1', 'drone2', 'drone3']

    actions = []

    for drone in drones:
        actions.append(
            GroupAction([
                PushRosNamespace(drone),
                Node(
                    package='crazyflie_control',
                    executable='crazyflie_control_node',
                    name='crazyflie_control_node',
                    parameters=[{'config_file': config_path}],
                    output='screen'
                )
            ])
        )

    return LaunchDescription(actions)
