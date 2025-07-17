from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('crazyflie_control'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='drone1'),
        Node(
            package='crazyflie_control',
            executable='crazyflie_control_node',
            name='crazyflie_control_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'config_file': config_path}],
            output='screen'
        )
    ])
