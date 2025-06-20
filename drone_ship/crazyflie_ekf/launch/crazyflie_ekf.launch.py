from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('crazyflie_ekf'),
        'config',
        'crazyflie_ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='crazyflie_ekf',
            executable='crazyflie_ekf_node',
            name='crazyflie_ekf_node',
            output='screen',
            parameters=[config_file]
        )
    ])
