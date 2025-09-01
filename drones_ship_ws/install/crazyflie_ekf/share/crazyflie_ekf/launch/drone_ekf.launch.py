from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(__file__), '..')
    config_file = os.path.join(pkg_share, 'config', 'drone_ekf.yaml')

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='drone1',
            description='Namespace for the drone EKF node'
        ),
        PushRosNamespace(namespace),
        Node(
            package='crazyflie_ekf',
            executable='crazyflie_ekf_node',
            name='ekf',            
            output='screen',
            parameters=[config_file, {'use_sim_time': True}]
        )
    ])
