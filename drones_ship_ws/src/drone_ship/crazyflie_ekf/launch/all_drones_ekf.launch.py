from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():
    pkg_share = os.path.join(os.path.dirname(__file__), '..')
    config_file = os.path.join(pkg_share, 'config', 'drone_ekf.yaml')

    return LaunchDescription([
        # Drone 1
        PushRosNamespace('drone1'),
        Node(
            package='crazyflie_ekf',
            executable='crazyflie_ekf_node',
            name='ekf',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}]
        ),

        # Drone 2
        PushRosNamespace('drone2'),
        Node(
            package='crazyflie_ekf',
            executable='crazyflie_ekf_node',
            name='ekf',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}]
        ),

        # Drone 3
        PushRosNamespace('drone3'),
        Node(
            package='crazyflie_ekf',
            executable='crazyflie_ekf_node',
            name='ekf',
            output='screen',
            parameters=[config_file, {'use_sim_time': True}]
        )
    ])
