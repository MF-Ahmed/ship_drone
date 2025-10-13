from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('crazyflie_ekf'),
        'config',
        'drone_ekf.yaml'
    )

    # You can extend this list to ['drone1', 'drone2', 'drone3']
    drones = ['drone1', 'drone2', 'drone3']

    actions = []

    for drone in drones:
        actions.append(
            GroupAction([
                PushRosNamespace(drone),
                Node(
                    package='crazyflie_ekf',
                    executable='crazyflie_ekf_node',
                    name='ekf',
                    parameters=[config_path, {'use_sim_time': True}],
                    output='screen'
                )
            ])
        )

    return LaunchDescription(actions)
