from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os

def launch_setup(context, *args, **kwargs):
    config = LaunchConfiguration('config').perform(context)
    # Convert string -> bool
    use_sim_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim = use_sim_str.lower() in ('1','true','yes','on')

    drones_arg = LaunchConfiguration('drones').perform(context)
    drones = [d.strip() for d in drones_arg.split(',') if d.strip()]

    actions = []
    for drone in drones:
        actions.append(
            GroupAction([
                PushRosNamespace(drone),
                Node(
                    package='crazyflie_ekf',
                    executable='crazyflie_ekf_node',
                    name='ekf',
                    parameters=[config, {'use_sim_time': use_sim}],
                    output='screen'
                )
            ])
        )
    return actions

def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('crazyflie_ekf'), 'config', 'drone_ekf.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('drones', default_value='drone1,drone2,drone3'),
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
