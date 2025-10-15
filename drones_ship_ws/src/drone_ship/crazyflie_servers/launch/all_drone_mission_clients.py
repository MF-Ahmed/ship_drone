# all_drone_mission_clients.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _make_nodes(context):
    drones_arg = LaunchConfiguration('drones').perform(context)
    drones = [d.strip() for d in drones_arg.replace(',', ' ').split() if d.strip()]
    nodes = []
    for ns in drones:
        nodes.append(Node(
            package='crazyflie_servers',
            executable='test_mission_client',
            name=f'mission_client_{ns}',
            output='screen',
            # Remap BOTH absolute and relative prefixes + explicit action names
            remappings=[
                ('/drone1', f'/{ns}'),
                ('drone1',  f'{ns}'),
                ('/drone1/navigate_to_hover', f'/{ns}/navigate_to_hover'),
                ('/drone1/move_forward',      f'/{ns}/move_forward'),
                ('drone1/navigate_to_hover',  f'{ns}/navigate_to_hover'),
                ('drone1/move_forward',       f'{ns}/move_forward'),
            ],
            parameters=[],
        ))
    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drones',
            default_value='drone1,drone2,drone3',
            description='Comma/space separated namespaces'),
        OpaqueFunction(function=_make_nodes),
    ])

