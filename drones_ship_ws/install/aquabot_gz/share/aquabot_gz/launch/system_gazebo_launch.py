import os
from simple_launch import SimpleLauncher, GazeboBridge
import xacro
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction



sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('world', 'medium_new')
sl.declare_arg('gui', True)


def launch_setup():

    # launch world
    world = sl.arg('world')
    if 'aquabot' not in world:
        world = f'aquabot_windturbines_{world}'
    world = world.replace('.sdf', '')

    gz_args = '-r'
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('aquabot_gz', world+'.sdf'), gz_args=gz_args)

    bridges = [GazeboBridge.clock()]


    pkg_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
     # Process crazyflie URDF from xacro


    #xacro_path = '/home/user/data/ros2_ws/src/drone_ship/aquabot_gz/models/crazy_fly/rviz/urdf/crazyflie.urdf.xacro'
    #crazyflie_urdf = xacro.process_file(xacro_path).toxml()


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

    #print(sdf_file)
    #print(robot_description)


    # turbine bridges
    bridges.append(('/vrx/windturbinesinspection/windturbine_checkup',
                    '/vrx/windturbinesinspection/windturbine_checkup',
                    'std_msgs/msg/String', GazeboBridge.ros2gz))
    bridges.append(('/aquabot/ais_sensor/windturbines_positions',
                    '/aquabot/ais_sensor/windturbines_positions',
                    'geometry_msgs/msg/Pose', GazeboBridge.gz2ros))
    sl.create_gz_bridge(bridges)

    # spawn aquabot robot
    with sl.group(ns = 'aquabot'):

        sl.robot_state_publisher('aquabot_description', 'aquabot.urdf')
        sl.spawn_gz_model('aquabot')

        # add bridges for this model
        bridges = []
        bridges.append((f'/world/{world}/model/aquabot/joint_state',
                        'joint_states',
                        'sensor_msgs/msg/JointState', GazeboBridge.gz2ros))
        # command
        for side in ('left','right'):
            bridges.append((f'/aquabot/thrusters/{side}/pos',
                            f'thrusters/{side}/cmd_pos',
                            'std_msgs/Float64', GazeboBridge.ros2gz))
            bridges.append((f'/aquabot/thrusters/{side}/thrust',
                            f'thrusters/{side}/thrust',
                            'std_msgs/Float64', GazeboBridge.ros2gz))
        bridges.append(('/aquabot/thrusters/main_camera_sensor/pos',
                        'camera/cmd_pos',
                        'std_msgs/Float64', GazeboBridge.ros2gz))

        # sensors
        bridges.append((f'/world/{world}/model/aquabot/link/aquabot/gps_link/sensor/navsat/navsat',
                        'sensors/gps/gps/fix',
                        'sensor_msgs/NavSatFix', GazeboBridge.gz2ros))
        bridges.append((f'/world/{world}/model/aquabot/link/aquabot/imu_link/sensor/imu_sensor/imu',
                        'sensors/imu/imu/data',
                        'sensor_msgs/Imu', GazeboBridge.gz2ros))
        bridges.append((f'/world/{world}/model/aquabot/link/aquabot/main_camera_post_link/sensor/main_camera_sensor/image',
                        'sensors/cameras/main_camera_sensor/image_raw',
                        'sensor_msgs/Image', GazeboBridge.gz2ros))
        for info in ('range', 'bearing'):
            bridges.append((f'/aquabot/sensors/acoustics/receiver/{info}',
                            f'sensors/acoustics/receiver/{info}',
                            'std_msgs/Float64', GazeboBridge.gz2ros))


        # Crazyflie setup
        with sl.group(ns='crazyflie'):

            # Spawn model by name (must exist in GZ_SIM_RESOURCE_PATH)
            #sl.set_spawn_pose('crazyflie', [0, 0, 0.4, 0, 0, 0])
                # Spawn Crazyflie model
        
            sl.node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_crazyflie',
                arguments=[
                    '-name', 'crazyflie',
                    '-x', '1', '-y', '0', '-z', '2',  # moved far and higher
                    '-file', sdf_file
                ],
                output='screen'
            )    
                             
            # Robot state publisher
            
            sl.node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='crazyflie_robot_state_publisher',
                parameters=[
                    {'robot_description': robot_description},
                    {'use_sim_time': True},
                    {'publish_fixed_joints': True}
                ],
                output='screen'
            )       
            

            # Bridge via YAML config
            sl.node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{
                    'config_file': sl.find('ros_gz_crazyflie_bringup', 'ros_gz_crazyflie_bridge.yaml'),
                }],
                output='screen'
            )
            sl.node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '2', '0', '0', '0', 'world', 'crazyflie/odom'],
                output='screen'
            )
            

            # Control services for drone
            sl.node(
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


        sl.node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'aquabot/base_link'],
            output='screen'
        )


        sl.node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_floating_container',
            arguments=[
                '-name', 'floating_container',
                '-x', '20', '-y', '5', '-z', '0.2',
                '-file', os.path.expanduser('~/.gz/models/floating_container/model.sdf')
            ],
            output='screen'
        )      


        sl.node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_container1',
            arguments=[
                '-name', 'container1',
                '-x', '10', '-y', '10', '-z', '1.0',
                '-file', os.path.expanduser('~/.gz/models/container1/model.sdf')
            ],
            output='screen'
        )      



        sl.node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_container2',
            arguments=[
                '-name', 'container2',
                '-x', '10', '-y', '20', '-z', '1.0',
                '-file', os.path.expanduser('~/.gz/models/container2/model.sdf')
            ],
            output='screen'
        )     



        sl.node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_container3',
            arguments=[
                '-name', 'container3',
                '-x', '10', '-y', '-20', '-z', '1.0',
                '-file', os.path.expanduser('~/.gz/models/container3/model.sdf')
            ],
            output='screen'
        )    



        sl.node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_container4',
            arguments=[
                '-name', 'container4',
                '-x', '10', '-y', '-30', '-z', '1.0',
                '-file', os.path.expanduser('~/.gz/models/container4/model.sdf')
            ],
            output='screen'
        )                    
     

     

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
