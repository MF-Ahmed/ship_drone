from simple_launch import SimpleLauncher, GazeboBridge
import xacro
from launch.actions import TimerAction
sl = SimpleLauncher(use_sim_time=True)
sl.declare_arg('world', 'easy')
sl.declare_arg('gui', True)


def launch_setup():

    # launch world
    world = sl.arg('world')
    if 'aquabot' not in world:
        world = f'aquabot_windturbines_{world}'
    world.strip('.sdf')

    gz_args = '-r'
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('aquabot_gz', world+'.sdf'), gz_args=gz_args)

    bridges = [GazeboBridge.clock()]
     # Process crazyflie URDF from xacro
    xacro_path = '/home/user/data/ros2_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/crazyflie.urdf.xacro'
    crazyflie_urdf = xacro.process_file(xacro_path).toxml()

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

        sl.create_gz_bridge(bridges)
        sl.node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', sl.find('aquabot_gz', 'aquabot.rviz')],
            parameters=[{'use_sim_time': True}]
        
        ) 
          

        sl.node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'aquabot/base_link'],
            output='screen'
        )

        # Crazyflie setup
        with sl.group(ns='crazyflie'):

            # Spawn model by name (must exist in GZ_SIM_RESOURCE_PATH)
            #sl.set_spawn_pose('crazyflie', [0, 0, 0.4, 0, 0, 0])
            sl.spawn_gz_model('crazyflie')
            # Robot state publisher
            sl.node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='crazyflie_robot_state_publisher',
                parameters=[
                    {'robot_description': crazyflie_urdf},
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
                arguments=['0', '0', '1', '0', '0', '0', 'world', 'crazyflie/odom'],
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


            
        

    return sl.launch_description()


generate_launch_description = sl.launch_description(launch_setup)
