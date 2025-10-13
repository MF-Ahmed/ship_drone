import os
from simple_launch import SimpleLauncher, GazeboBridge
import xacro
from ament_index_python.packages import get_package_share_directory

# Create launcher
sl = SimpleLauncher(use_sim_time=True)

# Declare arguments
sl.declare_arg('world', 'medium_new')
sl.declare_arg('gui', False)
sl.declare_arg('rviz_config', 'system_rviz.rviz')  # default file

# Packages
pkg_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
pkg_gazebo = get_package_share_directory('ros_gz_crazyflie_gazebo')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

# Paths
xacro_path1 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/drone1.urdf.xacro'
crazyflie_urdf1 = xacro.process_file(xacro_path1).toxml()


xacro_path2 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/drone2.urdf.xacro'
crazyflie_urdf2 = xacro.process_file(xacro_path2).toxml()


xacro_path3 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/drone3.urdf.xacro'
crazyflie_urdf3 = xacro.process_file(xacro_path3).toxml()


urdf_files = [crazyflie_urdf1, crazyflie_urdf2, crazyflie_urdf3]


sdf_file1 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/gazebo/crazyflie/drone1.sdf'
sdf_file2 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/gazebo/crazyflie/drone2.sdf'
sdf_file3 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/gazebo/crazyflie/drone3.sdf'

sdf_files = [sdf_file1, sdf_file2, sdf_file3]

def launch_setup():
    # Launch world
    world = sl.arg('world')
    if 'aquabot' not in world:
        world = f'aquabot_windturbines_{world}'
    world = world.replace('.sdf', '')

    gz_args = '-r'
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('aquabot_gz', world+'.sdf'), gz_args=gz_args)

    # Basic clock bridge

    sl.node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',   # unique name
        arguments=[
            f'/world/{world}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '--ros-args', '-r', f'/world/{world}/clock:=/clock'
        ],  
        output='screen'
    )

    # ------------------------------------------------------------------
    # AIS bridge (separate, no /clock)
    # ------------------------------------------------------------------
    sl.node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_ais',     # unique name
        arguments=[
            '/aquabot/ais_sensor/windturbines_positions@geometry_msgs/msg/Pose@gz.msgs.Pose'
        ],
        output='screen'
    )


    # Aquabot setup
    with sl.group(ns='aquabot'):
        sl.robot_state_publisher('aquabot_description', 'aquabot.urdf')
        sl.spawn_gz_model('aquabot')

        sl.node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/aquabot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            remappings=[('/model/aquabot/odometry', '/aquabot/odometry')],
            output='screen'
        )

        aquabot_bridges = []
        aquabot_bridges.append((f'/world/{world}/model/aquabot/joint_state',
                                'joint_states',
                                'sensor_msgs/msg/JointState', GazeboBridge.gz2ros))
        # Add thruster bridges
        for side in ('left', 'right'):
            aquabot_bridges.append((f'/aquabot/thrusters/{side}/pos',
                                    f'thrusters/{side}/cmd_pos',
                                    'std_msgs/Float64', GazeboBridge.ros2gz))
            aquabot_bridges.append((f'/aquabot/thrusters/{side}/thrust',
                                    f'thrusters/{side}/thrust',
                                    'std_msgs/Float64', GazeboBridge.ros2gz))
        aquabot_bridges.append(('/aquabot/thrusters/main_camera_sensor/pos',
                                'camera/cmd_pos',
                                'std_msgs/Float64', GazeboBridge.ros2gz))
        # Sensors
        aquabot_bridges.append((f'/world/{world}/model/aquabot/link/aquabot/gps_link/sensor/navsat/navsat',
                                'sensors/gps/gps/fix',
                                'sensor_msgs/NavSatFix', GazeboBridge.gz2ros))
        aquabot_bridges.append((f'/world/{world}/model/aquabot/link/aquabot/imu_link/sensor/imu_sensor/imu',
                                '/aquabot/imu',
                                'sensor_msgs/Imu', GazeboBridge.gz2ros))
        

        aquabot_bridges.append((f'/world/{world}/model/aquabot/link/aquabot/main_camera_post_link/sensor/main_camera_sensor/image',
                        'sensors/cameras/main_camera_sensor/image_raw',
                        'sensor_msgs/Image', GazeboBridge.gz2ros))
        for info in ('range', 'bearing'):
            aquabot_bridges.append((f'/aquabot/sensors/acoustics/receiver/{info}',
                            f'sensors/acoustics/receiver/{info}',
                            'std_msgs/Float64', GazeboBridge.gz2ros))

        sl.create_gz_bridge(aquabot_bridges)

        sl.node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_aquabot_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'aquabot/odom'],
            output='screen'
        )

    # Multi-Crazyflie setup
    num_drones = 3  # Set how many drones you want
    for i in range(num_drones):
        drone_name = f"drone{i+1}"  # Model name in Gazebo
        ns = f"/{drone_name}"       # ROS namespace

        with sl.group(ns=drone_name):
            # Spawn Crazyflie SDF
            this_sdf_file = sdf_files[i]   # drone names are implicitly decladed into the files 
            this_urdf_file = urdf_files[i]  # drone names are implicitly decladed into the files    
            sl.node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{drone_name}',
                arguments=[                    
                    '-x', str(1),# + i * 1.1),  # Offset drones along X
                    '-y', str(-0.60 + i *0.6),
                    '-z', '1.7',
                    '-file', this_sdf_file
                ],
                output='screen'
            )

            # Robot state publisher
            sl.node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name=f'{drone_name}_robot_state_publisher',
                parameters=[
                    {'robot_description': this_urdf_file},
                    {'use_sim_time': True},
                    {'publish_fixed_joints': True}
                ],
                output='screen'
            )

            # Bridges
            drone_bridges = []


            drone_bridges.append((
                f'/world/{world}/model/{drone_name}/link/{drone_name}/body/sensor/navsat/navsat',
                f'{ns}/gps',
                'sensor_msgs/msg/NavSatFix',
                GazeboBridge.gz2ros
            ))            

            drone_bridges.append((f'/world/{world}/model/{drone_name}/link/{drone_name}/body/sensor/imu_sensor/imu',
                                  f'{ns}/imu',
                                  'sensor_msgs/Imu',
                                  GazeboBridge.gz2ros))
            drone_bridges.append((f'/model/{drone_name}/pose',
                                  f'{ns}/ground_truth_pose',
                                  'geometry_msgs/msg/Pose',
                                  GazeboBridge.gz2ros))
            sl.create_gz_bridge(drone_bridges)

            # Static TF
            sl.node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_world_to_{drone_name}_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'world', f'{drone_name}/odom'],
                output='screen'
            )


            # Bridge via YAML config
            sl.node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'gz_bridge_{drone_name}',
                parameters=[{
                    'config_file': sl.find('aquabot_gz', f'ros_gz_{drone_name}_bridge.yaml'),
                }],
                output='screen'
            )
      
           
    sl.node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_container1',
        arguments=[
            '-name', 'container1',
            '-x', '40', '-y', '-5', '-z', '1.0',
            '-file', os.path.expanduser('~/.gz/models/newnames/container1/model.sdf')
        ],
        output='screen'
    )      
    
              
    sl.node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_container2',
        arguments=[
            '-name', 'container2',
            '-x', '60', '-y', '3', '-z', '1.0',
            #'-R', '0', '-P', '0', '-Y', '1.57', 
            '-file', os.path.expanduser('~/.gz/models/newnames/container2/model.sdf')
        ],
        output='screen'
    )     
     
    sl.node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_container3',
        arguments=[
            '-name', 'container3',
            '-x', '40', '-y', '10', '-z', '1.0',
            '-file', os.path.expanduser('~/.gz/models/newnames/container3/model.sdf')
        ],
        output='screen'
    )     
            
    sl.node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_container4',
        arguments=[
            '-name', 'container4',
            '-x', '60', '-y', '15', '-z', '1.0',
            '-file', os.path.expanduser('~/.gz/models/newnames/container4/model.sdf')
        ],
        output='screen'
    )    
       
        
    sl.node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_container5',
        arguments=[
            '-name', 'container5',
            '-x', '80', '-y', '-15', '-z', '1.0',
            '-file', os.path.expanduser('~/.gz/models/newnames/container5/model.sdf')
        ],
        output='screen'
    )                    
    
    '''
    sl.node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_barrel1',
        arguments=[
            '-name', 'barrel1',
            '-x', '10', '-y', '-20', '-z', '1.0',
            '-file', os.path.expanduser('~/.gz/models/newnames/barrel1/model.sdf')
        ],
        output='screen'
    )                    

    '''


    for name in ('container1','container2','container3','container4','container5', 'barrel1'):#, 'barrel2', 'log1', 'log2'):
        sl.node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'gz_bridge_{name}_odom',
            arguments=[f'/model/{name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            remappings=[(f'/model/{name}/odometry', f'/{name}/odometry')],
            output='screen'
        )



    # RViz
    sl.node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', sl.find('aquabot_gz', 'aquabot.rviz')],
        parameters=[{'use_sim_time': True}]
    )

    return sl.launch_description()

# Generate the launch description
generate_launch_description = sl.launch_description(launch_setup)
