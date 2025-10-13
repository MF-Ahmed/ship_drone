#!/usr/bin/env python3
import os
import math
import random
import time
import xacro

from simple_launch import SimpleLauncher, GazeboBridge
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------------------------
# SimpleLauncher
# ------------------------------------------------------------------------------
sl = SimpleLauncher(use_sim_time=True)

# ---------------------------
# Launch-time arguments
# ---------------------------
sl.declare_arg('world', 'medium_new')        # world base name; actual file: aquabot_windturbines_<world>.sdf
sl.declare_arg('gui', True)
sl.declare_arg('rviz_config', 'system_rviz.rviz')

# container randomization
sl.declare_arg('container_count', 5)         # number of containers
sl.declare_arg('radius', 80.0)               # max radius (m)
sl.declare_arg('min_radius', 0.0)            # inner radius (m), use >0 to avoid the center/Aquabot
sl.declare_arg('min_sep', 8.0)               # min spacing between containers (m)
sl.declare_arg('z_spawn', 1.0)               # initial Z (m)
sl.declare_arg('yaw_random', True)           # randomize yaw (heading)
sl.declare_arg('seed', '')                   # reproducible if set (int or any string)

# ------------------------------------------------------------------------------
# Paths (keep your originals)
# ------------------------------------------------------------------------------
pkg_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
pkg_gazebo  = get_package_share_directory('ros_gz_crazyflie_gazebo')
pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

# Xacro → URDF (your files)
xacro_path1 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/drone1.urdf.xacro'
xacro_path2 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/drone2.urdf.xacro'
xacro_path3 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/rviz/urdf/drone3.urdf.xacro'

crazyflie_urdf1 = xacro.process_file(xacro_path1).toxml()
crazyflie_urdf2 = xacro.process_file(xacro_path2).toxml()
crazyflie_urdf3 = xacro.process_file(xacro_path3).toxml()
urdf_files = [crazyflie_urdf1, crazyflie_urdf2, crazyflie_urdf3]

# SDF files for drones (your files)
sdf_file1 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/gazebo/crazyflie/drone1.sdf'
sdf_file2 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/gazebo/crazyflie/drone2.sdf'
sdf_file3 = '/home/user/data/drones_ship_ws/src/drone_ship/aquabot_gz/models/crazyflie/gazebo/crazyflie/drone3.sdf'
sdf_files = [sdf_file1, sdf_file2, sdf_file3]

# Containers model root (adjust if needed)
CONTAINER_MODELS_ROOT = os.path.expanduser('~/.gz/models/newnames')


# ------------------------------------------------------------------------------
# RNG helpers for container placement
# ------------------------------------------------------------------------------
def _seed_rng(seed_arg: str):
    """Seed RNG from arg, env, or time for reproducibility."""
    if seed_arg:
        try:
            seed = int(seed_arg)
        except ValueError:
            seed = hash(seed_arg)
    else:
        env = os.environ.get('CONTAINER_SEED', '').strip()
        if env:
            try:
                seed = int(env)
            except ValueError:
                seed = hash(env)
        else:
            seed = int(time.time() * 1e6) & 0xFFFFFFFF
    random.seed(seed)
    print(f"[launch] Container RNG seed: {seed}")
    return seed

def _rand_point_in_disk(radius=80.0, min_radius=0.0):
    """Uniformly sample a point in a disk (area-uniform via r^2 sampling)."""
    r2 = random.uniform(min_radius**2, radius**2)
    r = math.sqrt(r2)
    theta = random.uniform(-math.pi, math.pi)
    return r * math.cos(theta), r * math.sin(theta)

def _generate_positions(n, radius=80.0, min_radius=0.0, min_separation=8.0, max_tries_per_point=200):
    """Generate n points with simple rejection to satisfy min_separation."""
    pts = []
    for _ in range(n):
        for _try in range(max_tries_per_point):
            x, y = _rand_point_in_disk(radius, min_radius)
            if all((x - px)**2 + (y - py)**2 >= min_separation**2 for px, py in pts):
                pts.append((x, y))
                break
        else:
            # If constraints are tight, relax and accept the next sample
            x, y = _rand_point_in_disk(radius, min_radius)
            pts.append((x, y))
    return pts


# ------------------------------------------------------------------------------
# Launch setup
# ------------------------------------------------------------------------------
def launch_setup():
    # --- World selection
    world = sl.arg('world')
    if 'aquabot' not in world:
        world = f'aquabot_windturbines_{world}'
    world = world.replace('.sdf', '')

    gz_args = '-r'
    if not sl.arg('gui'):
        gz_args += ' -s'
    sl.gz_launch(sl.find('aquabot_gz', world + '.sdf'), gz_args=gz_args)

    # --- Clock bridge (world clock -> /clock)
    sl.node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_clock_bridge',
        arguments=[
            f'/world/{world}/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            '--ros-args', '-r', f'/world/{world}/clock:=/clock'
        ],
        output='screen'
    )

    # --- AIS bridge (as in your file)
    sl.node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge_ais',
        arguments=[
            '/aquabot/ais_sensor/windturbines_positions@geometry_msgs/msg/Pose@gz.msgs.Pose'
        ],
        output='screen'
    )

    # -----------------------------
    # Aquabot setup (group ns=aquabot)
    # -----------------------------
    with sl.group(ns='aquabot'):
        sl.robot_state_publisher('aquabot_description', 'aquabot.urdf')
        sl.spawn_gz_model('aquabot')

        # Odometry bridge for aquabot
        sl.node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/model/aquabot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            remappings=[('/model/aquabot/odometry', '/aquabot/odometry')],
            output='screen'
        )

        # Aquabot bridges
        aquabot_bridges = []
        # joint states
        aquabot_bridges.append((f'/world/{world}/model/aquabot/joint_state',
                                'joint_states',
                                'sensor_msgs/msg/JointState', GazeboBridge.gz2ros))
        # thrusters (cmds)
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
        # sensors
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

        # static TF world -> aquabot/odom
        sl.node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_world_to_aquabot_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'aquabot/odom'],
            output='screen'
        )

    # -----------------------------
    # Multi-Crazyflie setup (3 drones)
    # -----------------------------
    num_drones = 3
    for i in range(num_drones):
        drone_name = f"drone{i+1}"   # Gazebo model name
        ns = f"/{drone_name}"        # ROS namespace

        with sl.group(ns=drone_name):
            this_sdf_file = sdf_files[i]
            this_urdf_file = urdf_files[i]

            # Spawn drone (slight Y offsets like your original)
            sl.node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{drone_name}',
                arguments=[
                    '-x', str(1),                # you can add + i * 1.1 if desired
                    '-y', str(-0.60 + i * 0.6), # staggered Y
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

            # Bridges for each drone
            drone_bridges = []
            drone_bridges.append((
                f'/world/{world}/model/{drone_name}/link/{drone_name}/body/sensor/navsat/navsat',
                f'{ns}/gps',
                'sensor_msgs/msg/NavSatFix',
                GazeboBridge.gz2ros
            ))
            drone_bridges.append((
                f'/world/{world}/model/{drone_name}/link/{drone_name}/body/sensor/imu_sensor/imu',
                f'{ns}/imu',
                'sensor_msgs/Imu',
                GazeboBridge.gz2ros
            ))
            drone_bridges.append((
                f'/model/{drone_name}/pose',
                f'{ns}/ground_truth_pose',
                'geometry_msgs/msg/Pose',
                GazeboBridge.gz2ros
            ))
            sl.create_gz_bridge(drone_bridges)

            # static TF world -> drone odom
            sl.node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'static_tf_world_to_{drone_name}_odom',
                arguments=['0', '0', '0', '0', '0', '0', 'world', f'{drone_name}/odom'],
                output='screen'
            )

            # Additional YAML-configured bridges (your original)
            sl.node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name=f'gz_bridge_{drone_name}',
                parameters=[{
                    'config_file': sl.find('aquabot_gz', f'ros_gz_{drone_name}_bridge.yaml'),
                }],
                output='screen'
            )

    # -----------------------------
    # Randomized container spawns
    # -----------------------------
    count       = int(sl.arg('container_count'))
    radius      = float(sl.arg('radius'))
    min_radius  = float(sl.arg('min_radius'))
    min_sep     = float(sl.arg('min_sep'))
    z_spawn     = float(sl.arg('z_spawn'))
    yaw_random  = bool(sl.arg('yaw_random'))
    _seed_rng(str(sl.arg('seed')))

    # Build per-container model paths; reuse container1 if higher numbers are missing
    model_paths = []
    for i in range(count):
        idx = i + 1
        path = os.path.join(CONTAINER_MODELS_ROOT, f'container{idx}', 'model.sdf')
        if not os.path.exists(path):
            fallback = os.path.join(CONTAINER_MODELS_ROOT, 'container1', 'model.sdf')
            path = fallback
        model_paths.append(path)

    # Generate positions with spacing
    positions = _generate_positions(
        n=count,
        radius=radius,
        min_radius=min_radius,
        min_separation=min_sep
    )

    container_names = []
    for i, ((x, y), model_file) in enumerate(zip(positions, model_paths), start=1):
        name = f'container{i}'
        yaw = random.uniform(-math.pi, math.pi) if yaw_random else 0.0
        print(f"[launch] Spawning {name} at x={x:.2f}, y={y:.2f}, z={z_spawn:.2f}, yaw={yaw:.2f} rad")

        sl.node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{name}',
            arguments=[
                '-name', name,
                '-x', str(x),
                '-y', str(y),
                '-z', str(z_spawn),
                '-Y', str(yaw),
                '-file', model_file
            ],
            output='screen'
        )
        container_names.append(name)

    # Container odometry bridges
    for name in container_names:
        sl.node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'gz_bridge_{name}_odom',
            arguments=[f'/model/{name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'],
            remappings=[(f'/model/{name}/odometry', f'/{name}/odometry')],
            output='screen'
        )

    # -----------------------------
    # RViz (optional; uses your config path inside aquabot_gz)
    # -----------------------------
    sl.node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', sl.find('aquabot_gz', sl.arg('rviz_config'))],
        parameters=[{'use_sim_time': True}]
    )

    return sl.launch_description()


# ------------------------------------------------------------------------------
# Generate the launch description
# ------------------------------------------------------------------------------
generate_launch_description = sl.launch_description(launch_setup)

