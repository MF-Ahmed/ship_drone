import os
from simple_launch import SimpleLauncher, GazeboBridge
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

sl = SimpleLauncher(use_sim_time=True)

# ✅ Declare arguments at top-level
sl.declare_arg('world', 'aquabot_windturbines_medium_new')
sl.declare_arg('gui', True)

def launch_setup():
    world = sl.arg('world')

    # Add Gazebo bridge topics for aquabot
    bridges = [
        (f'/world/{world}/model/aquabot/joint_state', 'joint_states', 'sensor_msgs/msg/JointState', GazeboBridge.gz2ros),
        ('/aquabot/thrusters/left/pos', 'thrusters/left/cmd_pos', 'std_msgs/Float64', GazeboBridge.ros2gz),
        ('/aquabot/thrusters/right/pos', 'thrusters/right/cmd_pos', 'std_msgs/Float64', GazeboBridge.ros2gz),
        ('/aquabot/thrusters/left/thrust', 'thrusters/left/thrust', 'std_msgs/Float64', GazeboBridge.ros2gz),
        ('/aquabot/thrusters/right/thrust', 'thrusters/right/thrust', 'std_msgs/Float64', GazeboBridge.ros2gz),
        ('/aquabot/thrusters/main_camera_sensor/pos', 'camera/cmd_pos', 'std_msgs/Float64', GazeboBridge.ros2gz),
        (f'/world/{world}/model/aquabot/link/aquabot/gps_link/sensor/navsat/navsat', 'sensors/gps/gps/fix', 'sensor_msgs/NavSatFix', GazeboBridge.gz2ros),
        (f'/world/{world}/model/aquabot/link/aquabot/imu_link/sensor/imu_sensor/imu', 'sensors/imu/imu/data', 'sensor_msgs/Imu', GazeboBridge.gz2ros),
        (f'/world/{world}/model/aquabot/link/aquabot/main_camera_post_link/sensor/main_camera_sensor/image', 'sensors/cameras/main_camera_sensor/image_raw', 'sensor_msgs/Image', GazeboBridge.gz2ros),
        ('/aquabot/sensors/acoustics/receiver/range', 'sensors/acoustics/receiver/range', 'std_msgs/Float64', GazeboBridge.gz2ros),
        ('/aquabot/sensors/acoustics/receiver/bearing', 'sensors/acoustics/receiver/bearing', 'std_msgs/Float64', GazeboBridge.gz2ros),
    ]

    sl.create_gz_bridge(bridges)

    # Launch RViz
    sl.node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', sl.find('aquabot_gz', 'aquabot.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return sl.launch_description()

generate_launch_description = sl.launch_description(launch_setup)

