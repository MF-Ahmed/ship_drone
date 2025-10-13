from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Hard-coded drone namespaces
    drones = ['drone1', 'drone2', 'drone3']

    actions = []

    for drone in drones:
        actions.append(
            GroupAction([
                PushRosNamespace(drone),

                # YOLO Detector Node
                Node(
                    package='crazyflie_yolo',
                    executable='yolo_detector_node.py',
                    name='yolo_detector_node',
                    parameters=[
                        {"model_path": "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/models/rgb/best_rgb.pt"},
                        {"camera_topic": "downward_left_camera/image_raw"},
                        {"detections_topic": "yolo_detections"},
                        {"annotated_image_topic": "yolo/image_annotated"},
                        {"use_sim_time": True},
                    ],
                    output='screen'
                ),

                # Stereo Depth Node
                Node(
                    package='crazyflie_yolo',
                    executable='stereo_depth_node',
                    name='stereo_depth_node',
                    parameters=[{"use_sim_time": True}, {"min_confidence": 0.92}],
                    output='screen'
                ),

                # EKF Multi-Obstacle Tracker Node
                Node(
                    package='crazyflie_yolo',
                    executable='multi_obstacle_tracker_ekf_node',
                    name='multi_obstacle_tracker_ekf_node',
                    parameters=[
                        {"detection_topic": "detections_3d"},
                        {"group_mode": "class_and_drone"},
                        {"association_threshold": 40.0},
                        {"track_prune_time": 10.0},
                        {"r_x": 0.5}, {"r_y": 0.5}, {"r_z": 2.0},
                        {"q_process": 0.5},
                        {"init_var_pos": 2.0},
                        {"init_var_vel": 1.0},
                        {"use_sim_time": True},
                    ],
                    output='screen'
                ),
            ])
        )

    return LaunchDescription(actions)
