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
                        {"model_path": "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/dataset_rgb/runs/yolo8_rgb/train4/weights/best.pt"},                        
                        {"rgb_topic": "downward_left_camera/image_raw"},
                        {"disp_topic": "stereo/disparity_viz"},   
                        {"detections_topic": "yolo_detections"},
                        {"annotated_image_topic": "yolo/image_annotated"},
                        {"min_confidence": 0.90},
                        {"imgsz": 832},
                        {"iou": 0.45},
                        {"max_det": 50},
                        {"use_sim_time": True},
                    ],
                    respawn=True,
                    output='screen'
                ),

                # Stereo Depth Node
                Node(
                    package='crazyflie_yolo',
                    executable='stereo_depth_node',
                    name='stereo_depth_node',
                    parameters=[{"use_sim_time": True}, {"min_confidence": 0.80}],
                    respawn=True,
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
                        {"association_threshold": 10.0}, #was 40
                        {"track_prune_time": 10.0},
                        {"r_x": 1.0}, {"r_y": 1.0}, {"r_z": 3.0}, # was 0.5, 0.5. 2.0
                        {"q_process": 0.5},
                        {"init_var_pos": 2.0},
                        {"init_var_vel": 1.0},
                        {"use_sim_time": True},
                    ],
                    respawn=True,
                    output='screen'
                ),


                # Baseline 1: AB3DMOT-like 3D tracker (from detections_3d)
                Node(
                    package='crazyflie_yolo',
                    executable='ab3dmot_3d_tracker_node',
                    name='ab3dmot_3d_tracker_node',
                    parameters=[
                        {"detection_topic": "detections_3d"},
                        {"output_topic": "ab3dmot/tracked_obstacles_array"},
                        {"association_threshold": 10.0},
                        {"max_missed": 10},
                        {"q_process": 1.0},
                        {"r_meas": 0.75},
                        {"require_same_class": True},
                        {"use_sim_time": True},
                    ],
                    respawn=True,
                    output='screen'
                ),

                # Baseline 2: SORT-like tracker (2D YOLO + disparity -> 3D)
                Node(
                    package='crazyflie_yolo',
                    executable='sort_tracker_node',
                    name='sort_tracker_node',
                    parameters=[
                    {"yolo_topic": "yolo_detections"},
                    {"disparity_topic": "stereo/disparity"},   # raw disparity
                    {"camera_info_topic": "downward_left_camera/camera_info"},
                    {"output_topic": "sort/tracked_obstacles_array"},
                    {"stereo_baseline_m": 0.10},               
                    {"world_frame": "world"},
                    {"camera_frame": ""},
                    {"debug": True},
                    {"debug_every_n_ticks": 5},                     
                    {"tf_timeout_sec": 0.15},
                    {"min_disparity": 0.5},
                    {"max_depth_m": 80.0},
                    {"association_threshold": 12.0},
                    {"max_missed": 20},
                    {"q_process": 1.5},
                    {"r_meas": 1.0},
                    {"min_score": 0.70},
                    {"use_sim_time": True},
                    ],
                    respawn=True,
                    output='screen'
                ),

                # Metrics node: compare OURS vs AB3DMOT vs SORT
                Node(
                    package='crazyflie_yolo',
                    executable='tracking_metrics_node',
                    name='tracking_metrics_node',
                    parameters=[
                        {"eval_rate_hz": 5.0},
                        {"gt_container_count": 5},
                        {"dist_thresh": 15.0},                 # diagnostic: relax matching
                        {"require_nonempty_pred": True},      # keep counting even if empty
                        {"tick_time_source": "now"},           # often reduces GT time skew
                        {"pred_timeout_sec": 2.0},             # avoid false "stale"
                        {"debug": True},
                        {"debug_every_n_ticks": 5},

                        {"methods": ["ours", "ab3dmot", "sort"]},

                        {"method_topics.ours": f"/{drone}/tracked_obstacles_array"},
                        {"method_topics.ab3dmot": f"/{drone}/ab3dmot/tracked_obstacles_array"},
                        {"method_topics.sort": f"/{drone}/sort/tracked_obstacles_array"},

                        {"csv_path": f"/home/user/data/drones_ship_ws/metrics/{drone}_tracking_metrics.csv"},
                        {"csv_append": False},
                        {"write_every_n": 1},

                        {"use_sim_time": True},
                    ],
                    output='screen'
                ),
            ])
        )
     
    return LaunchDescription(actions)
