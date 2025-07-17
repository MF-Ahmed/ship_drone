from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Stereo depth node
        Node(
            package="crazyflie_yolo",
            executable="stereo_depth_node",
            name="stereo_depth_node",
            output="screen",
            parameters=[
                # Optional parameters for stereo_depth_node
                {"left_camera_topic": "/crazyflie/downward_left_camera/image_raw"},
                {"right_camera_topic": "/crazyflie/downward_right_camera/image_raw"},
                {"yolo_detections_topic": "/crazyflie/yolo_detections"},
            ]
        ),

        # Multi-obstacle tracker node
        Node(
            package="crazyflie_yolo",
            executable="multi_obstacle_tracker_node",
            name="multi_obstacle_tracker_node",
            output="screen",
            parameters=[
                
                {"association_threshold": 4.0},
                {"duplicate_distance_threshold": 0.5},
                {"track_prune_time": 3.0}
            ]
        )
    ])

