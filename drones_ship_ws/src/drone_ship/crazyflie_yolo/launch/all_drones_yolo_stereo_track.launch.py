from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace, Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='drone1',
        description='Namespace for the drone'
    )

    namespace = LaunchConfiguration('namespace')

    package_share = get_package_share_directory('crazyflie_yolo')

    return LaunchDescription([
        namespace_arg,
        PushRosNamespace(namespace),

        # YOLO Detector Node
        Node(
            package="crazyflie_yolo",
            executable="yolo_detector_node.py",
            name="yolo_detector_node",
            output="screen",
            parameters=[
                {"model_path": "yolov8n.pt"},
                {"camera_topic": "downward_left_camera/image_raw"},
                {"detections_topic": "yolo_detections"},
                {"annotated_image_topic": "yolo/image_annotated"},
                {"use_sim_time": True}
            ]
        ),

        # Stereo Depth Node
        Node(
            package="crazyflie_yolo",
            executable="stereo_depth_node",
            name="stereo_depth_node",
            output="screen",
            parameters=[
                {"use_sim_time": True},
            ]
        ),

        # Multi-Obstacle Tracker Node
        Node(
            package="crazyflie_yolo",
            executable="multi_obstacle_tracker_node",
            name="multi_obstacle_tracker_node",
            output="screen",
            parameters=[
                {"association_threshold": 4.0},
                {"duplicate_distance_threshold": 0.5},
                {"track_prune_time": 3.0},
                {"use_sim_time": True},
            ]
        )     
    ])
