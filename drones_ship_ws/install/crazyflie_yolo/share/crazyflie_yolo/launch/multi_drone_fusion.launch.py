from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Multi-Drone Fusion Node
        Node(
            package="crazyflie_yolo",
            executable="multi_drone_fusion_node",
            name="multi_drone_fusion_node",
            output="screen",
            parameters=[
                {"drone_names": ["drone1", "drone2", "drone3"]},  # ✅ Adjust drone list
                {"use_sim_time": True}
            ]
        )
    ])
