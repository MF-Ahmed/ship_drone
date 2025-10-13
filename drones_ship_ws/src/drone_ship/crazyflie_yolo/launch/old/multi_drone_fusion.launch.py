from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="crazyflie_yolo",
            executable="multi_drone_fusion_node",
            name="multi_drone_fusion_node",
            output="screen",
            parameters=[{
                "drone_names": ["drone1", "drone2", "drone3"],  # ✅ list of drones
                "topic_format": "/%s/tracked_obstacles_array",
                "association_threshold": 40.0,
                "prune_time_s": 5.0,
                "aquabot_frame": "aquabot/base_link",
                "log_path": "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fused_obstacles.csv",
                "log_append": False,
                "use_sim_time": True
            }]
        )
    ])
