from launch import LaunchDescription
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("aquabot_ekf"),
        "config", "ekf_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="aquabot_ekf",
            executable="ekf_node",
            name="ekf_node",
            output="screen",
            parameters=[config_path]
        )
    ])
