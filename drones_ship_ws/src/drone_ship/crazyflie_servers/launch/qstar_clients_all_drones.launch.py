#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _make_nodes(context, *args, **kwargs):
    # Comma-separated list of drone namespaces (no leading slash)
    drones_csv = LaunchConfiguration("drones").perform(context)
    drones = [d.strip().lstrip("/") for d in drones_csv.split(",") if d.strip()]
    if not drones:
        drones = ["drone1"]

    speed = float(LaunchConfiguration("speed").perform(context))
    hover_sec = int(LaunchConfiguration("hover_sec").perform(context))
    hover_first = LaunchConfiguration("hover_first").perform(context).lower() in ["true", "1", "yes"]
    oneshot = LaunchConfiguration("oneshot").perform(context).lower() in ["true", "1", "yes"]

    nodes = []
    for ns in drones:
        nodes.append(
            Node(
                package="crazyflie_servers",
                executable="qstar_dual_action_client",
                name=f"qstar_dual_action_client_{ns}",
                output="screen",
                parameters=[
                    {"ns": ns},
                    {"assigned_pose_topic": f"/{ns}/assigned_pose"},
                    {"odom_topic": f"/{ns}/ekf/odom"},
                    {"navigate_action": f"/{ns}/navigate_to_hover"},
                    {"forward_action": f"/{ns}/move_forward"},
                    {"speed": speed},
                    {"hover_sec": hover_sec},
                    {"hover_first": hover_first},
                    {"oneshot": oneshot},
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "drones",
            default_value="drone1,drone2,drone3",
            description="Comma-separated drone namespaces (e.g. 'drone1,drone2,drone3')",
        ),
        DeclareLaunchArgument(
            "speed",
            default_value="0.20",
            description="MoveForward speed (m/s)",
        ),
        DeclareLaunchArgument(
            "hover_sec",
            default_value="2",
            description="Hover duration for NavigateToHover action (sec)",
        ),
        DeclareLaunchArgument(
            "hover_first",
            default_value="true",
            description="If true: hover then move; else: move then hover",
        ),
        DeclareLaunchArgument(
            "oneshot",
            default_value="false",
            description="If true: run once then exit; if false: stays alive (still acts once in current client unless you extend it)",
        ),
        OpaqueFunction(function=_make_nodes),
    ])
