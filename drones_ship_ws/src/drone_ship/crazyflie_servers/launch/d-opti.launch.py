#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    #
    # Launch args
    #
    drone_ns_csv = DeclareLaunchArgument(
        "drone_ns_list",
        default_value="drone1,drone2,drone3",
        description="Comma-separated list of drone namespaces (without leading slash)"
    )
    world_frame = DeclareLaunchArgument(
        "world_frame",
        default_value="world",
        description="Global/world frame id"
    )
    fused_topic = DeclareLaunchArgument(
        "fused_topic",
        default_value="fused_tracked_obstacles_array",
        description="Fused obstacles topic (world frame)"
    )
    # Patrol defaults (applied per drone; you can override per drone by cloning a group)
    surveillance_hover_h = DeclareLaunchArgument(
        "surveillance_hover_h", default_value="10.0"
    )
    surveillance_forward = DeclareLaunchArgument(
        "surveillance_forward", default_value="10.0"
    )
    forward_speed = DeclareLaunchArgument(
        "forward_speed", default_value="1.5"
    )
    hover_dwell_s = DeclareLaunchArgument(
        "hover_dwell_s", default_value="2.0"
    )

    # D-optimality metrics node params
    ring_radius = DeclareLaunchArgument("ring_radius", default_value="4.0")
    ring_height = DeclareLaunchArgument("ring_height", default_value="6.0")
    ring_L = DeclareLaunchArgument("ring_L", default_value="12")
    safe_radius = DeclareLaunchArgument("safe_radius", default_value="0.5")
    tmax_sec = DeclareLaunchArgument("tmax_sec", default_value="10.0")
    publish_markers = DeclareLaunchArgument("publish_markers", default_value="true")
    publish_best_hover = DeclareLaunchArgument("publish_best_hover", default_value="true")

    #
    # Helpers
    #
    drone_ns_list = LaunchConfiguration("drone_ns_list")
    world = LaunchConfiguration("world_frame")
    fused = LaunchConfiguration("fused_topic")

    # Build per-drone groups at runtime (small helper)
    def per_drone_group(ns_name: str):
        """
        Creates a namespace group with:
          - navigate_to_hover_server
          - move_forward_server
          - surveillance_patrol_client
        Namespacing ensures topics are /<ns>/...
        """
        return GroupAction([
            PushRosNamespace(ns_name),

            # --- Servers (namespaced) ---
            Node(
                package="crazyflie_servers",
                executable="navigate_to_hover_server",
                name="navigate_to_hover_server",
                output="screen",
                parameters=[
                    # add server-specific params here if you have any
                ],
            ),
            Node(
                package="crazyflie_servers",
                executable="move_forward_server",
                name="move_forward_server",
                output="screen",
                parameters=[
                    # add server-specific params here if you have any
                ],
            ),

            # --- Patrol/Tracking client (namespaced) ---
            Node(
                package="crazyflie_servers",
                executable="surveillance_patrol_client",
                name="surveillance_patrol_client",
                output="screen",
                parameters=[{
                    # IMPORTANT: this client expects absolute drone_ns starting with '/'
                    "drone_ns": f"/{ns_name}",
                    "odom_topic": f"/{ns_name}/ekf/odom",
                    "assign_topic": f"/{ns_name}/assign_target",
                    "fused_topic": fused,
                    # Patrol behavior
                    "surveillance_hover_h": LaunchConfiguration("surveillance_hover_h"),
                    "surveillance_forward": LaunchConfiguration("surveillance_forward"),
                    "forward_speed": LaunchConfiguration("forward_speed"),
                    "hover_dwell_s": LaunchConfiguration("hover_dwell_s"),
                }],
            ),
        ])

    #
    # Build all per-drone groups from CSV list
    #
    # We can't easily split CSV inside pure launch descriptions,
    # so just repeat the group for the default 3 drones. If you
    # want a dynamic splitter, create a small Python shim or
    # pass explicit args and copy the group.
    #
    ns_csv_default = "drone1,drone2,drone3"
    ns_tokens = [t.strip() for t in ns_csv_default.split(",") if t.strip()]

    per_drone_groups = [per_drone_group(ns) for ns in ns_tokens]

    #
    # Global / shared nodes (non-namespaced)
    #
    global_nodes = [
        # D-optimality metrics / hover selector (world frame)
        Node(
            package="crazyflie_servers",
            executable="dopt_tracking_metrics",
            name="dopt_tracking_metrics",
            output="screen",
            parameters=[{
                "world_frame": world,
                "fused_topic": fused,
                "ring_radius": LaunchConfiguration("ring_radius"),
                "ring_height": LaunchConfiguration("ring_height"),
                "ring_L": LaunchConfiguration("ring_L"),
                "safe_radius": LaunchConfiguration("safe_radius"),
                "tmax_sec": LaunchConfiguration("tmax_sec"),
                "publish_markers": LaunchConfiguration("publish_markers"),
                "publish_best_hover": LaunchConfiguration("publish_best_hover"),
            }],
        ),

        # (Optional) Assignment node.
        # If your AssignmentNode executable lives in crazyflie_servers:
        # Node(
        #     package="crazyflie_servers",
        #     executable="assignment_node",
        #     name="assignment_node",
        #     output="screen",
        #     parameters=[{
        #         # Fill in your assignment node params here (eta, beta, ...),
        #         # and odom topic list for ALL drones:
        #         "drone_odom_topics": ["/drone1/ekf/odom", "/drone2/ekf/odom", "/drone3/ekf/odom"],
        #         "alloc_rate_hz": 1.0,
        #         # any CSV logging paths you want:
        #         # "assignment_map_csv_path": "/tmp/assignment_map.csv",
        #     }],
        # ),

        # (Optional) Multi-drone fusion node (if it’s in another package, change package/executable)
        # Node(
        #     package="crazyflie_yolo",
        #     executable="multi_drone_fusion_node",
        #     name="multi_drone_fusion_node",
        #     output="screen",
        #     parameters=[{
        #         "world_frame": world,
        #         "aquabot_frame": "aquabot/base_link",
        #         "association_threshold": 40.0,
        #     }],
        # ),
    ]

    return LaunchDescription([
        drone_ns_csv, world_frame, fused_topic,
        surveillance_hover_h, surveillance_forward, forward_speed, hover_dwell_s,
        ring_radius, ring_height, ring_L, safe_radius, tmax_sec, publish_markers, publish_best_hover,
        *per_drone_groups,
        *global_nodes,
    ])
