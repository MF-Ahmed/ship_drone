from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # --- Multi-drone fusion node ---
        Node(
            package="crazyflie_yolo",
            executable="multi_drone_fusion_node",
            name="multi_drone_fusion_node",
            output="screen",
            parameters=[{
                "drone_names": ["drone1", "drone2", "drone3"],
                "topic_format": "/%s/tracked_obstacles_array",
                "association_threshold": 40.0,
                "prune_time_s": 5.0,
                "aquabot_frame": "aquabot/base_link",
                "log_path": "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/fused_obstacles.csv",
                "log_append": False,
                "use_sim_time": True
            }]
        ),

        # --- Hungarian assignment node ---
        Node(
            package="crazyflie_yolo",
            executable="assignment_node",
            name="assignment_node",
            output="screen",
            parameters=[{
                # Odometry sources
                "drone_odom_topics": [
                    "/drone1/ekf/odom",
                    "/drone2/ekf/odom",
                    "/drone3/ekf/odom"
                ],

                # Hungarian cost weights
                "alpha": 0.1,     # logdet uncertainty weight
                "beta": 1.0,      # travel distance weight
                "gamma": 0.2,     # separation penalty weight

                # Distance and gating thresholds
                "r_safe": 0.5,
                "d_max": 25.0,
                "maha_gate_sq": 9.0,
                "kappa": 1000.0,
                "unassigned_penalty": 25.0,

                # Hover target configuration
                "hover_radius": 1.5,
                "hover_height": 2.0,
                "min_alt": 1.0,
                "max_alt": 50.0,
                "safety_radius": 0.5,
                "timeout_s": 10.0,
                "policy": "topdown",

                # Timer rate and verbosity
                "alloc_rate_hz": 1.0,
                "verbose": True,
                "log_costs": True,
                "use_sim_time": True,
                "assignment_policy": "many_to_one",
                "per_target_capacity": 3,
                "log_costs": True,
                "cost_csv_path": "/home/user/data/drones_ship_ws/src/drone_ship/crazyflie_yolo/logs_fusion/cost_matrix.csv",
                "verbose": True,

            }],
            remappings=[
                ("fused_tracks", "/fused_tracked_obstacles_array"),
            ],
            # Optional: run with debug logging level
            #arguments=['--ros-args', '--log-level', 'assignment_node:=debug']
        ),

        # --- Static TF to fix world→aquabot/base_link ---
        #Node(
            #package="tf2_ros",
            #executable="static_transform_publisher",
            #arguments=["0", "0", "0", "0", "0", "0", "world", "aquabot/base_link"],
            #name="static_world_to_aquabot"
        #),
    ])
