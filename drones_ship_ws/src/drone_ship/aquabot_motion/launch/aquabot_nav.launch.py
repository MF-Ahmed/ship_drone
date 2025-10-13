from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Thruster controller (global /cmd_vel)
        Node(package='aquabot_motion', executable='cmd.py', name='control',

             # uncomment if your sim uses /clock
             #parameters=[{'use_sim_time': True, 'unicycle': False}],
             parameters=[{'use_sim_time': True}],
             #remappings=[('odom', '/aquabot/ekf/odometry')],   the aquabot still moves afer reaching the goal perhaps need to tune the EKF
             remappings=[('odom', '/aquabot/odometry')], 

        ),
        # Planner
        Node(package='aquabot_motion', executable='planner.py', name='planner',
             parameters=[{'use_sim_time': True}],
        ),
        # Path follower
        Node(package='aquabot_motion', executable='path_follower.py', name='path_follower',
             parameters=[{'use_sim_time': True,
                          'lookahead_distance': 1.5,
                          'linear_speed': 1.0}],
        ),
    ])

