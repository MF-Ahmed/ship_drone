#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist
from math import atan2, sqrt

class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # Parameters
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('linear_speed', 1.0)
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value

        # Path and pose
        self.path = []
        self.current_pose = None
        self.current_index = 0

        # Subscriptions
        self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

    def plan_callback(self, msg):
        self.path = msg.poses
        self.current_index = 0
        self.get_logger().info(f'Received path with {len(self.path)} points.')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if not self.path or self.current_pose is None:
            return

        # Current position
        x = self.current_pose.position.x
        y = self.current_pose.position.y

        # Find the next target point ahead of us
        while self.current_index < len(self.path):
            target = self.path[self.current_index].pose.position
            dx = target.x - x
            dy = target.y - y
            dist = sqrt(dx**2 + dy**2)
            if dist >= self.lookahead_distance:
                break
            self.current_index += 1

        if self.current_index >= len(self.path):
            self.get_logger().info('Path complete. Stopping.')
            self.cmd_pub.publish(Twist())  # Stop
            return

        # Compute angle to target
        angle = atan2(dy, dx)

        # Publish velocity command
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = angle  # crude heading (can be improved with PID or heading error)
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
