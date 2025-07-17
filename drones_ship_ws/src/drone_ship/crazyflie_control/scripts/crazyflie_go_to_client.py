#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ls2n_drone_interfaces.action import GoTo
from geometry_msgs.msg import Pose

class CrazyflieGoToClient(Node):

    def __init__(self):
        super().__init__('crazyflie_go_to_client')
        self._action_client = ActionClient(self, GoTo, '/Crazy2fly1/go_to')

    def send_goal(self, x, y, z):
        goal_msg = GoTo.Goal()
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = 0.0
        goal_msg.pose.orientation.w = 1.0
        goal_msg.priority = 1
        goal_msg.relative = False
        goal_msg.group_mask = 0

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: ({x}, {y}, {z})')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = CrazyflieGoToClient()
    client.send_goal(0.0, 0.0, 8.0)  # ascend to 8m
    rclpy.spin(client)

if __name__ == '__main__':
    main()

