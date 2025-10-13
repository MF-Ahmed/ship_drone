#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from crazyflie_control.msg import AssignTarget
from crazyflie_control.action import NavigateToHover

class ModeMux(Node):
    def __init__(self):
        super().__init__('assign_or_user_to_hover_adapter')

        # Params
        self.declare_parameter('navigate_action', 'navigate_to_hover')
        self.declare_parameter('assign_topic', 'assign_target')
        self.declare_parameter('user_target_topic', 'user_target')
        self.declare_parameter('mode_topic', 'mode')
        self.declare_parameter('default_hover_time_s', 2.0)

        self.action_name = self.get_parameter('navigate_action').get_parameter_value().string_value
        self.assign_topic = self.get_parameter('assign_topic').get_parameter_value().string_value
        self.user_topic = self.get_parameter('user_target_topic').get_parameter_value().string_value
        self.mode_topic = self.get_parameter('mode_topic').get_parameter_value().string_value
        self.hover_s = float(self.get_parameter('default_hover_time_s').value)

        # Action client
        self.client = ActionClient(self, NavigateToHover, self.action_name)

        # State
        self.mode = 'surveillance'  # default
        self.active_goal = None

        # Subs
        self.create_subscription(String, self.mode_topic, self.on_mode, 10)
        self.create_subscription(AssignTarget, self.assign_topic, self.on_assign, 10)
        self.create_subscription(PoseStamped, self.user_topic, self.on_user, 10)

        self.get_logger().info(
            f"ModeMux up. mode_topic='{self.mode_topic}', user='{self.user_topic}', "
            f"assign='{self.assign_topic}', action='{self.action_name}'"
        )

    def on_mode(self, msg: String):
        new_mode = msg.data.strip().lower()
        if new_mode not in ('surveillance', 'tracking'):
            self.get_logger().warn(f"Unknown mode '{new_mode}' (expected 'surveillance'|'tracking')")
            return
        if new_mode != self.mode:
            self.get_logger().info(f"Mode change: {self.mode} -> {new_mode}")
            self.mode = new_mode
            self.cancel_active_goal()

    def on_user(self, pose: PoseStamped):
        if self.mode != 'surveillance':
            return
        self.send_goal_from_pose(pose)

    def on_assign(self, tgt: AssignTarget):
        if self.mode != 'tracking':
            return
        pose = PoseStamped()
        pose.header.frame_id = pose.header.frame_id or "world"
        pose.pose = tgt.hover_pose_world
        self.send_goal_from_pose(pose)

    def cancel_active_goal(self):
        if self.active_goal and self.active_goal.accepted:
            self.get_logger().info("Canceling active goal")
            self.active_goal.cancel_goal_async()
        self.active_goal = None

    def send_goal_from_pose(self, pose: PoseStamped):
        if not self.client.wait_for_server(timeout_sec=0.5):
            self.get_logger().warn("navigate_to_hover action not available yet")
            return

        self.cancel_active_goal()

        goal = NavigateToHover.Goal()
        goal.target_pose = pose.pose
        goal.yaw = 0.0
        goal.hover_time = Duration(seconds=self.hover_s).to_msg()

        self.get_logger().info(
            f"Send goal ({self.mode}): "
            f"p=({pose.pose.position.x:.2f},{pose.pose.position.y:.2f},{pose.pose.position.z:.2f}) "
            f"hover={self.hover_s:.1f}s"
        )

        send_future = self.client.send_goal_async(goal,
            feedback_callback=self.on_feedback)
        send_future.add_done_callback(self.on_goal_sent)

    def on_goal_sent(self, fut):
        self.active_goal = fut.result()
        if not self.active_goal.accepted:
            self.get_logger().warn("Goal rejected")
            return
        self.get_logger().info("Goal accepted")
        self.active_goal.get_result_async().add_done_callback(self.on_result)

    def on_feedback(self, fb):
        f = fb.feedback
        # Optional debug:
        # self.get_logger().debug(f"dist={f.distance_to_target:.2f} hover_left={f.remaining_hover_s:.2f}")

    def on_result(self, fut):
        res = fut.result().result
        self.get_logger().info(f"Result: success={res.success} msg='{res.message}'")
        self.active_goal = None

def main():
    rclpy.init()
    node = ModeMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

