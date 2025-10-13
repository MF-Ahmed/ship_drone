#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
from math import atan2, sqrt, sin, cos, pi
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration




ME = 'aquabot/base_link'
WORLD = 'world'

def wrap_to_pi(a):
    while a >  pi: a -= 2*pi
    while a < -pi: a += 2*pi
    return a

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        # Parameters
        self.declare_parameter('lookahead_distance', 1.5)
        self.declare_parameter('linear_speed', 1.0)
        self.declare_parameter('k_omega', 1.5)
        self.declare_parameter('arrive_dist', 0.8)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.k_omega = float(self.get_parameter('k_omega').value)
        self.arrive_dist = float(self.get_parameter('arrive_dist').value)

        # TF buffer/listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Path state
        self.path = []
        self.current_index = 0

        # IO
        self.create_subscription(Path, '/plan', self.plan_callback, 10)  # planner publishes 'world' frame
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.traj_pub = self.create_publisher(Path, '/aquabot/trajectory', 10)
        self.trajectory = Path()
        self.trajectory.header.frame_id = 'world'
        # Control loop
        self.create_timer(0.1, self.control_loop)

    def plan_callback(self, msg: Path):
        # Expect poses in WORLD
        self.path = msg.poses
        self.current_index = 0
        self.get_logger().info(f'Received path with {len(self.path)} points (frame={msg.header.frame_id}).')

    def get_pose_in_world(self):
        now = rclpy.time.Time()
        if not self.tf_buffer.can_transform(WORLD, ME, now):
            return None
        tf = self.tf_buffer.lookup_transform(WORLD, ME, now).transform
        x = tf.translation.x
        y = tf.translation.y
        # yaw from quaternion (z,w used by your codebase)
        z = tf.rotation.z
        w = tf.rotation.w
        yaw = 2.0 * atan2(z, w)
        return x, y, yaw
        
        
        
    def publish_trajectory_point(self, x, y, yaw):
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        # yaw just for visualization (z,w format used elsewhere)
        from math import sin, cos
        ps.pose.orientation.z = sin(0.5*yaw)
        ps.pose.orientation.w = cos(0.5*yaw)
        self.trajectory.poses.append(ps)
        self.trajectory.header.stamp = ps.header.stamp
        self.traj_pub.publish(self.trajectory)

        

    def control_loop(self):
        if not self.path:
            return

        pose = self.get_pose_in_world()
        if pose is None:
            return
        x, y, yaw = pose
        
        # in control_loop(), right after you compute (x, y, yaw):
        self.publish_trajectory_point(x, y, yaw)   

        # Find target point ahead by lookahead_distance
        while self.current_index < len(self.path):
            target = self.path[self.current_index].pose.position
            dx = target.x - x
            dy = target.y - y
            dist = sqrt(dx*dx + dy*dy)
            if dist >= self.lookahead_distance:
                break
            self.current_index += 1

        # If we’ve consumed the path, stop
        if self.current_index >= len(self.path):
            # Check final target distance for arrive behavior
            last = self.path[-1].pose.position
            if sqrt((last.x - x)**2 + (last.y - y)**2) <= self.arrive_dist:
                self.get_logger().info('Goal reached. Stopping.')
            else:
                self.get_logger().info('Path ended; close to goal. Stopping.')
            self.cmd_pub.publish(Twist())
            return

        # Steering to the lookahead target
        target = self.path[self.current_index].pose.position
        dx = target.x - x
        dy = target.y - y
        dist = sqrt(dx*dx + dy*dy)

        # Desired heading in world
        desired_yaw = atan2(dy, dx)
        yaw_err = wrap_to_pi(desired_yaw - yaw)

        # Speeds
        cmd = Twist()
        # Reduce forward speed if turning sharply; stop when basically on top of the target
        cmd.linear.x = self.linear_speed * max(0.1, 1.0 - min(1.0, abs(yaw_err)/1.0))
        if dist < self.arrive_dist:
            cmd.linear.x = 0.0

        cmd.angular.z = self.k_omega * yaw_err
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

