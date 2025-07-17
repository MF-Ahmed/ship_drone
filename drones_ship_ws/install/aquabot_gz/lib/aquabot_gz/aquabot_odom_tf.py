#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aquabot_odom_tf')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, '/aquabot/odom', self.odom_callback, 10)
        self.get_logger().info("Broadcasting TF from world → aquabot/odom using /aquabot/odom")

    def odom_callback(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'world'
        t.child_frame_id = 'aquabot/odom'

        # If your odometry includes position data, use that.
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
