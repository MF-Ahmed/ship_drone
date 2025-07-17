#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import tf_transformations


class FakeOdom(Node):
    def __init__(self):
        super().__init__('fake_odom')

        # Publisher and Subscriber
        self.sub = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.imu_cb, 10)
        self.pub = self.create_publisher(Odometry, '/aquabot/odom', 10)

        self.get_logger().info("Fake Odom Node started. Subscribing to /aquabot/sensors/imu/imu/data")
        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_cb(self, msg: Imu):
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'aquabot/odom'
        odom.child_frame_id = 'aquabot/base_link'

        # Use orientation from IMU
        odom.pose.pose.orientation = msg.orientation

        # Fake twist from IMU data
        odom.twist.twist.angular = msg.angular_velocity
        odom.twist.twist.linear.x = msg.linear_acceleration.x  # This is not real velocity!

        self.pub.publish(odom)
         # Publish TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'aquabot/odom'
        t.child_frame_id = 'aquabot/base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
