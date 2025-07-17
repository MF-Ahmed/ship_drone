#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class CrazyflieOdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('crazyflie_odom_tf')
        self.declare_parameter('odom_topic', '/crazyflie/odom')
        self.odom_topic = self.get_parameter('odom_topic').value

        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f'Broadcasting TF from world to crazyflie/odom using topic {self.odom_topic}')

    def odom_callback(self, msg: Odometry):
        t1 = TransformStamped()
        t1.header.stamp = msg.header.stamp
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'crazyflie/odom'
        t1.transform.translation.x = msg.pose.pose.position.x
        t1.transform.translation.y = msg.pose.pose.position.y
        t1.transform.translation.z = msg.pose.pose.position.z
        t1.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t1)
        
        # Transform from crazyflie/odom → crazyflie/base_footprint (zero offset)
        t2 = TransformStamped()
        t2.header.stamp = msg.header.stamp
        t2.header.frame_id = 'crazyflie/odom'
        t2.child_frame_id = 'crazyflie/base_footprint'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        # Identity quaternion
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0  # identity quaternion
        self.br.sendTransform(t2)        
        
        
        
        


def main(args=None):
    rclpy.init(args=args)
    node = CrazyflieOdomTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
