#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class GeodesicCenterPublisher(Node):
    def __init__(self):
        super().__init__('geodesic_center_publisher')
        self.publisher_ = self.create_publisher(Marker, '/geodesic_center_marker', 10)

        self.marker = Marker()
        self.marker.header.frame_id = 'map'
        self.marker.ns = 'geodesic'
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD
        self.marker.pose.position.x = 352710.0
        self.marker.pose.position.y = 5323335.0
        self.marker.pose.position.z = 0.0
        self.marker.scale.x = 5.0
        self.marker.scale.y = 5.0
        self.marker.scale.z = 5.0
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Geodesic marker publishing every second')

    def timer_callback(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.marker)

def main():
    rclpy.init()
    node = GeodesicCenterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
