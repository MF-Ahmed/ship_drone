#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Header

def d2(xy, ref): dx=xy[0]-ref[0]; dy=xy[1]-ref[1]; return dx*dx+dy*dy

class ContainersGT(Node):
    def __init__(self):
        super().__init__('containers_gt')
        self.sub = self.create_subscription(
            PoseArray,
            '/world/aquabot_windturbines_medium_new/pose/info',
            self.cb, 10
        )
        # publishers
        self.p4 = self.create_publisher(PoseStamped, '/container4/pose_gt', 10)
        self.p5 = self.create_publisher(PoseStamped, '/container5/pose_gt', 10)
        # params: expected spawn XY (use your launch args)
        self.c4_ref = [ self.declare_parameter('c4_x', 10.0).get_parameter_value().double_value,
                        self.declare_parameter('c4_y', -5.0).get_parameter_value().double_value ]
        self.c5_ref = [ self.declare_parameter('c5_x', 15.0).get_parameter_value().double_value,
                        self.declare_parameter('c5_y',  3.0).get_parameter_value().double_value ]
        self.c4_idx = None
        self.c5_idx = None

    def cb(self, msg: PoseArray):
        if not msg.poses: return

        # lock indices once, by nearest XY to refs (ignore Z)
        if self.c4_idx is None or self.c5_idx is None:
            best4 = (1e18, 0); best5 = (1e18, 0)
            for i, p in enumerate(msg.poses):
                xy = (p.position.x, p.position.y)
                d4 = d2(xy, self.c4_ref)
                d5 = d2(xy, self.c5_ref)
                if d4 < best4[0]: best4 = (d4, i)
                if d5 < best5[0]: best5 = (d5, i)
            self.c4_idx, self.c5_idx = best4[1], best5[1]
            self.get_logger().info(f'Locked container4 idx={self.c4_idx}, container5 idx={self.c5_idx}')

        # publish PoseStamped for each
        now = self.get_clock().now().to_msg()
        hdr = Header(stamp=now, frame_id='world')
        if 0 <= self.c4_idx < len(msg.poses):
            ps = PoseStamped(header=hdr, pose=msg.poses[self.c4_idx])
            self.p4.publish(ps)
        if 0 <= self.c5_idx < len(msg.poses):
            ps = PoseStamped(header=hdr, pose=msg.poses[self.c5_idx])
            self.p5.publish(ps)

def main():
    rclpy.init(); rclpy.spin(ContainersGT()); rclpy.shutdown()
if __name__ == '__main__': main()

