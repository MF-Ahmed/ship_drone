#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from crazyflie_yolo.msg import AssignmentMetrics


def clean_ns(s: str) -> str:
    # "/drone1" -> "drone1"
    return s.strip("/")


class DoptPlotter(Node):
    """
    Subscribes: /assignment_metrics (AssignmentMetrics)
    Publishes:
      /droneX/dopt_primary  (Float64)   -> ΔJ_{j, i*} where i* is primary_track_index[j]
      /droneX/dopt_max      (Float64)   -> max_i ΔJ_{j,i}   (optional)
    """
    def __init__(self):
        super().__init__("dopt_plotter")

        self.metrics_topic = self.declare_parameter("metrics_topic", "/assignment_metrics").value
        self.publish_max = self.declare_parameter("publish_max", True).value

        self.sub = self.create_subscription(
            AssignmentMetrics,
            self.metrics_topic,
            self.cb,
            10
        )

        self.pub_primary = {}  # ns -> publisher
        self.pub_max = {}      # ns -> publisher

        self.get_logger().info(f"Listening on {self.metrics_topic}")

    def _get_pub(self, ns: str, kind: str):
        ns_clean = clean_ns(ns)
        if kind == "primary":
            if ns_clean not in self.pub_primary:
                topic = f"/{ns_clean}/dopt_primary"
                self.pub_primary[ns_clean] = self.create_publisher(Float64, topic, 10)
                self.get_logger().info(f"Publishing {topic}")
            return self.pub_primary[ns_clean]
        else:
            if ns_clean not in self.pub_max:
                topic = f"/{ns_clean}/dopt_max"
                self.pub_max[ns_clean] = self.create_publisher(Float64, topic, 10)
                self.get_logger().info(f"Publishing {topic}")
            return self.pub_max[ns_clean]

    def cb(self, msg: AssignmentMetrics):
        M = int(msg.num_drones)
        N = int(msg.num_targets)
        if M <= 0 or N <= 0:
            return
        if len(msg.dopt) != M * N:
            self.get_logger().warn(f"dopt size mismatch: got {len(msg.dopt)} expected {M*N}")
            return

        # If labels are missing, fall back to drone0..drone(M-1)
        drone_ns = list(msg.drone_ns) if len(msg.drone_ns) == M else [f"drone{j+1}" for j in range(M)]

        for j in range(M):
            ns = drone_ns[j]

            # ---- primary ΔJ_{j,i*} ----
            i_star = int(msg.primary_track_index[j]) if j < len(msg.primary_track_index) else -1
            val_primary = float("nan")
            if 0 <= i_star < N:
                val_primary = float(msg.dopt[j * N + i_star])

            self._get_pub(ns, "primary").publish(Float64(data=val_primary))

            # ---- max ΔJ_{j,i} (optional) ----
            if self.publish_max:
                row = [float(msg.dopt[j * N + i]) for i in range(N)]
                val_max = max(row) if row else float("nan")
                self._get_pub(ns, "max").publish(Float64(data=val_max))


def main():
    rclpy.init()
    node = DoptPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

