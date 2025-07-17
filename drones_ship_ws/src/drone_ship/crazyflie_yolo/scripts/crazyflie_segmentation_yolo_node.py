#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ContainerROIDetector(Node):
    def __init__(self):
        super().__init__('container_roi_detector')
        self.bridge = CvBridge()

        # Subscribe to drone camera feed
        self.subscription = self.create_subscription(
            Image,
            '/crazyflie/downward_left_camera/image_raw',
            self.image_callback,
            10)

        self.get_logger().info("Container ROI detector node started.")

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply adaptive thresholding to highlight dark areas
        thresh = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 10)

        # Apply Canny edge detection
        edges = cv2.Canny(thresh, 50, 150)

        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Fixed container size in pixels (approximate)
        container_width_px = 150
        container_height_px = 50

        for cnt in contours:
            # Approximate contour to polygon
            epsilon = 0.02 * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)

            # Filter for rectangular shapes
            if len(approx) == 4 and cv2.isContourConvex(approx):
                x, y, w, h = cv2.boundingRect(approx)

                # Skip small or excessively large boxes
                if w < 40 or h < 20 or w > 500 or h > 500:
                    continue

                # Center and draw fixed-size ROI box
                center_x = x + w // 2
                center_y = y + h // 2
                top_left = (center_x - container_width_px // 2, center_y - container_height_px // 2)
                bottom_right = (center_x + container_width_px // 2, center_y + container_height_px // 2)

                cv2.rectangle(frame, top_left, bottom_right, (0, 255, 0), 2)
                cv2.putText(frame, "Container ROI", (top_left[0], top_left[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Display results
        cv2.imshow("Original Frame", frame)
        cv2.imshow("Thresholded", thresh)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ContainerROIDetector()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
