#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D
from cv_bridge import CvBridge

from ultralytics import YOLO
import cv2
import numpy as np

from message_filters import Subscriber, ApproximateTimeSynchronizer


class YOLODetectorRGBD(Node):
    def __init__(self):
        super().__init__('yolo_detector_rgbd_node')

        # ---------------- Parameters ----------------
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("min_confidence", 0.80)
        self.declare_parameter("iou", 0.45)
        self.declare_parameter("imgsz", 832)
        self.declare_parameter("max_det", 50)

        # Topics
        self.declare_parameter("rgb_topic", "downward_left_camera/image_raw")
        self.declare_parameter("disp_topic", "disparity/image")  # <-- set to your disparity topic
        self.declare_parameter("detections_topic", "yolo_detections")
        self.declare_parameter("annotated_image_topic", "yolo/image_annotated")

        # Disparity normalization behavior
        # - "auto": normalize per-frame min/max to [0,255] (good for visualization, may vary scale)
        # - "clip": clip to [disp_min, disp_max] then scale to [0,255] (more stable)
        self.declare_parameter("disp_norm_mode", "clip")
        self.declare_parameter("disp_min", 0.0)
        self.declare_parameter("disp_max", 128.0)

        # Sync
        self.declare_parameter("sync_slop", 0.10)  # seconds
        self.declare_parameter("sync_queue", 20)

        # ---------------- Load params ----------------
        self.model_path = self.get_parameter("model_path").value
        self.min_conf = float(self.get_parameter("min_confidence").value)
        self.iou = float(self.get_parameter("iou").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.max_det = int(self.get_parameter("max_det").value)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.disp_topic = self.get_parameter("disp_topic").value
        self.detections_topic = self.get_parameter("detections_topic").value
        self.annotated_topic = self.get_parameter("annotated_image_topic").value

        self.disp_norm_mode = self.get_parameter("disp_norm_mode").value
        self.disp_min = float(self.get_parameter("disp_min").value)
        self.disp_max = float(self.get_parameter("disp_max").value)

        sync_slop = float(self.get_parameter("sync_slop").value)
        sync_queue = int(self.get_parameter("sync_queue").value)

        # ---------------- Model ----------------
        self.model = YOLO(self.model_path)
        self.bridge = CvBridge()

        # ---------------- Subscribers (SYNC) ----------------
        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.disp_sub = Subscriber(self, Image, self.disp_topic)

        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.disp_sub],
            queue_size=sync_queue,
            slop=sync_slop
        )
        self.sync.registerCallback(self.synced_callback)

        # ---------------- Publishers ----------------
        self.det_pub = self.create_publisher(Detection2DArray, self.detections_topic, 10)
        self.ann_pub = self.create_publisher(Image, self.annotated_topic, 10)

        self.get_logger().info("✅ YOLOv8 RGBD Detector Node started.")
        self.get_logger().info(f"   model_path: {self.model_path}")
        self.get_logger().info(f"   rgb_topic : {self.rgb_topic}")
        self.get_logger().info(f"   disp_topic: {self.disp_topic}")
        self.get_logger().info(f"   conf={self.min_conf:.2f}, iou={self.iou:.2f}, imgsz={self.imgsz}, max_det={self.max_det}")
        self.get_logger().info(f"   disp_norm_mode={self.disp_norm_mode}, disp_min={self.disp_min}, disp_max={self.disp_max}")
        self.get_logger().info(f"   sync_slop={sync_slop}s, sync_queue={sync_queue}")

    # -------- disparity conversion helpers --------
    def disparity_to_uint8(self, disp_msg: Image) -> np.ndarray:
        """
        Convert disparity Image msg to uint8 HxW suitable as 4th channel.
        Supports: mono8, 16UC1, 32FC1, 64FC1 (via cv_bridge as passthrough).
        """
        # Try to keep raw type
        try:
            disp = self.bridge.imgmsg_to_cv2(disp_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CVBridge disparity error: {e}")
            return None

        if disp is None:
            return None

        # Ensure 2D
        if disp.ndim == 3:
            disp = disp[:, :, 0]

        # If already uint8, use it directly (mono8)
        if disp.dtype == np.uint8:
            return disp

        # Convert to float32 for scaling
        disp_f = disp.astype(np.float32)

        # Handle NaNs/Infs
        disp_f = np.nan_to_num(disp_f, nan=0.0, posinf=0.0, neginf=0.0)

        if self.disp_norm_mode == "auto":
            # Per-frame normalize min/max -> [0,255]
            mn = float(np.min(disp_f))
            mx = float(np.max(disp_f))
            if mx - mn < 1e-6:
                return np.zeros_like(disp_f, dtype=np.uint8)
            out = (disp_f - mn) / (mx - mn) * 255.0
            return out.clip(0, 255).astype(np.uint8)

        # "clip" mode: stable mapping
        dmin = self.disp_min
        dmax = self.disp_max if self.disp_max > self.disp_min else (self.disp_min + 1.0)
        out = (disp_f - dmin) / (dmax - dmin) * 255.0
        return out.clip(0, 255).astype(np.uint8)

    def synced_callback(self, rgb_msg: Image, disp_msg: Image):
        # -------- RGB conversion --------
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge RGB error: {e}")
            return

        # -------- Disparity conversion --------
        disp_u8 = self.disparity_to_uint8(disp_msg)
        if disp_u8 is None:
            return

        # Ensure same resolution
        if disp_u8.shape[0] != rgb.shape[0] or disp_u8.shape[1] != rgb.shape[1]:
            # Resize disparity to match rgb
            disp_u8 = cv2.resize(disp_u8, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)

        # -------- Build 4-channel RGBD (B,G,R,D) --------
        rgbd = cv2.merge([rgb[:, :, 0], rgb[:, :, 1], rgb[:, :, 2], disp_u8])  # uint8 HxWx4

        # -------- Run YOLO on 4-channel image --------
        results = self.model(
            rgbd,
            verbose=False,
            imgsz=self.imgsz,
            conf=self.min_conf,
            iou=self.iou,
            max_det=self.max_det
        )[0]

        detections_msg = Detection2DArray()
        detections_msg.header = rgb_msg.header
        detections_msg.header.frame_id = f"{self.get_namespace()}/downward_left_camera_link"

        YOLO_CLASSES = self.model.names
        annotated = rgb.copy()

        if results.boxes is not None:
            for det in results.boxes:
                detection = Detection2D()
                detection.header = rgb_msg.header
                detection.header.frame_id = f"{self.get_namespace()}/downward_left_camera_link"

                x_center = float(det.xywh[0][0])
                y_center = float(det.xywh[0][1])
                width = float(det.xywh[0][2])
                height = float(det.xywh[0][3])

                class_id = int(det.cls[0])
                score = float(det.conf[0])
                class_name = YOLO_CLASSES[class_id]

                bbox = BoundingBox2D()
                bbox.center.position.x = x_center
                bbox.center.position.y = y_center
                bbox.size_x = width
                bbox.size_y = height
                detection.bbox = bbox

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_id)
                hypothesis.hypothesis.score = score
                detection.results.append(hypothesis)

                detections_msg.detections.append(detection)

                # Draw on RGB visualization
                x1, y1, x2, y2 = map(int, det.xyxy[0])
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f"{class_name} {score:.2f}", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Publish detections
        self.det_pub.publish(detections_msg)

        # Publish annotated RGB image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = rgb_msg.header
        annotated_msg.header.frame_id = f"{self.get_namespace()}/downward_left_camera_link"
        self.ann_pub.publish(annotated_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorRGBD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
