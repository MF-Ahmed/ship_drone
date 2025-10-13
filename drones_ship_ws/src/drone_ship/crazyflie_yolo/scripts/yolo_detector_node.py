#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
from vision_msgs.msg import BoundingBox2D, Pose2D 

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector_node')

        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("min_confidence", 0.90)  


        model_path = self.get_parameter("model_path").get_parameter_value().string_value

        try:
            self.min_conf = float(self.get_parameter("min_confidence").value)   # FIX: always set self.min_conf
        except Exception:
            self.min_conf = 0.90   

        self.model = YOLO(model_path)#.to('cpu')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'downward_left_camera/image_raw',
            self.image_callback,
            10)

        self.publisher = self.create_publisher(Detection2DArray, 'yolo_detections', 10)
        self.annotated_image_pub = self.create_publisher(Image, 'yolo/image_annotated', 10)

        self.get_logger().info("YOLOv8 Detector Node started.")

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return
        #self.get_logger().info("Received image frame.")
        results = self.model(cv_image, verbose=False)[0]
        #self.get_logger().info(f"Detections: {len(results.boxes) if results.boxes is not None else 0}")
        #self.get_logger().info(f"Incoming camera frame_id: {msg.header.frame_id}")


        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        detections_msg.header.frame_id = f"{self.get_namespace()}/downward_left_camera_link"

       

        YOLO_CLASSES = self.model.names  # Get class names from the model
        annotated_image = cv_image.copy()
        kept = 0
        dropped = 0


        for det in results.boxes:
          detection = Detection2D()
          detection.header = msg.header
          detection.header.frame_id = f"{self.get_namespace()}/downward_left_camera_link"

          x_center = float(det.xywh[0][0])
          y_center = float(det.xywh[0][1])
          width = float(det.xywh[0][2])
          height = float(det.xywh[0][3])

          # Print detection details
          class_id = int(det.cls[0])
          score = float(det.conf[0])

          class_name = YOLO_CLASSES[class_id]
          #self.get_logger().info(f"Detected: ID={class_id} ({class_name}), score={score:.2f}, bbox=[{x_center:.1f}, {y_center:.1f}, {width:.1f}, {height:.1f}]")


          # NEW: filter by confidence
          if score < self.min_conf:
             dropped += 1
             continue


          bbox = BoundingBox2D()
          #bbox.center = Pose2D(x=x_center, y=y_center, theta=0.0)
          bbox.center.position.x = x_center
          bbox.center.position.y = y_center
          bbox.size_x = width
          bbox.size_y = height
          detection.bbox = bbox

          hypothesis = ObjectHypothesisWithPose()
          class_id = int(det.cls[0])
          hypothesis.hypothesis.class_id = str(class_id)
          hypothesis.hypothesis.score = float(det.conf[0])
          detection.results.append(hypothesis)

          detections_msg.detections.append(detection)

            # Add bounding box
          x1, y1, x2, y2 = map(int, det.xyxy[0])
          cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
          cv2.putText(annotated_image, f"{class_name} {score:.2f}", (x1, y1 - 10),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.publisher.publish(detections_msg)
        # Convert and publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        annotated_msg.header = msg.header
        annotated_msg.header.frame_id = f"{self.get_namespace()}/downward_left_camera_link"
        self.annotated_image_pub.publish(annotated_msg)
        #self.get_logger().info("Published annotated image")
        if (kept + dropped) > 0:
            pass
            #self.get_logger().info(
                #f"Detections: kept={kept}, dropped<{self.min_conf:.2f}={dropped}"
            #)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()