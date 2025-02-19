#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/detection_info', 10)
        self.bridge = CvBridge()

        # Load the YOLO model (ensure the .pt file is in your package or provide a full path)
        package_share_dir = get_package_share_directory('mr2_yolo')
        model_path = os.path.join(package_share_dir, 'resources', 'model.pt')
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO detection node initialized.")

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to an OpenCV image (BGR format)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error("Failed to convert image: %s" % str(e))
            return

        # Run object detection on the image
        results = self.model(cv_image)
        detection_info = ""

        # Process the detection results.
        # The results object contains a list of result(s); each result has a 'boxes' attribute.
        for result in results:
            if result.boxes is not None:
                # Each row in result.boxes.data is expected to be:
                # [x1, y1, x2, y2, confidence, class]
                for box in result.boxes.data.tolist():
                    x1, y1, x2, y2, conf, cls = box
                    width = x2 - x1
                    height = y2 - y1
                    detection_info += (f"Class: {int(cls)}, Confidence: {conf:.2f}, "
                                       f"Box: ({x1:.0f}, {y1:.0f}, {width:.0f}, {height:.0f})\n")

        if detection_info == "":
            detection_info = "No detections."

        # Publish the detection information
        detection_msg = String()
        detection_msg.data = detection_info
        self.publisher.publish(detection_msg)
        self.get_logger().info("Published detection info.")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Shutdown the node gracefully
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
