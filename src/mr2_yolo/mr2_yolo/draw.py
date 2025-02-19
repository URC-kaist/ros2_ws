import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time

class ImageProcessor(Node):
    def __init__(self):
        super().__init__('image_processor')

        # Throttle rate (5 FPS → 0.2s per frame)
        self.last_time = time.time()
        self.frame_interval = 1.0 / 5.0  # 5 FPS

        # Create a CV Bridge
        self.bridge = CvBridge()

        # Define QoS for reliable image transport
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber to input image topic
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Change to your actual topic
            self.image_callback,
            qos_profile
        )

        # Publisher for processed image
        self.publisher = self.create_publisher(Image, '/camera/image_processed', 10)

    def image_callback(self, msg):
        """ Callback function to process images. """
        current_time = time.time()
        if current_time - self.last_time < self.frame_interval:
            return  # Throttle to 5 FPS
        self.last_time = current_time

        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Draw bounding box (example: centered in the image)
        height, width, _ = cv_image.shape
        start_point = (width // 4, height // 4)  # Top-left
        end_point = (3 * width // 4, 3 * height // 4)  # Bottom-right
        color = (0, 255, 0)  # Green
        thickness = 2

        cv2.rectangle(cv_image, start_point, end_point, color, thickness)

        # Convert back to ROS Image and publish
        processed_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        processed_msg.header = msg.header  # Keep the same timestamp
        self.publisher.publish(processed_msg)

        self.get_logger().info("Published processed image with bounding box")

def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
