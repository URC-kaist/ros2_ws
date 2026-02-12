import math
import os
from typing import Optional, Tuple, List

import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from image_geometry import PinholeCameraModel
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import rclpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_geometry_msgs import do_transform_point
from message_filters import Subscriber, ApproximateTimeSynchronizer
from ultralytics import YOLO


class YoloRgbdDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_rgbd_detector")

        self.declare_parameter("rgb_topic", "/rgbd_camera/color/image_raw")
        self.declare_parameter("depth_topic", "/rgbd_camera/depth/image_rect_raw")
        self.declare_parameter("camera_info_topic", "/rgbd_camera/color/camera_info")
        self.declare_parameter("annotated_topic", "yolo/annotated_image")
        self.declare_parameter("pose_topic", "yolo/object_pose")
        self.declare_parameter("target_frame", "")
        self.declare_parameter("model_path", "yolov11.pt")
        self.declare_parameter("conf_threshold", 0.25)
        self.declare_parameter("iou_threshold", 0.45)
        self.declare_parameter("device", "")
        self.declare_parameter("target_class", "")
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("class_ids", [0, 1, 2])
        self.declare_parameter("class_id_map", "")
        self.declare_parameter("semantic_class_names", [])
        self.declare_parameter("camera_frame_is_optical", True)

        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        self.camera_info_topic = (
            self.get_parameter("camera_info_topic").get_parameter_value().string_value
        )
        self.annotated_topic = (
            self.get_parameter("annotated_topic").get_parameter_value().string_value
        )
        self.pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.model_path = self.get_parameter("model_path").get_parameter_value().string_value
        self.conf_threshold = (
            self.get_parameter("conf_threshold").get_parameter_value().double_value
        )
        self.iou_threshold = (
            self.get_parameter("iou_threshold").get_parameter_value().double_value
        )
        self.device = self.get_parameter("device").get_parameter_value().string_value
        self.target_class = (
            self.get_parameter("target_class").get_parameter_value().string_value
        )
        self.target_class_id = (
            self.get_parameter("target_class_id").get_parameter_value().integer_value
        )
        self.class_ids = list(
            self.get_parameter("class_ids").get_parameter_value().integer_array_value
        )
        self.class_id_map = self._parse_class_id_map(
            self.get_parameter("class_id_map").value
        )
        self.semantic_class_names = list(
            self.get_parameter("semantic_class_names")
            .get_parameter_value()
            .string_array_value
        )
        self.semantic_class_names = [
            name.strip().lower() for name in self.semantic_class_names if name.strip()
        ]
        self.semantic_name_to_id = {
            name: idx for idx, name in enumerate(self.semantic_class_names)
        }
        self.camera_frame_is_optical = (
            self.get_parameter("camera_frame_is_optical")
            .get_parameter_value()
            .bool_value
        )

        self.camera_model = PinholeCameraModel()
        self.camera_info_received = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        image_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.image_pub = self.create_publisher(Image, self.annotated_topic, image_qos)
        self.pose_pubs = {}
        for class_id in self._effective_class_ids():
            topic = f"{self.pose_topic}/class_{class_id}"
            self.pose_pubs[class_id] = self.create_publisher(PoseStamped, topic, 10)

        self.camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self._on_camera_info, 10
        )

        self.rgb_sub = Subscriber(self, Image, self.rgb_topic)
        self.depth_sub = Subscriber(self, Image, self.depth_topic)
        self.sync = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self._on_images)

        resolved_model_path = self._resolve_model_path(self.model_path)
        self.get_logger().info(f"Loading YOLO model: {resolved_model_path}")
        self.model = YOLO(resolved_model_path, task="detect")

    def _resolve_model_path(self, model_path: str) -> str:
        if model_path.startswith("package://"):
            suffix = model_path[len("package://") :]
            return os.path.join(get_package_share_directory("mr2_yolo_perception"), suffix)

        expanded = os.path.expanduser(model_path)
        if os.path.isabs(expanded):
            return expanded

        share_dir = get_package_share_directory("mr2_yolo_perception")
        return os.path.join(share_dir, "models", expanded)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self.camera_model.fromCameraInfo(msg)
        self.camera_info_received = True

    def _decode_image(self, msg: Image, dtype: np.dtype, channels: int) -> np.ndarray:
        itemsize = np.dtype(dtype).itemsize
        expected_step = msg.width * channels * itemsize
        if msg.step < expected_step:
            raise ValueError(
                f"Invalid image step for encoding {msg.encoding}: got {msg.step}, expected >= {expected_step}"
            )

        flat = np.frombuffer(msg.data, dtype=dtype)
        row_stride = msg.step // itemsize
        image = flat.reshape((msg.height, row_stride))
        image = image[:, : msg.width * channels]

        if channels == 1:
            image = image.reshape((msg.height, msg.width))
        else:
            image = image.reshape((msg.height, msg.width, channels))

        if msg.is_bigendian and itemsize > 1:
            image = image.byteswap().newbyteorder()

        return np.ascontiguousarray(image)

    def _rgb_from_msg(self, msg: Image) -> np.ndarray:
        if msg.encoding == "bgr8":
            return self._decode_image(msg, np.uint8, 3)
        if msg.encoding == "rgb8":
            rgb = self._decode_image(msg, np.uint8, 3)
            return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if msg.encoding == "mono8":
            mono = self._decode_image(msg, np.uint8, 1)
            return cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
        raise ValueError(f"Unsupported RGB encoding: {msg.encoding}")

    def _depth_from_msg(self, msg: Image) -> np.ndarray:
        if msg.encoding == "16UC1":
            return self._decode_image(msg, np.uint16, 1)
        if msg.encoding == "32FC1":
            return self._decode_image(msg, np.float32, 1)
        raise ValueError(f"Unsupported depth encoding: {msg.encoding}")

    def _bgr_to_imgmsg(self, image: np.ndarray, header) -> Image:
        if image.ndim != 3 or image.shape[2] != 3:
            raise ValueError("Annotated image must be HxWx3 BGR")
        image_u8 = np.ascontiguousarray(image, dtype=np.uint8)
        msg = Image()
        msg.header = header
        msg.height = int(image_u8.shape[0])
        msg.width = int(image_u8.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(image_u8.shape[1] * 3)
        msg.data = image_u8.tobytes()
        return msg

    def _select_detection(
        self, boxes, names
    ) -> Optional[Tuple[int, float, np.ndarray]]:
        if boxes is None or len(boxes) == 0:
            return None

        target_id = None
        if self.target_class_id >= 0:
            target_id = int(self.target_class_id)
        elif self.target_class:
            want = self.target_class.strip().lower()
            if self.semantic_name_to_id:
                target_id = self.semantic_name_to_id.get(want)
            else:
                for idx, name in names.items():
                    if name.strip().lower() == want:
                        target_id = int(idx)
                        break
            if target_id is None:
                # self.get_logger().warn(
                #     f"target_class '{self.target_class}' not found in model classes"
                # )
                pass

        best_idx = None
        best_conf = -1.0
        best_xyxy = None
        for i, box in enumerate(boxes):
            cls_id = int(box.cls[0])
            sem_id = self._semantic_id_for(cls_id, names)
            if sem_id is None:
                continue
            conf = float(box.conf[0])
            if target_id is not None and sem_id != target_id:
                continue
            if conf > best_conf:
                best_idx = sem_id
                best_conf = conf
                best_xyxy = box.xyxy[0].cpu().numpy()

        if best_xyxy is None:
            return None
        return best_idx, best_conf, best_xyxy

    def _select_detection_for_class(
        self, boxes, names, class_id: int
    ) -> Optional[Tuple[float, np.ndarray]]:
        if boxes is None or len(boxes) == 0:
            return None

        best_conf = -1.0
        best_xyxy = None
        for box in boxes:
            cls_id = int(box.cls[0])
            sem_id = self._semantic_id_for(cls_id, names)
            if sem_id is None or sem_id != class_id:
                continue
            conf = float(box.conf[0])
            if conf > best_conf:
                best_conf = conf
                best_xyxy = box.xyxy[0].cpu().numpy()

        if best_xyxy is None:
            return None
        return best_conf, best_xyxy

    def _effective_class_ids(self) -> List[int]:
        if self.class_ids:
            return [int(x) for x in self.class_ids]
        if self.target_class_id >= 0:
            return [int(self.target_class_id)]
        return []

    def _semantic_id_for(self, class_id: int, names) -> Optional[int]:
        if self.class_id_map:
            if class_id not in self.class_id_map:
                return None
            return self.class_id_map[class_id]
        if not self.semantic_name_to_id:
            return class_id
        name = names.get(class_id, "")
        if not name:
            return None
        return self.semantic_name_to_id.get(name.strip().lower())

    def _parse_class_id_map(self, raw) -> dict:
        if not raw:
            return {}
        if isinstance(raw, dict):
            return {int(k): int(v) for k, v in raw.items()}
        if isinstance(raw, (list, tuple)):
            items = list(raw)
            if len(items) % 2 != 0:
                return {}
            out = {}
            for i in range(0, len(items), 2):
                out[int(items[i])] = int(items[i + 1])
            return out
        if isinstance(raw, str):
            out = {}
            for part in raw.split(","):
                part = part.strip()
                if not part:
                    continue
                if ":" not in part:
                    return {}
                k, v = part.split(":", 1)
                out[int(k.strip())] = int(v.strip())
            return out
        return {}

    def _depth_at_bbox(self, depth_image: np.ndarray, bbox: np.ndarray) -> Optional[float]:
        x1, y1, x2, y2 = bbox
        cx = int(round((x1 + x2) * 0.5))
        cy = int(round((y1 + y2) * 0.5))

        h, w = depth_image.shape[:2]
        if cx < 0 or cy < 0 or cx >= w or cy >= h:
            return None

        x1i = max(cx - 2, 0)
        x2i = min(cx + 2, w - 1)
        y1i = max(cy - 2, 0)
        y2i = min(cy + 2, h - 1)
        window = depth_image[y1i : y2i + 1, x1i : x2i + 1].astype(np.float32)

        valid = window[np.isfinite(window)]
        valid = valid[valid > 0.0]
        if valid.size == 0:
            return None

        return float(np.median(valid))

    def _depth_to_meters(self, depth_image: np.ndarray, encoding: str) -> np.ndarray:
        if encoding == "16UC1":
            return depth_image.astype(np.float32) * 0.001
        return depth_image.astype(np.float32)

    def _point_from_pixel(self, u: float, v: float, depth: float) -> Tuple[float, float, float]:
        fx = self.camera_model.fx()
        fy = self.camera_model.fy()
        cx = self.camera_model.cx()
        cy = self.camera_model.cy()
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
        return x, y, z

    def _optical_to_camera_link(
        self, x: float, y: float, z: float
    ) -> Tuple[float, float, float]:
        # Optical frame: x right, y down, z forward
        # Camera link frame: x forward, y left, z up
        return z, -x, -y

    def _on_images(self, rgb_msg: Image, depth_msg: Image) -> None:
        if not self.camera_info_received:
            self.get_logger().warn("Waiting for camera_info")
            return

        try:
            rgb_image = self._rgb_from_msg(rgb_msg)
            depth_raw = self._depth_from_msg(depth_msg)
        except Exception as exc:
            self.get_logger().error(f"Failed to convert images: {exc}")
            return

        depth_m = self._depth_to_meters(depth_raw, depth_msg.encoding)

        results = self.model.predict(
            source=rgb_image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            device=self.device if self.device else None,
            verbose=False,
        )
        if not results:
            # self.get_logger().info("YOLO returned no results")
            return

        result = results[0]
        boxes = result.boxes
        effective_class_ids = self._effective_class_ids()
        if not effective_class_ids:
            selection = self._select_detection(boxes, result.names)
            if selection is None:
                # self.get_logger().info(
                #     "No detections matched target class or confidence"
                # )
                return
            class_id, conf, bbox = selection
            effective_class_ids = [class_id]
            detections = {class_id: (conf, bbox)}
        else:
            detections = {}
            for class_id in effective_class_ids:
                selection = self._select_detection_for_class(boxes, result.names, class_id)
                if selection is None:
                    continue
                detections[class_id] = selection

            if not detections:
                # self.get_logger().info(
                #     "No detections matched target class or confidence"
                # )
                return

        for class_id, (conf, bbox) in detections.items():
            x1, y1, x2, y2 = bbox
            depth = self._depth_at_bbox(depth_m, bbox)
            if depth is None or math.isnan(depth):
                self.get_logger().warn(
                    f"No valid depth at detection for class {class_id}"
                )
                continue

            u = (x1 + x2) * 0.5
            v = (y1 + y2) * 0.5
            x, y, z = self._point_from_pixel(u, v, depth)
            if not self.camera_frame_is_optical:
                x, y, z = self._optical_to_camera_link(x, y, z)

            point = PointStamped()
            point.header = rgb_msg.header
            point.point.x = float(x)
            point.point.y = float(y)
            point.point.z = float(z)

            pose = PoseStamped()
            pose.header = rgb_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = float(z)
            pose.pose.orientation.w = 1.0

            camera_tf = TransformStamped()
            camera_tf.header = rgb_msg.header
            camera_tf.child_frame_id = f"yolo/class_{class_id}"
            camera_tf.transform.translation.x = float(x)
            camera_tf.transform.translation.y = float(y)
            camera_tf.transform.translation.z = float(z)
            camera_tf.transform.rotation.w = 1.0

            if self.target_frame:
                if self.target_frame != rgb_msg.header.frame_id:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            rgb_msg.header.frame_id,
                            rgb_msg.header.stamp,
                            timeout=Duration(seconds=0.2),
                        )
                        point = do_transform_point(point, transform)
                        pose.header.frame_id = self.target_frame
                        pose.pose.position.x = point.point.x
                        pose.pose.position.y = point.point.y
                        pose.pose.position.z = point.point.z
                    except Exception as exc:
                        self.get_logger().warn(f"TF transform failed: {exc}")
                        continue

            pose_pub = self.pose_pubs.get(class_id)
            if pose_pub is not None:
                pose_pub.publish(pose)
                # self.get_logger().info(
                #     f"Published pose for class {class_id} (conf={conf:.2f})"
                # )

            self.tf_broadcaster.sendTransform(camera_tf)

        annotated = result.plot()
        annotated_msg = self._bgr_to_imgmsg(annotated, rgb_msg.header)
        self.image_pub.publish(annotated_msg)

        self.get_logger().debug(f"Published pose for class {class_id} (conf={conf:.2f})")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = YoloRgbdDetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
