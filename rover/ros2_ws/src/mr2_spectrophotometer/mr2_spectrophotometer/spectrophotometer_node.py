from __future__ import annotations

import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from mr2_spectrophotometer_msgs.msg import Spectrum
from mr2_spectrophotometer_msgs.srv import GetSpectrum

from spectro import apply_calibration, capture_frame, extract_profile, load_calibration
from spectro.intensity import load_intensity_reference, transmittance, absorbance


class SpectrophotometerNode(Node):
    MODE_RAW = 0
    MODE_TRANSMITTANCE = 1
    MODE_ABSORBANCE = 2

    def __init__(self) -> None:
        super().__init__("spectrophotometer_node")

        self.declare_parameter("camera_index", 0)
        default_calibration = self._default_data_path("calibration.json")
        default_intensity_ref = self._default_data_path("intensity_ref.npz")
        self.declare_parameter("calibration_path", default_calibration)
        self.declare_parameter("intensity_ref_path", default_intensity_ref)
        self.declare_parameter("band_height", 40)
        self.declare_parameter("y_center", -1)
        self.declare_parameter("smooth_kernel", 5)
        self.declare_parameter("frame_average", 3)
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("frame_id", "spectrophotometer")
        self.declare_parameter("service_name", "get_spectrum")
        self.declare_parameter("spectrum_topic", "spectrum")

        self._camera_index = int(self.get_parameter("camera_index").value)
        self._calibration_path = str(self.get_parameter("calibration_path").value)
        self._intensity_ref_path = str(self.get_parameter("intensity_ref_path").value)
        self._band_height = int(self.get_parameter("band_height").value)
        y_center = int(self.get_parameter("y_center").value)
        self._y_center = None if y_center < 0 else y_center
        self._smooth_kernel = int(self.get_parameter("smooth_kernel").value)
        self._frame_average = int(self.get_parameter("frame_average").value)
        self._frame_width = int(self.get_parameter("frame_width").value)
        self._frame_height = int(self.get_parameter("frame_height").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._service_name = str(self.get_parameter("service_name").value)
        self._spectrum_topic = str(self.get_parameter("spectrum_topic").value)

        self._lock = threading.Lock()
        self._calibration = self._load_calibration(self._calibration_path)
        self._intensity_ref = self._load_intensity_ref(self._intensity_ref_path)
        self._warned_ref_mismatch = False

        self._publisher = self.create_publisher(Spectrum, self._spectrum_topic, 10)
        self._service = self.create_service(GetSpectrum, self._service_name, self._handle_get_spectrum)

        self.get_logger().info(
            f"Spectrophotometer ready. Service: {self._service_name}, topic: {self._spectrum_topic}"
        )

    def _load_calibration(self, path: str):
        try:
            calib = load_calibration(path)
            self.get_logger().info(f"Loaded calibration from {path}")
            return calib
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load calibration '{path}': {exc}")
            return None

    def _load_intensity_ref(self, path: str):
        if not path:
            self.get_logger().info("No intensity reference path provided; using raw intensity")
            return None
        try:
            ref = load_intensity_reference(path)
            self.get_logger().info(f"Loaded intensity reference from {path}")
            return ref
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"Failed to load intensity reference '{path}': {exc}")
            return None

    @staticmethod
    def _default_data_path(filename: str) -> str:
        share_dir = get_package_share_directory("mr2_spectrophotometer")
        return os.path.join(share_dir, "config", filename)

    def _handle_get_spectrum(self, request: GetSpectrum.Request, response: GetSpectrum.Response):
        if self._calibration is None:
            response.success = False
            response.message = "Calibration not loaded"
            response.spectrum = Spectrum()
            return response

        if request.use_absorbance and self._intensity_ref is None:
            response.success = False
            response.message = "Absorbance requested but intensity reference not loaded"
            response.spectrum = Spectrum()
            return response

        with self._lock:
            try:
                resolution = None
                if self._frame_width > 0 or self._frame_height > 0:
                    width = self._frame_width if self._frame_width > 0 else None
                    height = self._frame_height if self._frame_height > 0 else None
                    resolution = (width, height)
                frame = capture_frame(
                    camera_index=self._camera_index,
                    warmup_seconds=0.0,
                    frame_average=self._frame_average,
                    resolution=resolution,
                )
            except Exception as exc:  # noqa: BLE001
                response.success = False
                response.message = f"Capture failed: {exc}"
                response.spectrum = Spectrum()
                return response

        x, intensity = extract_profile(
            frame,
            y_center=self._y_center,
            band_height=self._band_height,
            smooth_kernel=self._smooth_kernel,
        )
        wl, inten = apply_calibration(self._calibration, x, intensity)

        if self._intensity_ref is not None:
            ref_dark, ref_reference = self._match_intensity_reference(x)
            trans = transmittance(inten, ref_reference, ref_dark)
            if request.use_absorbance:
                y = absorbance(trans)
                mode = self.MODE_ABSORBANCE
            else:
                y = trans
                mode = self.MODE_TRANSMITTANCE
        else:
            y = inten
            mode = self.MODE_RAW

        msg = Spectrum()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.wavelength_nm = np.asarray(wl, dtype=np.float32).tolist()
        msg.intensity = np.asarray(y, dtype=np.float32).tolist()
        msg.mode = int(mode)

        response.success = True
        response.message = "ok"
        response.spectrum = msg

        if request.publish_topic:
            self._publisher.publish(msg)

        return response

    def _match_intensity_reference(self, x: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        ref = self._intensity_ref
        if ref is None:
            raise RuntimeError("Intensity reference not loaded")
        if len(ref.x) == len(x) and np.array_equal(ref.x, x):
            return ref.dark, ref.reference

        if not self._warned_ref_mismatch:
            self.get_logger().warn(
                "Intensity reference length does not match current capture; interpolating to fit."
            )
            self._warned_ref_mismatch = True

        ref_dark = np.interp(x, ref.x, ref.dark)
        ref_reference = np.interp(x, ref.x, ref.reference)
        return ref_dark, ref_reference


def main() -> None:
    rclpy.init()
    node = SpectrophotometerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
