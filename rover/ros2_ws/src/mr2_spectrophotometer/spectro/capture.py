"""Webcam frame capture utilities."""

from __future__ import annotations

import shutil
import subprocess
import time
from typing import Optional

import cv2
import numpy as np


def _open_camera(
    camera_index: int,
    resolution: Optional[tuple[int | None, int | None]] = None,
) -> cv2.VideoCapture:
    """
    Try backends in order; return an opened VideoCapture or raise.
    """
    backend_order = []
    if hasattr(cv2, "CAP_V4L2"):
        backend_order.append(cv2.CAP_V4L2)  # prefer V4L2 on Linux
    backend_order.append(None)  # then default backend
    if hasattr(cv2, "CAP_ANY"):
        backend_order.append(cv2.CAP_ANY)  # fallback generic

    last_err = None
    for backend in backend_order:
        if backend is None:
            cap = cv2.VideoCapture(camera_index)
        else:
            cap = cv2.VideoCapture(camera_index, backend)
        if resolution:
            width, height = resolution
            if width is not None and width > 0:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height is not None and height > 0:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if cap.isOpened():
            return cap
        last_err = "default" if backend is None else backend
        cap.release()

    raise RuntimeError(
        f"Could not open camera {camera_index} (tried backends {backend_order}, last={last_err})"
    )


def find_working_camera(max_index: int = 5) -> Optional[int]:
    """
    Probe camera indices [0, max_index] and return the first that opens.
    """
    for idx in range(max_index + 1):
        try:
            cap = _open_camera(idx)
            cap.release()
            return idx
        except RuntimeError:
            continue
    return None


DEFAULT_V4L2_CONTROLS = (
    "auto_exposure=1,"
    "exposure_time_absolute=6000,"
    "exposure_dynamic_framerate=1,"
    "white_balance_automatic=0,"
    "white_balance_temperature=4600,"
    "power_line_frequency=1,"
    "backlight_compensation=0,"
    "brightness=0,"
    "contrast=38,"
    "gamma=400,"
    "hue=0,"
    "saturation=0,"
    "sharpness=0"
)
_V4L2_APPLIED: set[str] = set()


def _format_v4l2_controls(controls: dict[str, object] | str) -> str:
    if isinstance(controls, str):
        return controls
    parts = []
    for key, value in controls.items():
        parts.append(f"{key}={value}")
    return ",".join(parts)


def apply_v4l2_controls(
    device: str,
    controls: dict[str, object] | str,
    *,
    once: bool = True,
    v4l2_ctl: str = "v4l2-ctl",
) -> None:
    """
    Apply V4L2 control settings via v4l2-ctl (Linux only).
    """
    if not controls:
        return
    if shutil.which(v4l2_ctl) is None:
        raise RuntimeError(f"{v4l2_ctl} not found; install v4l-utils or set v4l2_ctl path")
    ctrl_arg = _format_v4l2_controls(controls)
    key = f"{device}|{ctrl_arg}|{v4l2_ctl}"
    if once and key in _V4L2_APPLIED:
        return
    try:
        subprocess.run(
            [v4l2_ctl, "-d", device, f"--set-ctrl={ctrl_arg}"],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as exc:
        detail = exc.stderr.strip() or exc.stdout.strip() or str(exc)
        raise RuntimeError(f"v4l2-ctl failed: {detail}") from exc
    if once:
        _V4L2_APPLIED.add(key)


def capture_frame(
    camera_index: int = 0,
    warmup_seconds: float = 1.0,
    frame_average: int = 5,
    resolution: Optional[tuple[int | None, int | None]] = None,
    exposure: Optional[float] = None,
    gain: Optional[float] = None,
) -> np.ndarray:
    """
    Grab a single averaged frame from a webcam.

    Args:
        camera_index: OpenCV camera index (0 is usually default webcam).
        warmup_seconds: Time to wait for the sensor to stabilize.
        frame_average: Number of frames to average for noise reduction.
        resolution: Optional (width, height). Use None or <=0 values to keep defaults.
        exposure: Manual exposure value (units backend-dependent; for UVC: log2(seconds)).
        gain: Optional gain setting (dB on many UVC cameras).

    Returns:
        BGR image as a NumPy array.
    """
    device = f"/dev/video{camera_index}"
    apply_v4l2_controls(device, DEFAULT_V4L2_CONTROLS, once=True, v4l2_ctl="v4l2-ctl")

    cap = _open_camera(camera_index, resolution)

    if hasattr(cv2, "CAP_PROP_AUTO_EXPOSURE"):
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25 if exposure is not None else 0.75)
    if exposure is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    if gain is not None:
        cap.set(cv2.CAP_PROP_GAIN, gain)

    def warmup() -> None:
        end_time = time.time() + warmup_seconds
        while time.time() < end_time:
            cap.read()

    def read_average() -> np.ndarray:
        frames = []
        for _ in range(max(1, frame_average)):
            ok, frame = cap.read()
            if not ok:
                cap.release()
                raise RuntimeError("Failed to read frame from camera")
            frames.append(frame.astype(np.float32))
        return np.mean(frames, axis=0)

    # Single exposure
    if warmup_seconds > 0:
        warmup()
    avg = read_average()
    cap.release()
    return avg.astype(np.uint8)
