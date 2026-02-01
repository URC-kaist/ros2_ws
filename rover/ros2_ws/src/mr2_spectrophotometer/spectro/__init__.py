"""Lightweight tools for webcam-based spectrophotometer calibration and analysis."""

from .calibration import (
    Calibration,
    apply_calibration,
    calibrate_from_image,
    calibrate_from_profile,
    load_calibration,
    save_calibration,
)
from .capture import capture_frame, find_working_camera
from .processing import extract_profile
from .intensity import (
    IntensityReference,
    absorbance,
    capture_profile,
    load_intensity_reference,
    save_intensity_reference,
    transmittance,
)

__all__ = [
    "Calibration",
    "apply_calibration",
    "calibrate_from_image",
    "calibrate_from_profile",
    "load_calibration",
    "save_calibration",
    "capture_frame",
    "find_working_camera",
    "extract_profile",
    "IntensityReference",
    "capture_profile",
    "save_intensity_reference",
    "load_intensity_reference",
    "transmittance",
    "absorbance",
]
