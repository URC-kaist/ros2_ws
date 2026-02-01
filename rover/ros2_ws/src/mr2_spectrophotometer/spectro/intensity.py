"""Intensity calibration utilities for transmission/absorbance measurements."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

from .capture import capture_frame
from .processing import extract_profile


@dataclass
class IntensityReference:
    """Holds dark and reference profiles for intensity calibration."""

    x: np.ndarray
    dark: np.ndarray
    reference: np.ndarray


def capture_profile(
    camera_index: int = 0,
    warmup_seconds: float = 1.0,
    frame_average: int = 5,
    y_center: Optional[int] = None,
    band_height: int = 40,
    smooth_kernel: int = 5,
):
    """Capture a frame and return (x, intensity) profile."""
    frame = capture_frame(
        camera_index=camera_index,
        warmup_seconds=warmup_seconds,
        frame_average=frame_average,
    )
    return extract_profile(
        frame,
        y_center=y_center,
        band_height=band_height,
        smooth_kernel=smooth_kernel,
    )


def save_intensity_reference(ref: IntensityReference, path: str | Path) -> None:
    """Persist intensity reference to NPZ."""
    path = Path(path)
    np.savez_compressed(path, x=ref.x, dark=ref.dark, reference=ref.reference)


def load_intensity_reference(path: str | Path) -> IntensityReference:
    """Load intensity reference from NPZ."""
    path = Path(path)
    data = np.load(path)
    return IntensityReference(x=data["x"], dark=data["dark"], reference=data["reference"])


def transmittance(
    sample: np.ndarray,
    reference: np.ndarray,
    dark: np.ndarray,
    clip: float = 1e-6,
) -> np.ndarray:
    """Compute transmittance (I_sample - I_dark) / (I_ref - I_dark)."""
    denom = reference - dark
    numer = sample - dark
    denom = np.clip(denom, clip, None)
    trans = numer / denom
    return np.clip(trans, clip, None)


def absorbance(trans: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    """Compute absorbance A = -log10(trans)."""
    return -np.log10(np.clip(trans, eps, None))
