"""Image → spectral profile utilities."""

from __future__ import annotations

from typing import Iterable, Tuple

import cv2
import numpy as np


def extract_profile(
    image: np.ndarray,
    y_center: int | None = None,
    band_height: int = 30,
    smooth_kernel: int = 5,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Reduce a 2‑D spectrum image to a 1‑D intensity profile along x.

    Args:
        image: BGR or grayscale NumPy array with wavelength spread across x.
        y_center: Optional row index to center the extraction band. If None, uses image center.
        band_height: Number of rows to average; increase to improve SNR.
        smooth_kernel: Size of 1‑D box filter to lightly smooth the profile; set 1 to disable.

    Returns:
        x_positions, intensity arrays.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    h, w = gray.shape
    if y_center is None:
        y_center = h // 2

    half = max(1, band_height // 2)
    y0 = max(0, y_center - half)
    y1 = min(h, y_center + half)
    band = gray[y0:y1, :]

    profile = band.mean(axis=0)
    if smooth_kernel > 1:
        k = smooth_kernel
        box = np.ones(k) / k
        profile = np.convolve(profile, box, mode="same")

    x_positions = np.arange(w, dtype=float)
    return x_positions, profile
