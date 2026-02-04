"""Calibration helpers for mapping pixel positions to wavelengths."""

from __future__ import annotations

import json
from dataclasses import dataclass, asdict, fields
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np


@dataclass
class Calibration:
    """Holds a pixel→wavelength polynomial along with metadata."""

    coefficients: Sequence[float]
    pixel_centers: Sequence[float]
    known_wavelengths: Sequence[float]
    poly_order: int
    note: str | None = None

    def pixel_to_wavelength(self, pixels: np.ndarray) -> np.ndarray:
        """Evaluate the calibration polynomial."""
        return np.polyval(self.coefficients, pixels)

    def wavelength_to_pixel(self, wavelengths: np.ndarray) -> np.ndarray:
        """Invert the polynomial via numerical root search."""
        # Simple numeric inversion using Newton iterations per value.
        pixels = np.zeros_like(wavelengths, dtype=float)
        for i, wl in enumerate(wavelengths):
            # Start guess: linear map based on calibration span
            px0 = np.mean(self.pixel_centers)
            x = px0
            for _ in range(20):
                f = np.polyval(self.coefficients, x) - wl
                df = np.polyval(np.polyder(self.coefficients), x)
                if df == 0:
                    break
                x -= f / df
            pixels[i] = x
        return pixels


def _find_peaks(intensity: np.ndarray, min_prominence: float = 0.2, distance: int = 10) -> np.ndarray:
    """
    Lightweight peak finder without SciPy.

    Args:
        intensity: 1‑D array of signal values.
        min_prominence: Fraction of the (max-min) range a peak must exceed over its neighbors.
        distance: Minimum index spacing between consecutive peaks.
    """
    if intensity.ndim != 1:
        raise ValueError("intensity must be 1‑D")

    rng = intensity.max() - intensity.min()
    prom = rng * min_prominence
    peaks: list[int] = []
    last_idx = -distance

    for i in range(1, len(intensity) - 1):
        if i - last_idx < distance:
            continue
        if intensity[i] <= intensity[i - 1] or intensity[i] <= intensity[i + 1]:
            continue
        left = intensity[i] - intensity[i - 1]
        right = intensity[i] - intensity[i + 1]
        if left > prom and right > prom:
            peaks.append(i)
            last_idx = i
    return np.array(peaks, dtype=int)


def calibrate_from_profile(
    x: np.ndarray,
    intensity: np.ndarray,
    known_wavelengths_nm: Sequence[float],
    peak_prominence: float = 0.25,
    min_peak_distance: int = 15,
) -> Calibration:
    """
    Compute calibration from a 1‑D profile and known laser wavelengths.

    Args:
        x: Pixel positions (same length as intensity).
        intensity: 1‑D intensity profile.
        known_wavelengths_nm: Laser wavelengths in nm (sorted ascending).
        peak_prominence: Fractional prominence used to detect peaks.
        min_peak_distance: Minimum pixel distance between detected peaks.
    """
    peaks = _find_peaks(intensity, min_prominence=peak_prominence, distance=min_peak_distance)
    if len(peaks) < len(known_wavelengths_nm):
        raise RuntimeError(
            f"Detected {len(peaks)} peaks but {len(known_wavelengths_nm)} wavelengths were provided."
        )

    # Choose the strongest peaks if more than needed, then sort left→right
    if len(peaks) > len(known_wavelengths_nm):
        peak_strengths = intensity[peaks]
        strongest_indices = np.argsort(peak_strengths)[-len(known_wavelengths_nm):]
        peaks = np.sort(peaks[strongest_indices])
    else:
        peaks = np.sort(peaks)

    known_wavelengths_nm = np.asarray(list(known_wavelengths_nm), dtype=float)
    if not np.all(np.diff(known_wavelengths_nm) > 0):
        raise ValueError("known_wavelengths_nm must be sorted ascending")

    if len(peaks) != len(known_wavelengths_nm):
        raise RuntimeError("Mismatch between detected peaks and provided wavelengths after filtering")

    order = min(2, len(peaks) - 1)  # linear for 2 points, quadratic for 3+
    px_centers = np.asarray(x)[peaks].astype(float)
    coeffs = np.polyfit(px_centers, known_wavelengths_nm, deg=order)

    return Calibration(
        coefficients=coeffs.tolist(),
        pixel_centers=px_centers.tolist(),
        known_wavelengths=known_wavelengths_nm.tolist(),
        poly_order=order,
    )


def calibrate_from_image(
    image: np.ndarray,
    known_wavelengths_nm: Sequence[float],
    peak_prominence: float = 0.25,
    min_peak_distance: int = 15,
    profile_extractor=None,
) -> Calibration:
    """
    Convenience wrapper that extracts a profile then calls calibrate_from_profile.
    """
    if profile_extractor is None:
        raise ValueError("profile_extractor is required when image is given directly")

    x, intensity = profile_extractor(image)
    return calibrate_from_profile(
        x=x,
        intensity=intensity,
        known_wavelengths_nm=known_wavelengths_nm,
        peak_prominence=peak_prominence,
        min_peak_distance=min_peak_distance,
    )


def apply_calibration(calibration: Calibration, x_pixels: np.ndarray, intensity: np.ndarray):
    """
    Convert a pixel-domain profile to wavelength domain.

    Returns:
        wavelengths_nm, intensity (unchanged).
    """
    wl = calibration.pixel_to_wavelength(x_pixels)
    return wl, intensity


def save_calibration(calibration: Calibration, path: str | Path) -> None:
    """Persist calibration to JSON."""
    path = Path(path)
    payload = asdict(calibration)
    path.write_text(json.dumps(payload, indent=2))


def load_calibration(path: str | Path) -> Calibration:
    """Load calibration from JSON."""
    path = Path(path)
    data = json.loads(path.read_text())
    allowed = {f.name for f in fields(Calibration)}
    filtered = {k: v for k, v in data.items() if k in allowed}
    return Calibration(**filtered)
