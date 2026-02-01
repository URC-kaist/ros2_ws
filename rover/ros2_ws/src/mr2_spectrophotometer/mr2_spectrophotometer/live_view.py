"""
Live spectrogram viewer using an existing calibration.json.

Usage (after .venv activated):
    python live_view.py --camera 0 --calibration calibration.json
Open the printed URL in a browser on another machine.
"""

from __future__ import annotations

import argparse
import signal
import sys
import time

import cv2

from spectro import (
    apply_calibration,
    capture_frame,
    extract_profile,
    load_calibration,
)
from spectro.intensity import load_intensity_reference, transmittance, absorbance
from spectro.webviz import LivePlotServer


def main():
    parser = argparse.ArgumentParser(description="Live calibrated spectrogram viewer")
    parser.add_argument(
        "--camera", type=int, default=0, help="OpenCV camera index (default 0)"
    )
    parser.add_argument(
        "--calibration",
        type=str,
        default="calibration.json",
        help="Path to calibration file",
    )
    parser.add_argument(
        "--intensity-ref",
        type=str,
        default=None,
        help="Path to intensity reference NPZ (from intensity_setup.py) for transmittance/absorbance",
    )
    parser.add_argument(
        "--absorbance",
        action="store_true",
        help="Display absorbance (-log10 T) instead of transmittance",
    )
    parser.add_argument(
        "--band-height",
        type=int,
        default=40,
        help="Rows to average when extracting profile",
    )
    parser.add_argument(
        "--y-center",
        type=int,
        default=None,
        help="Row center for extraction band (default: image center)",
    )
    parser.add_argument(
        "--frame-average",
        type=int,
        default=3,
        help="Number of frames to average per update",
    )
    parser.add_argument(
        "--smooth-kernel",
        type=int,
        default=5,
        help="Box smoothing kernel size (1 disables smoothing)",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=200,
        help="Update interval in milliseconds",
    )
    parser.add_argument(
        "--no-image",
        action="store_true",
        help="Disable live webcam image in the web UI",
    )
    parser.add_argument(
        "--host",
        type=str,
        default="0.0.0.0",
        help="Host/interface to bind the web viewer",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8000,
        help="Port to bind the web viewer",
    )
    args = parser.parse_args()

    calib = load_calibration(args.calibration)
    intensity_ref = None
    if args.intensity_ref:
        intensity_ref = load_intensity_reference(args.intensity_ref)

    y_label = "Absorbance" if args.absorbance else "Transmittance" if intensity_ref else "Intensity (a.u.)"
    server = LivePlotServer(
        title="Live spectrum",
        x_label="Wavelength (nm)",
        y_label=y_label,
        host=args.host,
        port=args.port,
        interval_ms=args.interval_ms,
    )
    server.start()
    print(f"Live view at http://{args.host}:{server.port}/")
    if args.host == "0.0.0.0":
        print("Use this machine's IP/hostname instead of 0.0.0.0 in your browser.")

    running = True

    def handle_sigint(signum, frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, handle_sigint)

    try:
        while running:
            start = time.time()
            frame = capture_frame(
                camera_index=args.camera,
                warmup_seconds=0.0,
                frame_average=args.frame_average,
            )
            x, intensity = extract_profile(
                frame,
                y_center=args.y_center,
                band_height=args.band_height,
                smooth_kernel=args.smooth_kernel,
            )
            wl, inten = apply_calibration(calib, x, intensity)

            if intensity_ref:
                T = transmittance(inten, intensity_ref.reference, intensity_ref.dark)
                y = absorbance(T) if args.absorbance else T
            else:
                y = inten

            server.update(wl, y, y_label=y_label)
            if not args.no_image:
                ok, buf = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 80],
                )
                if ok:
                    server.update_image(buf.tobytes(), mime="image/jpeg")
            elapsed = time.time() - start
            sleep_s = max(0.0, args.interval_ms / 1000.0 - elapsed)
            time.sleep(sleep_s)
    finally:
        server.stop()


if __name__ == "__main__":
    sys.exit(main())
