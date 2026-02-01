"""
Web-based calibration for headless systems.

Usage:
    python web_calibrate.py --camera 0 --wavelengths 405 532 638

Workflow:
    1) Capture dark + reference for intensity calibration.
    2) Capture laser peaks for wavelength calibration.
"""

from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

import numpy as np

from spectro.calibration import Calibration, save_calibration
from spectro.capture import capture_frame
from spectro.intensity import IntensityReference, save_intensity_reference
from spectro.processing import extract_profile
from spectro.webviz import ensure_html_path, write_plot_html


_PLOTLY_CDN = "https://cdn.plot.ly/plotly-2.26.0.min.js"


class _CalibState:
    def __init__(
        self,
        wavelengths: list[float],
        camera_source: int,
    ) -> None:
        self.lock = threading.Lock()
        self.camera_lock = threading.Lock()
        self.wavelengths = wavelengths
        self.camera_source = camera_source
        self.index = 0
        self.stage = "dark"
        self.x: list[float] = []
        self.y: list[float] = []
        self.peak_px: float | None = None
        self.peak_val: float | None = None
        self.captured: list[dict[str, float]] = []
        self.dark_profile: tuple[list[float], list[float]] | None = None
        self.reference_profile: tuple[list[float], list[float]] | None = None
        self.done = False
        self.message = "Waiting for data..."

    def current_wavelength(self) -> float | None:
        if self.index >= len(self.wavelengths):
            return None
        return self.wavelengths[self.index]


def _html_page() -> str:
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>Web Calibration</title>
    <script src="{_PLOTLY_CDN}"></script>
    <style>
      body {{ margin: 0; font-family: sans-serif; background: #0b0d10; color: #e6e6e6; }}
      header {{ padding: 12px 16px; background: #11151c; display: flex; flex-wrap: wrap; gap: 12px; align-items: center; }}
      header strong {{ font-size: 1.05rem; margin-right: 6px; }}
      button {{ background: #2b72ff; color: white; border: none; padding: 8px 14px; border-radius: 6px; cursor: pointer; }}
      button.secondary {{ background: #3a3f4b; }}
      button:disabled {{ opacity: 0.4; cursor: default; }}
      .status {{ opacity: 0.75; font-size: 0.9rem; }}
      #plot {{ width: 100vw; height: 55vh; }}
      #panel {{ padding: 12px 16px; display: grid; gap: 12px; }}
      table {{ width: 100%; border-collapse: collapse; }}
      th, td {{ border-bottom: 1px solid #222; padding: 6px 4px; text-align: left; }}
      .grid {{ display: grid; gap: 12px; grid-template-columns: 1fr 1fr; }}
      @media (max-width: 900px) {{ .grid {{ grid-template-columns: 1fr; }} }}
    </style>
  </head>
  <body>
    <header>
      <strong>Web Calibration</strong>
      <span id="stage" class="status">Stage: loading</span>
      <span id="current" class="status">Laser: --</span>
      <span id="camera" class="status">Camera: --</span>
      <button id="captureDarkBtn" class="secondary" onclick="captureDark()">Capture Dark</button>
      <button id="captureRefBtn" class="secondary" onclick="captureRef()">Capture Reference</button>
      <button id="captureLaserBtn" onclick="captureLaser()">Capture Laser</button>
      <span id="message" class="status"></span>
    </header>
    <div id="plot"></div>
    <div id="panel">
      <div id="instructions" class="status">Loading...</div>
      <div class="grid">
        <div>
          <strong>Intensity Calibration</strong>
          <table>
            <thead>
              <tr><th>Step</th><th>Status</th></tr>
            </thead>
            <tbody>
              <tr><td>Dark</td><td id="darkStatus">pending</td></tr>
              <tr><td>Reference</td><td id="refStatus">pending</td></tr>
            </tbody>
          </table>
        </div>
        <div>
          <strong>Wavelength Captures</strong>
          <table>
            <thead>
              <tr><th>Wavelength (nm)</th><th>Pixel</th></tr>
            </thead>
            <tbody id="capturedRows"></tbody>
          </table>
        </div>
      </div>
    </div>
    <script>
      let initialized = false;
      async function fetchData() {{
        const resp = await fetch("/data.json", {{ cache: "no-store" }});
        if (!resp.ok) throw new Error("bad response");
        return await resp.json();
      }}
      async function updatePlot() {{
        try {{
          const payload = await fetchData();
          const traces = [{{ x: payload.x || [], y: payload.y || [], mode: "lines", line: {{ color: "#6cc4ff" }} }}];
          const layout = {{
            title: "Live Spectrum",
            paper_bgcolor: "#0b0d10",
            plot_bgcolor: "#0b0d10",
            font: {{ color: "#e6e6e6" }},
            xaxis: {{ title: "Pixel", gridcolor: "#222" }},
            yaxis: {{ title: "Intensity (a.u.)", gridcolor: "#222" }},
            margin: {{ t: 50, l: 70, r: 30, b: 60 }}
          }};
          if (payload.peak_px !== null && payload.peak_px !== undefined) {{
            layout.shapes = [{{
              type: "line",
              x0: payload.peak_px,
              x1: payload.peak_px,
              y0: 0,
              y1: 1,
              yref: "paper",
              line: {{ color: "#ff5f5f", width: 2, dash: "dash" }}
            }}];
          }}
          if (!initialized) {{
            Plotly.newPlot("plot", traces, layout, {{responsive: true, displaylogo: false}});
            initialized = true;
          }} else {{
            Plotly.react("plot", traces, layout, {{responsive: true, displaylogo: false}});
          }}

          const stage = payload.stage || "dark";
          let current = "--";
          if (stage === "wavelengths") {{
            current = payload.current_wavelength === null ? "Done" : `${{payload.current_wavelength}} nm`;
          }} else if (stage === "done") {{
            current = "Done";
          }}
          const cameraIndex = payload.camera_source === null || payload.camera_source === undefined ? "--" : payload.camera_source;
          document.getElementById("stage").textContent = `Stage: ${{stage}}`;
          document.getElementById("current").textContent = `Laser: ${{current}}`;
          document.getElementById("camera").textContent = `Camera: ${{cameraIndex}}`;
          document.getElementById("message").textContent = payload.message || "";

          const instructions = {{
            dark: "Block light, then click Capture Dark.",
            reference: "Insert blank/reference, then click Capture Reference.",
            wavelengths: "Turn on the current laser and click Capture Laser.",
            done: "Calibration complete."
          }};
          document.getElementById("instructions").textContent = instructions[stage] || "";

          const disabled = !!payload.done;
          document.getElementById("captureDarkBtn").disabled = disabled || stage !== "dark";
          document.getElementById("captureRefBtn").disabled = disabled || stage !== "reference";
          document.getElementById("captureLaserBtn").disabled = disabled || stage !== "wavelengths";

          document.getElementById("darkStatus").textContent = payload.dark_done ? "captured" : "pending";
          document.getElementById("refStatus").textContent = payload.reference_done ? "captured" : "pending";

          const rows = payload.captured || [];
          const body = document.getElementById("capturedRows");
          body.innerHTML = rows.map(r => `<tr><td>${{r.wavelength.toFixed(1)}}</td><td>${{r.pixel.toFixed(1)}}</td></tr>`).join("");
        }} catch (err) {{
          document.getElementById("message").textContent = "Waiting for data...";
        }} finally {{
          setTimeout(updatePlot, 200);
        }}
      }}
      async function captureDark() {{
        await fetch("/capture_dark", {{ method: "POST" }});
      }}
      async function captureRef() {{
        await fetch("/capture_reference", {{ method: "POST" }});
      }}
      async function captureLaser() {{
        await fetch("/capture_laser", {{ method: "POST" }});
      }}
      updatePlot();
    </script>
  </body>
</html>
"""


class _CalibHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            self._send_html(self.server.page_html)
            return
        if self.path.startswith("/data"):
            payload = self.server.payload_fn()
            self._send_json(payload)
            return
        self.send_error(404, "Not Found")

    def do_POST(self) -> None:  # noqa: N802
        if self.path.startswith("/capture_dark"):
            payload = self.server.capture_dark_fn()
            self._send_json(payload)
            return
        if self.path.startswith("/capture_reference"):
            payload = self.server.capture_reference_fn()
            self._send_json(payload)
            return
        if self.path.startswith("/capture_laser"):
            payload = self.server.capture_laser_fn()
            self._send_json(payload)
            return
        self.send_error(404, "Not Found")

    def log_message(self, format: str, *args: Any) -> None:
        return

    def _send_html(self, html: str) -> None:
        encoded = html.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_json(self, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)


def _capture_loop(
    state: _CalibState,
    band_height: int,
    y_center: int | None,
    smooth_kernel: int,
    frame_average: int,
    interval_ms: int,
    running: threading.Event,
) -> None:
    while running.is_set():
        with state.lock:
            stage = state.stage
            camera_source = state.camera_source
        try:
            with state.camera_lock:
                frame = capture_frame(
                    camera_index=camera_source,
                    warmup_seconds=0.0,
                    frame_average=frame_average,
                )
        except RuntimeError as exc:
            with state.lock:
                state.message = f"Camera error: {exc}"
            time.sleep(1.0)
            continue
        x, intensity = extract_profile(
            frame,
            y_center=y_center,
            band_height=band_height,
            smooth_kernel=smooth_kernel,
        )
        peak_px = float(x[int(np.argmax(intensity))]) if intensity.size else None
        peak_val = float(np.max(intensity)) if intensity.size else None

        with state.lock:
            state.x = x.tolist()
            state.y = intensity.tolist()
            state.peak_px = peak_px
            state.peak_val = peak_val
            if stage == "wavelengths" and state.current_wavelength() is not None:
                state.message = f"Peak @ {peak_px:.1f}px" if peak_px is not None else "No peak detected"

        time.sleep(interval_ms / 1000.0)


def _capture_profile(
    camera_index: int,
    band_height: int,
    y_center: int | None,
    smooth_kernel: int,
    frame_average: int,
    camera_lock: threading.Lock,
) -> tuple[np.ndarray, np.ndarray]:
    with camera_lock:
        frame = capture_frame(
            camera_index=camera_index,
            warmup_seconds=0.0,
            frame_average=frame_average,
        )
    return extract_profile(
        frame,
        y_center=y_center,
        band_height=band_height,
        smooth_kernel=smooth_kernel,
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="Web-based calibration")
    parser.add_argument("--camera", type=int, default=0, help="OpenCV camera index")
    parser.add_argument("--wavelengths", type=float, nargs="+", required=True, help="Laser wavelengths in nm")
    parser.add_argument("--band-height", type=int, default=40, help="Rows to average when extracting profile")
    parser.add_argument("--y-center", type=int, default=None, help="Row center for extraction band")
    parser.add_argument("--smooth-kernel", type=int, default=3, help="Box smoothing kernel size")
    parser.add_argument("--frame-average", type=int, default=5, help="Frames to average per capture")
    parser.add_argument("--interval-ms", type=int, default=200, help="Update interval in milliseconds")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Host/interface to bind")
    parser.add_argument("--port", type=int, default=8001, help="Port to bind")
    parser.add_argument(
        "--outfile",
        type=str,
        default="/home/mr2/mr2-stack/rover/ros2_ws/src/mr2_spectrophotometer/config/calibration.json",
        help="Where to write calibration JSON",
    )
    parser.add_argument(
        "--save-plot",
        type=str,
        default="/home/mr2/mr2-stack/rover/ros2_ws/src/mr2_spectrophotometer/config/calibration_fit.html",
        help="HTML calibration plot",
    )
    parser.add_argument(
        "--intensity-out",
        type=str,
        default="/home/mr2/mr2-stack/rover/ros2_ws/src/mr2_spectrophotometer/config/intensity_ref.npz",
        help="Where to write intensity reference NPZ",
    )
    args = parser.parse_args()

    wavelengths = [float(w) for w in args.wavelengths]
    if sorted(wavelengths) != wavelengths:
        raise SystemExit("wavelengths must be sorted ascending")

    camera_source: int = args.camera
    state = _CalibState(wavelengths, camera_source)

    def payload_fn() -> dict[str, Any]:
        with state.lock:
            return {
                "x": state.x,
                "y": state.y,
                "peak_px": state.peak_px,
                "current_wavelength": state.current_wavelength(),
                "captured": list(state.captured),
                "done": state.done,
                "message": state.message,
                "stage": state.stage,
                "dark_done": state.dark_profile is not None,
                "reference_done": state.reference_profile is not None,
                "camera_source": state.camera_source,
            }

    def finalize_calibration() -> None:
        pixels = [item["pixel"] for item in state.captured]
        wls = [item["wavelength"] for item in state.captured]
        order = min(2, len(pixels) - 1)
        coeffs = np.polyfit(pixels, wls, deg=order)
        calib = Calibration(
            coefficients=coeffs.tolist(),
            pixel_centers=pixels,
            known_wavelengths=wls,
            poly_order=order,
            note="Web calibration",
        )
        save_calibration(calib, args.outfile)
        xs = np.linspace(min(pixels) - 20, max(pixels) + 20, 300)
        fit = np.polyval(coeffs, xs)
        plot_path = ensure_html_path(args.save_plot)
        write_plot_html(
            plot_path,
            traces=[
                {"x": pixels, "y": wls, "mode": "markers", "name": "measured peaks"},
                {"x": xs, "y": fit, "mode": "lines", "name": f"poly order {order}"},
            ],
            title="Calibration fit",
            x_label="Pixel",
            y_label="Wavelength (nm)",
        )
        state.message = f"Saved {args.outfile} and {plot_path}"

    def capture_dark_fn() -> dict[str, Any]:
        with state.lock:
            if state.stage != "dark":
                return payload_fn()
            camera_source = state.camera_source
        try:
            x, intensity = _capture_profile(
                camera_index=camera_source,
                band_height=args.band_height,
                y_center=args.y_center,
                smooth_kernel=args.smooth_kernel,
                frame_average=args.frame_average,
                camera_lock=state.camera_lock,
            )
        except RuntimeError as exc:
            with state.lock:
                state.message = f"Dark capture failed: {exc}"
            return payload_fn()
        with state.lock:
            state.dark_profile = (x.tolist(), intensity.tolist())
            state.stage = "reference"
            state.message = "Dark captured. Insert reference/blank and capture."
        return payload_fn()

    def capture_reference_fn() -> dict[str, Any]:
        with state.lock:
            if state.stage != "reference":
                return payload_fn()
            dark_profile = state.dark_profile
            camera_source = state.camera_source
        try:
            x, intensity = _capture_profile(
                camera_index=camera_source,
                band_height=args.band_height,
                y_center=args.y_center,
                smooth_kernel=args.smooth_kernel,
                frame_average=args.frame_average,
                camera_lock=state.camera_lock,
            )
        except RuntimeError as exc:
            with state.lock:
                state.message = f"Reference capture failed: {exc}"
            return payload_fn()
        if dark_profile is None:
            with state.lock:
                state.message = "Dark reference missing; capture dark first."
            return payload_fn()
        x_dark, dark = dark_profile
        if len(x_dark) != len(x):
            with state.lock:
                state.message = "Profile length mismatch; check setup."
            return payload_fn()
        ref = IntensityReference(x=np.array(x), dark=np.array(dark), reference=np.array(intensity))
        save_intensity_reference(ref, args.intensity_out)
        with state.lock:
            state.reference_profile = (x.tolist(), intensity.tolist())
            state.stage = "wavelengths"
            state.message = f"Reference captured. Saved {args.intensity_out}. Capture lasers."
        return payload_fn()

    def capture_laser_fn() -> dict[str, Any]:
        with state.lock:
            if state.stage != "wavelengths":
                return payload_fn()
            if state.done:
                return payload_fn()
            current = state.current_wavelength()
            peak_px = state.peak_px
            if current is None or peak_px is None:
                state.message = "No peak detected."
                return payload_fn()
            state.captured.append({"wavelength": float(current), "pixel": float(peak_px)})
            state.index += 1
            if state.index >= len(state.wavelengths):
                state.done = True
                state.stage = "done"
        if state.done:
            finalize_calibration()
        return payload_fn()

    server = ThreadingHTTPServer((args.host, args.port), _CalibHandler)
    server.page_html = _html_page()
    server.payload_fn = payload_fn
    server.capture_dark_fn = capture_dark_fn
    server.capture_reference_fn = capture_reference_fn
    server.capture_laser_fn = capture_laser_fn

    running = threading.Event()
    running.set()

    thread = threading.Thread(
        target=_capture_loop,
        args=(
            state,
            args.band_height,
            args.y_center,
            args.smooth_kernel,
            args.frame_average,
            args.interval_ms,
            running,
        ),
        daemon=True,
    )
    thread.start()

    def stop_server(message: str) -> None:
        running.clear()
        with state.lock:
            state.stage = "done"
            state.done = True
            state.message = message
        server.shutdown()

    def handle_sigint(signum, frame) -> None:
        stop_server("Aborted by user.")

    signal.signal(signal.SIGINT, handle_sigint)

    print(f"Calibration web UI at http://{args.host}:{server.server_address[1]}/")
    if args.host == "0.0.0.0":
        print("Use this machine's IP/hostname instead of 0.0.0.0 in your browser.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        stop_server("Aborted by user.")
    finally:
        running.clear()
        server.server_close()
        thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
