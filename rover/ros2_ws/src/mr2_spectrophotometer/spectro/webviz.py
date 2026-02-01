"""Minimal web-based plotting utilities for headless environments."""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence

import numpy as np

_PLOTLY_CDN = "https://cdn.plot.ly/plotly-2.26.0.min.js"


def _to_list(values: Iterable[float] | np.ndarray) -> list[float]:
    array = np.asarray(values, dtype=float)
    return array.tolist()


def _sanitize_traces(traces: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    cleaned: list[dict[str, Any]] = []
    for trace in traces:
        item = dict(trace)
        if "x" in item:
            item["x"] = _to_list(item["x"])
        if "y" in item:
            item["y"] = _to_list(item["y"])
        cleaned.append(item)
    return cleaned


def _html_for_static_plot(traces: Sequence[Mapping[str, Any]], layout: Mapping[str, Any]) -> str:
    traces_json = json.dumps(_sanitize_traces(traces))
    layout_json = json.dumps(layout)
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>{layout.get("title", "Spectro Plot")}</title>
    <script src="{_PLOTLY_CDN}"></script>
    <style>
      body {{ margin: 0; font-family: sans-serif; background: #0b0d10; color: #e6e6e6; }}
      #plot {{ width: 100vw; height: 100vh; }}
    </style>
  </head>
  <body>
    <div id="plot"></div>
    <script>
      const traces = {traces_json};
      const layout = {layout_json};
      Plotly.newPlot("plot", traces, layout, {{responsive: true, displaylogo: false}});
    </script>
  </body>
</html>
"""


def write_plot_html(
    path: str | Path,
    traces: Sequence[Mapping[str, Any]],
    title: str,
    x_label: str,
    y_label: str,
    vlines: Sequence[float] | None = None,
) -> Path:
    """Write a static plot HTML file and return its path."""
    layout: dict[str, Any] = {
        "title": title,
        "paper_bgcolor": "#0b0d10",
        "plot_bgcolor": "#0b0d10",
        "font": {"color": "#e6e6e6"},
        "xaxis": {"title": x_label, "gridcolor": "#222"},
        "yaxis": {"title": y_label, "gridcolor": "#222"},
        "margin": {"t": 60, "l": 70, "r": 30, "b": 60},
    }
    if vlines:
        layout["shapes"] = [
            {
                "type": "line",
                "x0": float(x),
                "x1": float(x),
                "y0": 0,
                "y1": 1,
                "yref": "paper",
                "line": {"color": "#ff5f5f", "width": 2, "dash": "dash"},
            }
            for x in vlines
        ]
    html = _html_for_static_plot(traces, layout)
    out = ensure_html_path(path)
    out.write_text(html)
    return out


def ensure_html_path(path: str | Path) -> Path:
    """Ensure the path has an .html suffix."""
    out = Path(path)
    if out.suffix.lower() != ".html":
        out = out.with_suffix(".html")
    return out


def _html_for_live_plot(title: str, x_label: str, y_label: str, interval_ms: int) -> str:
    safe_title = title.replace('"', "&quot;")
    safe_x = x_label.replace('"', "&quot;")
    safe_y = y_label.replace('"', "&quot;")
    return f"""<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>{safe_title}</title>
    <script src="{_PLOTLY_CDN}"></script>
    <style>
      body {{ margin: 0; font-family: sans-serif; background: #0b0d10; color: #e6e6e6; }}
      header {{ padding: 12px 16px; background: #11151c; display: flex; gap: 16px; align-items: center; flex-wrap: wrap; }}
      #status {{ opacity: 0.75; font-size: 0.9rem; }}
      #content {{ display: grid; grid-template-columns: 2fr 1fr; height: calc(100vh - 52px); }}
      #plot {{ width: 100%; height: 100%; }}
      #imagePanel {{ padding: 10px; display: flex; flex-direction: column; gap: 8px; }}
      #frame {{ width: 100%; height: auto; border: 1px solid #222; background: #000; }}
      #noImage {{ opacity: 0.6; font-size: 0.9rem; }}
      @media (max-width: 900px) {{
        #content {{ grid-template-columns: 1fr; grid-template-rows: 60vh auto; }}
        #imagePanel {{ padding-top: 0; }}
      }}
      a {{ color: #8fb3ff; }}
    </style>
  </head>
  <body>
    <header>
      <strong>{safe_title}</strong>
      <span id="status">Waiting for data...</span>
    </header>
    <div id="content">
      <div id="plot"></div>
      <div id="imagePanel">
        <strong>Live Camera</strong>
        <img id="frame" alt="Live frame" style="display:none;" />
        <div id="noImage">No image yet</div>
      </div>
    </div>
    <script>
      const baseLayout = {{
        title: "{safe_title}",
        paper_bgcolor: "#0b0d10",
        plot_bgcolor: "#0b0d10",
        font: {{ color: "#e6e6e6" }},
        xaxis: {{ title: "{safe_x}", gridcolor: "#222" }},
        yaxis: {{ title: "{safe_y}", gridcolor: "#222" }},
        margin: {{ t: 50, l: 70, r: 30, b: 60 }}
      }};
      let initialized = false;
      async function updatePlot() {{
        try {{
          const response = await fetch("/data.json", {{ cache: "no-store" }});
          if (!response.ok) throw new Error("bad response");
          const payload = await response.json();
          const traces = [{{ x: payload.x || [], y: payload.y || [], mode: "lines", line: {{ color: "#6cc4ff" }} }}];
          const layout = {{ ...baseLayout }};
          if (payload.y_label) layout.yaxis.title = payload.y_label;
          if (payload.title) layout.title = payload.title;
          if (!initialized) {{
            Plotly.newPlot("plot", traces, layout, {{responsive: true, displaylogo: false}});
            initialized = true;
          }} else {{
            Plotly.react("plot", traces, layout, {{responsive: true, displaylogo: false}});
          }}
          const frameEl = document.getElementById("frame");
          const noImageEl = document.getElementById("noImage");
          if (payload.has_image && payload.image_ts) {{
            frameEl.style.display = "block";
            noImageEl.style.display = "none";
            frameEl.src = "/frame.jpg?t=" + payload.image_ts;
          }} else {{
            frameEl.style.display = "none";
            noImageEl.style.display = "block";
          }}
          const stamp = payload.timestamp ? new Date(payload.timestamp * 1000).toLocaleTimeString() : "unknown";
          document.getElementById("status").textContent = `Last update: ${{stamp}}`;
          const interval = payload.interval_ms || {interval_ms};
          setTimeout(updatePlot, interval);
        }} catch (err) {{
          document.getElementById("status").textContent = "Waiting for data...";
          setTimeout(updatePlot, {interval_ms});
        }}
      }}
      updatePlot();
    </script>
  </body>
</html>
"""


class _LivePlotHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            self._send_html(self.server.page_html)
            return
        if self.path.startswith("/frame"):
            image_bytes, mime = self.server.image_fn()
            if image_bytes is None:
                self.send_error(404, "No image")
                return
            self._send_bytes(image_bytes, mime)
            return
        if self.path.startswith("/data"):
            payload = self.server.payload_fn()
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

    def _send_json(self, payload: Mapping[str, Any]) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_bytes(self, payload: bytes, mime: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", mime)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)


class LivePlotServer:
    """Serve a live-updating plot over HTTP for headless use."""

    def __init__(
        self,
        title: str,
        x_label: str,
        y_label: str,
        host: str = "0.0.0.0",
        port: int = 8000,
        interval_ms: int = 200,
    ) -> None:
        self._lock = threading.Lock()
        self._payload: dict[str, Any] = {
            "x": [],
            "y": [],
            "title": title,
            "x_label": x_label,
            "y_label": y_label,
            "interval_ms": interval_ms,
            "timestamp": None,
        }
        self._image_bytes: bytes | None = None
        self._image_mime = "image/jpeg"
        self._image_ts: float | None = None
        self._server = ThreadingHTTPServer((host, port), _LivePlotHandler)
        self._server.payload_fn = self._get_payload
        self._server.image_fn = self._get_image
        self._server.page_html = _html_for_live_plot(title, x_label, y_label, interval_ms)
        self.host = host
        self.port = self._server.server_address[1]
        self._thread: threading.Thread | None = None

    def _get_payload(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._payload)

    def _get_image(self) -> tuple[bytes | None, str]:
        with self._lock:
            return self._image_bytes, self._image_mime

    def update(self, x: Iterable[float], y: Iterable[float], y_label: str | None = None) -> None:
        with self._lock:
            self._payload["x"] = _to_list(x)
            self._payload["y"] = _to_list(y)
            if y_label is not None:
                self._payload["y_label"] = y_label
            self._payload["timestamp"] = time.time()
            self._payload["has_image"] = self._image_bytes is not None
            self._payload["image_ts"] = self._image_ts

    def update_image(self, image_bytes: bytes, mime: str = "image/jpeg") -> None:
        with self._lock:
            self._image_bytes = image_bytes
            self._image_mime = mime
            self._image_ts = time.time()

    def start(self) -> None:
        if self._thread is not None:
            return
        thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        thread.start()
        self._thread = thread

    def stop(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
