#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP_DIR="$(mktemp -d /tmp/mr2-video-e2e.XXXXXX)"

GATEWAY_PORT="${GATEWAY_PORT:-18081}"
VIEWER_PORT="${VIEWER_PORT:-18082}"
BIND_HOST="${BIND_HOST:-0.0.0.0}"
ROS_STREAM_ID="front_nav_cam"
V4L2_STREAM_ID="loopback_cam"
V4L2_VIDEO_NR="${V4L2_VIDEO_NR:-30}"
V4L2_DEVICE="/dev/video${V4L2_VIDEO_NR}"

SIK_A="${TMP_DIR}/sik_a"
SIK_B="${TMP_DIR}/sik_b"
VIDEO_CONFIG="${TMP_DIR}/video_streams.json"
VIEWER_HTML="${TMP_DIR}/index.html"
VIEWER_PROXY_JS="${TMP_DIR}/viewer_proxy.js"

PIDS=()
LOOPBACK_LOADED=0

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done

  if [[ "${LOOPBACK_LOADED}" == "1" ]]; then
    sudo -n modprobe -r v4l2loopback >/dev/null 2>&1 || true
  fi

  rm -rf "${TMP_DIR}"
}

trap cleanup EXIT INT TERM

cat >"${VIDEO_CONFIG}" <<EOF
{
  "version": 1,
  "streams": [
    {
      "stream_id": "${ROS_STREAM_ID}",
      "source": {
        "type": "ros_topic",
        "ros_topic": "/front_camera/image_raw",
        "ros_encoding": "rgb8"
      },
      "udp_port": 5000,
      "width": 320,
      "height": 180,
      "framerate": 30,
      "encoder": {
        "bitrate_kbps": 900,
        "keyframe_interval": 30,
        "speed_preset": "ultrafast",
        "tune": "zerolatency"
      },
      "display": {
        "label": "ROS Topic",
        "panel": "delivery",
        "order": 1
      }
    },
    {
      "stream_id": "${V4L2_STREAM_ID}",
      "source": {
        "type": "v4l2",
        "device": "${V4L2_DEVICE}",
        "pixel_format": "YUY2"
      },
      "udp_port": 5002,
      "width": 320,
      "height": 180,
      "framerate": 30,
      "encoder": {
        "bitrate_kbps": 900,
        "keyframe_interval": 30,
        "speed_preset": "ultrafast",
        "tune": "zerolatency"
      },
      "display": {
        "label": "V4L2 Loopback",
        "panel": "delivery",
        "order": 2
      }
    }
  ]
}
EOF

cat >"${VIEWER_HTML}" <<'EOF'
<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1" />
    <title>MR2 Video E2E</title>
    <style>
      body {
        margin: 0;
        min-height: 100vh;
        background:
          radial-gradient(circle at top, rgba(29, 170, 145, 0.24), transparent 30%),
          #0b0f14;
        color: #e7edf4;
        font: 16px/1.4 "Space Grotesk", system-ui, sans-serif;
      }
      main {
        width: min(96vw, 1200px);
        margin: 0 auto;
        padding: 24px;
        display: grid;
        gap: 16px;
      }
      .panel {
        padding: 18px;
        border-radius: 18px;
        border: 1px solid rgba(255, 255, 255, 0.08);
        background: rgba(6, 10, 18, 0.88);
        box-shadow: 0 20px 60px rgba(0, 0, 0, 0.35);
      }
      .streams {
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(320px, 1fr));
        gap: 16px;
      }
      canvas {
        width: 100%;
        aspect-ratio: 16 / 9;
        border-radius: 14px;
        background: #05070b;
        border: 1px solid rgba(255, 255, 255, 0.1);
      }
      .meta {
        display: flex;
        flex-wrap: wrap;
        align-items: baseline;
        gap: 12px;
        margin: 8px 0 12px;
        color: rgba(231, 237, 244, 0.78);
        font-size: 14px;
      }
      .meta > span {
        flex: 1 1 180px;
        min-width: 0;
      }
      .meta > span:last-child {
        text-align: right;
        font-variant-numeric: tabular-nums;
      }
      .status-live {
        color: #89f0de;
      }
      .status-error {
        color: #ff8c8c;
      }
      code {
        color: #89f0de;
      }
    </style>
  </head>
  <body>
    <main>
      <section class="panel">
        <h1>MR2 Local Video E2E</h1>
        <p>Single gateway, two streams: one ROS topic and one direct V4L2 loopback device.</p>
        <p id="socket-status">Connecting...</p>
      </section>
      <section class="streams">
        <article class="panel">
          <h2>ROS Topic</h2>
          <p><code>front_nav_cam</code></p>
          <div class="meta">
            <span id="front_nav_cam-status">Waiting for socket...</span>
            <span id="front_nav_cam-counts">config=0 key=0 delta=0</span>
          </div>
          <canvas id="front_nav_cam-canvas" width="320" height="180"></canvas>
        </article>
        <article class="panel">
          <h2>V4L2 Loopback</h2>
          <p><code>loopback_cam</code></p>
          <div class="meta">
            <span id="loopback_cam-status">Waiting for socket...</span>
            <span id="loopback_cam-counts">config=0 key=0 delta=0</span>
          </div>
          <canvas id="loopback_cam-canvas" width="320" height="180"></canvas>
        </article>
      </section>
    </main>
    <script>
      const STREAM_IDS = ['front_nav_cam', 'loopback_cam']
      const gatewayUrl = new URL(window.location.href)
      gatewayUrl.protocol = gatewayUrl.protocol === 'https:' ? 'wss:' : 'ws:'
      gatewayUrl.pathname = '/video-ws'
      gatewayUrl.search = ''
      gatewayUrl.hash = ''
      const wsUrl = gatewayUrl.toString()
      const socketStatusEl = document.getElementById('socket-status')

      function findStartCodeLength(bytes, offset) {
        if (offset + 3 > bytes.length) return 0
        if (bytes[offset] !== 0 || bytes[offset + 1] !== 0) return 0
        if (bytes[offset + 2] === 1) return 3
        if (offset + 4 <= bytes.length && bytes[offset + 2] === 0 && bytes[offset + 3] === 1) return 4
        return 0
      }

      function splitAnnexBNalus(bytes) {
        const starts = []
        for (let index = 0; index < bytes.length - 2; index += 1) {
          const length = findStartCodeLength(bytes, index)
          if (length > 0) {
            starts.push(index)
            index += length - 1
          }
        }
        const nals = []
        for (let index = 0; index < starts.length; index += 1) {
          const start = starts[index]
          const next = index + 1 < starts.length ? starts[index + 1] : bytes.length
          const startCode = findStartCodeLength(bytes, start)
          nals.push(bytes.slice(start + startCode, next))
        }
        return nals.filter((nal) => nal.length > 0)
      }

      function annexBToAvcc(bytes) {
        const nals = splitAnnexBNalus(bytes)
        const totalLength = nals.reduce((sum, nal) => sum + 4 + nal.length, 0)
        const output = new Uint8Array(totalLength)
        const view = new DataView(output.buffer)
        let offset = 0
        for (const nal of nals) {
          view.setUint32(offset, nal.length, false)
          offset += 4
          output.set(nal, offset)
          offset += nal.length
        }
        return output
      }

      function buildDescription(sps, pps) {
        const output = new Uint8Array(11 + sps.length + pps.length)
        let offset = 0
        output[offset++] = 1
        output[offset++] = sps[1] ?? 0x42
        output[offset++] = sps[2] ?? 0xe0
        output[offset++] = sps[3] ?? 0x1f
        output[offset++] = 0xff
        output[offset++] = 0xe1
        output[offset++] = (sps.length >> 8) & 0xff
        output[offset++] = sps.length & 0xff
        output.set(sps, offset)
        offset += sps.length
        output[offset++] = 1
        output[offset++] = (pps.length >> 8) & 0xff
        output[offset++] = pps.length & 0xff
        output.set(pps, offset)
        return output
      }

      function deriveCodecString(sps) {
        if (!sps || sps.length < 4) return 'avc1.42E01F'
        const profile = sps[1].toString(16).padStart(2, '0').toUpperCase()
        const constraints = sps[2].toString(16).padStart(2, '0').toUpperCase()
        const level = sps[3].toString(16).padStart(2, '0').toUpperCase()
        return `avc1.${profile}${constraints}${level}`
      }

      function normalizeCodecString(codec, sps) {
        if (!codec) return deriveCodecString(sps)
        if (/^avc1\./i.test(codec)) return `avc1.${codec.slice(5)}`
        return deriveCodecString(sps)
      }

      function buildDecoderCandidates(message) {
        const baseConfig = {
          codec: normalizeCodecString(message.codec, message.sps),
          codedWidth: message.width,
          codedHeight: message.height,
          hardwareAcceleration: 'prefer-hardware',
        }
        return [
          {
            config: {
              ...baseConfig,
              description: buildDescription(message.sps, message.pps),
            },
            payloadFormat: 'avcc',
          },
          {
            config: baseConfig,
            payloadFormat: 'annexb',
          },
        ]
      }

      async function selectDecoderConfiguration(message) {
        const candidates = buildDecoderCandidates(message)
        if (typeof VideoDecoder.isConfigSupported !== 'function') return candidates[0]
        for (const candidate of candidates) {
          try {
            const result = await VideoDecoder.isConfigSupported(candidate.config)
            if (result.supported) return candidate
          } catch (_error) {
            continue
          }
        }
        return null
      }

      function decodeMessage(buffer) {
        const view = new DataView(buffer)
        const type = view.getUint8(0)
        if (type === 1) {
          const streamIdLength = view.getUint16(1, true)
          const codecLength = view.getUint16(3, true)
          const spsLength = view.getUint32(5, true)
          const ppsLength = view.getUint32(9, true)
          const width = view.getUint16(13, true)
          const height = view.getUint16(15, true)
          const bytes = new Uint8Array(buffer)
          let offset = 17
          const streamId = new TextDecoder().decode(bytes.slice(offset, offset + streamIdLength))
          offset += streamIdLength
          const codec = new TextDecoder().decode(bytes.slice(offset, offset + codecLength))
          offset += codecLength
          const sps = bytes.slice(offset, offset + spsLength)
          offset += spsLength
          const pps = bytes.slice(offset, offset + ppsLength)
          return { kind: 'config', streamId, codec, width, height, sps, pps }
        }
        if (type === 2) {
          const streamIdLength = view.getUint16(1, true)
          const flags = view.getUint8(3)
          const timestamp = Number(view.getBigUint64(4, true))
          const payloadLength = view.getUint32(12, true)
          const bytes = new Uint8Array(buffer)
          const streamId = new TextDecoder().decode(bytes.slice(16, 16 + streamIdLength))
          const payload = bytes.slice(16 + streamIdLength, 16 + streamIdLength + payloadLength)
          return { kind: 'chunk', streamId, flags, timestamp, payload }
        }
        return null
      }

      function setSocketStatus(text, className = '') {
        socketStatusEl.textContent = text
        socketStatusEl.className = className
      }

      const streams = Object.fromEntries(
        STREAM_IDS.map((streamId) => [
          streamId,
          {
            streamId,
            canvas: document.getElementById(`${streamId}-canvas`),
            statusEl: document.getElementById(`${streamId}-status`),
            countsEl: document.getElementById(`${streamId}-counts`),
            decoder: null,
            decoderConfig: null,
            payloadFormat: 'avcc',
            configCount: 0,
            keyCount: 0,
            deltaCount: 0,
          },
        ])
      )

      function setStreamStatus(stream, text, className = '') {
        stream.statusEl.textContent = text
        stream.statusEl.className = className
      }

      function updateCounts(stream) {
        stream.countsEl.textContent = `config=${stream.configCount} key=${stream.keyCount} delta=${stream.deltaCount}`
      }

      if (!('VideoDecoder' in window)) {
        setSocketStatus('WebCodecs is unavailable in this browser', 'status-error')
        for (const stream of Object.values(streams)) {
          setStreamStatus(stream, 'WebCodecs unavailable', 'status-error')
        }
      } else {
        let reconnectTimer = null
        let socketAttempt = 0

        for (const stream of Object.values(streams)) {
          stream.decoder = new VideoDecoder({
            output(frame) {
              const canvas = stream.canvas
              const ctx = canvas.getContext('2d')
              if (!ctx) {
                frame.close()
                return
              }
              if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
                canvas.width = frame.displayWidth
                canvas.height = frame.displayHeight
              }
              ctx.drawImage(frame, 0, 0, canvas.width, canvas.height)
              frame.close()
              setStreamStatus(stream, 'Live', 'status-live')
            },
            error(error) {
              setStreamStatus(stream, `Decoder error: ${error.message}`, 'status-error')
            },
          })
        }

        function scheduleReconnect() {
          if (reconnectTimer != null) return
          reconnectTimer = window.setTimeout(() => {
            reconnectTimer = null
            connectSocket()
          }, 1000)
        }

        function connectSocket() {
          socketAttempt += 1
          setSocketStatus(`Connecting (attempt ${socketAttempt})...`)
          for (const stream of Object.values(streams)) {
            setStreamStatus(stream, 'Waiting for socket...')
          }

          const ws = new WebSocket(wsUrl)
          ws.binaryType = 'arraybuffer'

          ws.addEventListener('open', () => {
            socketAttempt = 0
            setSocketStatus('Connected', 'status-live')
            for (const streamId of STREAM_IDS) {
              ws.send(JSON.stringify({ type: 'subscribe', stream_id: streamId }))
              setStreamStatus(streams[streamId], 'Subscribed, waiting for config...')
            }
          })

          ws.addEventListener('message', (event) => {
            const message = decodeMessage(event.data)
            if (!message) return
            const stream = streams[message.streamId]
            if (!stream) return

            if (message.kind === 'config') {
              void (async () => {
                stream.configCount += 1
                updateCounts(stream)
                const supported = await selectDecoderConfiguration(message)
                if (!supported || !stream.decoder) {
                  stream.decoderConfig = null
                  setStreamStatus(stream, 'WebCodecs does not support this H.264 stream', 'status-error')
                  return
                }
                stream.decoderConfig = supported.config
                stream.payloadFormat = supported.payloadFormat
                stream.decoder.reset()
                stream.decoder.configure(stream.decoderConfig)
                setStreamStatus(stream, 'Configured, waiting for frame...')
              })()
              return
            }

            if (!stream.decoderConfig || !stream.decoder) return
            if ((message.flags & 1) !== 0) stream.keyCount += 1
            else stream.deltaCount += 1
            updateCounts(stream)
            stream.decoder.decode(
              new EncodedVideoChunk({
                type: (message.flags & 1) !== 0 ? 'key' : 'delta',
                timestamp: message.timestamp,
                data:
                  stream.payloadFormat === 'annexb'
                    ? message.payload
                    : annexBToAvcc(message.payload),
              })
            )
          })

          ws.addEventListener('close', () => {
            setSocketStatus('WebSocket closed, retrying...', 'status-error')
            for (const stream of Object.values(streams)) {
              setStreamStatus(stream, 'Socket closed, retrying...', 'status-error')
            }
            scheduleReconnect()
          })

          ws.addEventListener('error', () => {
            setSocketStatus('WebSocket error, retrying...', 'status-error')
            for (const stream of Object.values(streams)) {
              setStreamStatus(stream, 'Socket error, retrying...', 'status-error')
            }
            scheduleReconnect()
          })
        }

        connectSocket()
      }
    </script>
  </body>
</html>
EOF

cat >"${VIEWER_PROXY_JS}" <<'EOF'
const fs = require('fs')
const http = require('http')
const net = require('net')

const bindHost = process.env.BIND_HOST || '0.0.0.0'
const viewerPort = Number(process.env.VIEWER_PORT || '18082')
const gatewayPort = Number(process.env.GATEWAY_PORT || '18081')
const viewerHtml = process.env.VIEWER_HTML

const server = http.createServer((req, res) => {
  if (!req.url) {
    res.writeHead(400)
    res.end('missing url')
    return
  }

  if (req.url === '/' || req.url.startsWith('/?')) {
    res.writeHead(200, { 'Content-Type': 'text/html; charset=utf-8' })
    res.end(fs.readFileSync(viewerHtml))
    return
  }

  if (req.url.startsWith('/video/')) {
    const upstream = http.request(
      {
        host: '127.0.0.1',
        port: gatewayPort,
        path: req.url,
        method: req.method,
        headers: req.headers,
      },
      (upstreamRes) => {
        res.writeHead(upstreamRes.statusCode || 502, upstreamRes.headers)
        upstreamRes.pipe(res)
      }
    )
    upstream.on('error', (error) => {
      res.writeHead(502)
      res.end(String(error))
    })
    req.pipe(upstream)
    return
  }

  res.writeHead(404)
  res.end('not found')
})

server.on('upgrade', (req, socket, head) => {
  if (req.url !== '/video-ws') {
    socket.destroy()
    return
  }

  const upstream = net.connect(gatewayPort, '127.0.0.1', () => {
    const headers = [
      'GET /video-ws HTTP/1.1',
      'Host: 127.0.0.1',
      'Connection: Upgrade',
      'Upgrade: websocket',
      `Sec-WebSocket-Key: ${req.headers['sec-websocket-key'] || ''}`,
      `Sec-WebSocket-Version: ${req.headers['sec-websocket-version'] || '13'}`,
    ]

    if (req.headers.origin) headers.push(`Origin: ${req.headers.origin}`)
    if (req.headers['sec-websocket-protocol']) {
      headers.push(`Sec-WebSocket-Protocol: ${req.headers['sec-websocket-protocol']}`)
    }
    headers.push('\r\n')
    upstream.write(headers.join('\r\n'))
    if (head && head.length > 0) {
      upstream.write(head)
    }
  })

  upstream.on('data', (chunk) => socket.write(chunk))
  socket.on('data', (chunk) => upstream.write(chunk))
  upstream.on('error', () => socket.destroy())
  upstream.on('close', () => socket.destroy())
  socket.on('error', () => upstream.destroy())
  socket.on('close', () => upstream.destroy())
})

server.listen(viewerPort, bindHost, () => {
  console.log(`Viewer proxy listening on ${bindHost}:${viewerPort}`)
})
EOF

start_bg() {
  "$@" &
  PIDS+=("$!")
}

wait_for_http() {
  local url="$1"
  for _ in $(seq 1 60); do
    if curl -fsS "${url}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.5
  done
  echo "Timed out waiting for ${url}" >&2
  return 1
}

rm -f "${SIK_A}" "${SIK_B}"

sudo -n modprobe v4l2loopback devices=1 video_nr="${V4L2_VIDEO_NR}" card_label=codex-loopback exclusive_caps=1
LOOPBACK_LOADED=1
sleep 1

start_bg socat -d -d pty,raw,echo=0,link="${SIK_A}" pty,raw,echo=0,link="${SIK_B}"
sleep 1

start_bg gst-launch-1.0 -q \
  videotestsrc is-live=true pattern=ball \
  ! timeoverlay shaded-background=true \
  ! video/x-raw,width=320,height=180,framerate=30/1 \
  ! videoconvert \
  ! video/x-raw,format=YUY2 \
  ! v4l2sink device="${V4L2_DEVICE}" sync=false

sleep 1
v4l2-ctl -d "${V4L2_DEVICE}" -c sustain_framerate=1
v4l2loopback-ctl set-fps 30 "${V4L2_DEVICE}"

start_bg bash -lc "
  source /opt/ros/humble/setup.bash &&
  source '${ROOT_DIR}/rover/ros2_ws/install/setup.bash' &&
  cd '${ROOT_DIR}/base/gateway' &&
  npm start -- --device '${SIK_A}' --baud 57600 --host '${BIND_HOST}' --port '${GATEWAY_PORT}' --video-config '${VIDEO_CONFIG}' --video-jitter-ms 0
"

wait_for_http "http://127.0.0.1:${GATEWAY_PORT}/video/streams"

start_bg bash -lc "
  source /opt/ros/humble/setup.bash &&
  source '${ROOT_DIR}/rover/ros2_ws/install/setup.bash' &&
  ros2 run mr2_video_streaming video_streaming_node --ros-args -p video_config_path:='${VIDEO_CONFIG}' -p base_host:=127.0.0.1
"

sleep 1

start_bg bash -lc "
  source /opt/ros/humble/setup.bash &&
  source '${ROOT_DIR}/rover/ros2_ws/install/setup.bash' &&
  python3 - <<'PY'
import rclpy
import time
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image

class Publisher(Node):
    BAR_COLORS = np.array(
        [
            (255, 255, 255),
            (255, 255, 0),
            (0, 255, 255),
            (0, 255, 0),
            (255, 0, 255),
            (255, 0, 0),
            (0, 0, 255),
            (24, 24, 24),
        ],
        dtype=np.uint8,
    )

    def __init__(self):
        super().__init__('video_test_pattern_publisher')
        self.publisher = self.create_publisher(Image, '/front_camera/image_raw', 10)
        self.start = time.monotonic()
        self.width = 320
        self.height = 180
        self.frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        bar_width = self.width // len(self.BAR_COLORS)
        for index, color in enumerate(self.BAR_COLORS):
            start_x = index * bar_width
            end_x = self.width if index == len(self.BAR_COLORS) - 1 else (index + 1) * bar_width
            self.frame[:, start_x:end_x, :] = color
        self.create_timer(1.0 / 30.0, self.tick)

    def tick(self):
        elapsed = time.monotonic() - self.start
        frame = self.frame.copy()
        seconds_text = f'ROS {elapsed:06.2f}s'
        cv2.rectangle(frame, (10, 10), (190, 52), (0, 0, 0), thickness=-1)
        cv2.putText(
            frame,
            seconds_text,
            (18, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = self.height
        msg.width = self.width
        msg.encoding = 'rgb8'
        msg.step = self.width * 3
        msg.data = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB).tobytes()
        self.publisher.publish(msg)

rclpy.init()
node = Publisher()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
"

start_bg env \
  BIND_HOST="${BIND_HOST}" \
  VIEWER_PORT="${VIEWER_PORT}" \
  GATEWAY_PORT="${GATEWAY_PORT}" \
  VIEWER_HTML="${VIEWER_HTML}" \
  node "${VIEWER_PROXY_JS}"

wait_for_http "http://127.0.0.1:${VIEWER_PORT}/"

echo "Viewer: http://$(hostname -I | awk '{print $1}'):${VIEWER_PORT}"
echo "Metadata: http://$(hostname -I | awk '{print $1}'):${VIEWER_PORT}/video/streams"
echo "Config: ${VIDEO_CONFIG}"

wait
