#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP_DIR="$(mktemp -d /tmp/mr2-video-e2e.XXXXXX)"

GATEWAY_PORT="${GATEWAY_PORT:-18081}"
VIEWER_PORT="${VIEWER_PORT:-18082}"
BIND_HOST="${BIND_HOST:-0.0.0.0}"
STREAM_ID="front_nav_cam"

SIK_A="${TMP_DIR}/sik_a"
SIK_B="${TMP_DIR}/sik_b"
VIDEO_CONFIG="${TMP_DIR}/video_streams.json"
VIEWER_HTML="${TMP_DIR}/index.html"
SOCAT_LOG="${TMP_DIR}/socat.log"
GATEWAY_LOG="${TMP_DIR}/gateway.log"
ROVER_LOG="${TMP_DIR}/rover_video.log"
PUBLISHER_LOG="${TMP_DIR}/publisher.log"
HTTP_LOG="${TMP_DIR}/http.log"

PIDS=()

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  rm -rf "${TMP_DIR}"
}

trap cleanup EXIT INT TERM

cat >"${VIDEO_CONFIG}" <<'EOF'
{
  "version": 1,
  "streams": [
    {
      "stream_id": "front_nav_cam",
      "ros_topic": "/front_camera/image_raw",
      "udp_port": 5000,
      "ros_encoding": "rgb8",
      "width": 64,
      "height": 48,
      "framerate": 5,
      "encoder": {
        "bitrate_kbps": 500,
        "keyframe_interval": 5,
        "speed_preset": "ultrafast",
        "tune": "zerolatency"
      },
      "display": {
        "label": "Front Nav",
        "panel": "delivery",
        "order": 1
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
        display: grid;
        place-items: center;
        background:
          radial-gradient(circle at top, rgba(29, 170, 145, 0.24), transparent 30%),
          #0b0f14;
        color: #e7edf4;
        font: 16px/1.4 "Space Grotesk", system-ui, sans-serif;
      }
      main {
        width: min(92vw, 920px);
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
      canvas {
        width: 100%;
        aspect-ratio: 4 / 3;
        border-radius: 14px;
        background: #05070b;
        border: 1px solid rgba(255, 255, 255, 0.1);
      }
      code {
        color: #89f0de;
      }
      #status.live {
        color: #89f0de;
      }
      #status.error {
        color: #ff8c8c;
      }
    </style>
  </head>
  <body>
    <main>
      <section class="panel">
        <h1>MR2 Local Video E2E</h1>
        <p>Stream: <code>front_nav_cam</code></p>
        <p id="status">Connecting...</p>
        <p id="counts">config=0 key=0 delta=0</p>
      </section>
      <section class="panel">
        <canvas id="canvas" width="640" height="480"></canvas>
      </section>
    </main>
    <script>
      const STREAM_ID = 'front_nav_cam'
      const GATEWAY_PORT = __GATEWAY_PORT__
      const gatewayUrl = new URL(window.location.href)
      gatewayUrl.protocol = gatewayUrl.protocol === 'https:' ? 'wss:' : 'ws:'
      gatewayUrl.port = String(GATEWAY_PORT)
      gatewayUrl.pathname = '/video-ws'
      gatewayUrl.search = ''
      gatewayUrl.hash = ''
      const wsUrl = gatewayUrl.toString()
      const statusEl = document.getElementById('status')
      const countsEl = document.getElementById('counts')
      const canvas = document.getElementById('canvas')
      const ctx = canvas.getContext('2d')
      let decoder = null
      let decoderConfig = null
      let payloadFormat = 'avcc'
      let configCount = 0
      let keyCount = 0
      let deltaCount = 0

      function setStatus(text, className = '') {
        statusEl.textContent = text
        statusEl.className = className
      }

      function updateCounts() {
        countsEl.textContent = `config=${configCount} key=${keyCount} delta=${deltaCount}`
      }

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
        if (typeof VideoDecoder.isConfigSupported !== 'function') {
          return candidates[0]
        }
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

      if (!('VideoDecoder' in window)) {
        setStatus('WebCodecs is unavailable in this browser', 'error')
      } else {
        decoder = new VideoDecoder({
          output(frame) {
            if (canvas.width !== frame.displayWidth || canvas.height !== frame.displayHeight) {
              canvas.width = frame.displayWidth
              canvas.height = frame.displayHeight
            }
            ctx.drawImage(frame, 0, 0, canvas.width, canvas.height)
            frame.close()
            setStatus('Live', 'live')
          },
          error(error) {
            setStatus(`Decoder error: ${error.message}`, 'error')
          },
        })

        const ws = new WebSocket(wsUrl)
        ws.binaryType = 'arraybuffer'
        ws.addEventListener('open', () => {
          setStatus('Subscribed, waiting for config...')
          ws.send(JSON.stringify({ type: 'subscribe', stream_id: STREAM_ID }))
        })
        ws.addEventListener('message', (event) => {
          const message = decodeMessage(event.data)
          if (!message || message.streamId !== STREAM_ID) return

          if (message.kind === 'config') {
            void (async () => {
              configCount += 1
              updateCounts()
              const supported = await selectDecoderConfiguration(message)
              if (!supported) {
                decoderConfig = null
                setStatus('WebCodecs does not support this H.264 stream', 'error')
                return
              }
              decoderConfig = supported.config
              payloadFormat = supported.payloadFormat
              decoder.reset()
              decoder.configure(decoderConfig)
              setStatus('Configured, waiting for frame...')
            })()
            return
          }

          if (!decoderConfig) return
          if ((message.flags & 1) !== 0) keyCount += 1
          else deltaCount += 1
          updateCounts()
          decoder.decode(
            new EncodedVideoChunk({
              type: (message.flags & 1) !== 0 ? 'key' : 'delta',
              timestamp: message.timestamp,
              data: payloadFormat === 'annexb' ? message.payload : annexBToAvcc(message.payload),
            })
          )
        })
        ws.addEventListener('close', () => {
          setStatus('WebSocket closed', 'error')
        })
        ws.addEventListener('error', () => {
          setStatus('WebSocket error', 'error')
        })
      }
    </script>
  </body>
</html>
EOF

sed -i "s/__GATEWAY_PORT__/${GATEWAY_PORT}/g" "${VIEWER_HTML}"

start_bg() {
  "$@" &
  PIDS+=("$!")
}

wait_for_port() {
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

start_bg socat -d -d pty,raw,echo=0,link="${SIK_A}" pty,raw,echo=0,link="${SIK_B}"
sleep 1

start_bg bash -lc "
  source /opt/ros/humble/setup.bash &&
  source '${ROOT_DIR}/rover/ros2_ws/install/setup.bash' &&
  cd '${ROOT_DIR}/base/gateway' &&
  npm start -- --device '${SIK_A}' --baud 57600 --host '${BIND_HOST}' --port '${GATEWAY_PORT}' --video-config '${VIDEO_CONFIG}'
"

wait_for_port "http://127.0.0.1:${GATEWAY_PORT}/video/streams"

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
from rclpy.node import Node
from sensor_msgs.msg import Image

class Publisher(Node):
    def __init__(self):
        super().__init__('video_e2e_publisher')
        self.pub = self.create_publisher(Image, '/front_camera/image_raw', 10)
        self.frame = 0
        self.create_timer(0.2, self.tick)

    def tick(self):
        msg = Image()
        msg.header.frame_id = 'camera'
        msg.height = 48
        msg.width = 64
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = 64 * 3
        row = []
        for x in range(64):
            row.extend([
                ((x * 4) + self.frame * 10) % 256,
                (64 + self.frame * 3) % 256,
                (255 - (x * 4) - self.frame * 5) % 256,
            ])
        msg.data = row * 48
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.frame += 1

rclpy.init()
node = Publisher()
try:
    rclpy.spin(node)
finally:
    node.destroy_node()
    rclpy.shutdown()
PY
"

start_bg python3 -m http.server "${VIEWER_PORT}" --bind "${BIND_HOST}" --directory "${TMP_DIR}"

wait_for_port "http://127.0.0.1:${VIEWER_PORT}"

REMOTE_HOST="$(hostname -I 2>/dev/null | awk '{print $1}')"
if [[ -z "${REMOTE_HOST}" ]]; then
  REMOTE_HOST="127.0.0.1"
fi

cat <<EOF

Local MR2 video E2E stack is running.

Viewer URL:
  http://${REMOTE_HOST}:${VIEWER_PORT}

Gateway metadata URL:
  http://${REMOTE_HOST}:${GATEWAY_PORT}/video/streams

Bind address:
  ${BIND_HOST}

Temporary files live under:
  ${TMP_DIR}

Press Ctrl-C to stop everything.
EOF

wait -n "${PIDS[@]}"
