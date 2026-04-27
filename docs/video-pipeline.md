# MR2 Video Pipeline

This document describes the current rover-to-base-to-browser video path in the
`mr2-stack` repository.

## Overview

The video system is intentionally split into three stages:

1. rover-side ROS 2 + GStreamer encode
2. base-side GStreamer receive + Node.js gateway
3. browser-side WebSocket + WebCodecs decode

The design goal is low latency and freshness. The pipeline prefers dropping old
frames over accumulating delay.

## Data Flow

### 1. Central stream configuration

The authoritative stream mapping lives in:

- `rover/ros2_ws/src/mr2_launch/config/video_streams.json`

Each stream entry defines:

- `stream_id`
- `udp_port`
- `source`
- optional `width`
- optional `height`
- optional `framerate`
- encoder settings
- display metadata

Preferred source forms:

```json
{
  "stream_id": "front_nav_cam",
  "source": {
    "type": "ros_topic",
    "ros_topic": "/front_camera/image_raw",
    "ros_encoding": "rgb8"
  },
  "udp_port": 5000
}
```

```json
{
  "stream_id": "front_usb_cam",
  "source": {
    "type": "v4l2",
    "device": "/dev/video0",
    "pixel_format": "YUY2"
  },
  "udp_port": 5002,
  "width": 1280,
  "height": 720,
  "framerate": 30
}
```

Configuration note:

- every stream must declare an explicit `source` object
- top-level `ros_topic` / `ros_encoding` entries are not supported

The intended mapping rule is:

`1 ROS image topic = 1 H.264 stream = 1 UDP port = 1 stream ID`

### 2. Rover-side pipeline

The rover publisher is implemented in:

- `rover/ros2_ws/src/mr2_video_streaming/src/video_streaming_node.cpp`

Per configured stream, the node does one of two things:

- `source.type = ros_topic`
  - subscribes to `sensor_msgs/msg/Image`
  - accepts only `rgb8` and `bgr8`
  - normalizes frames to internal RGB
  - lazily creates one GStreamer pipeline on first valid frame
  - pushes frames into `appsrc`
- `source.type = v4l2`
  - starts a direct `v4l2src` GStreamer pipeline at node startup
  - optionally constrains the device by width, height, framerate, and pixel format
  - uses `jpegdec` automatically when `pixel_format` is `MJPG` or `JPEG`

For both source types, the rover node:

- timestamps encoded frames for low-latency live streaming
- keeps queue depth bounded so stale frames are dropped

Conceptual rover pipeline:

```text
appsrc
  ! queue leaky=downstream max-size-buffers=1
  ! videoconvert
  ! video/x-raw,format=I420
  ! x264enc ...
  ! h264parse config-interval=1
  ! rtph264pay pt=96 mtu=1200 config-interval=1
  ! udpsink host=<base_host> port=<udp_port> sync=false async=false
```

Conceptual direct V4L2 pipeline:

```text
v4l2src device=<device> do-timestamp=true
  ! queue leaky=downstream max-size-buffers=1
  ! [image/jpeg ... ! jpegdec] or [video/x-raw ...]
  ! videorate
  ! videoscale
  ! videoconvert
  ! video/x-raw,format=I420[,width=...][,height=...][,framerate=.../1]
  ! x264enc ...
  ! h264parse config-interval=1
  ! rtph264pay pt=96 mtu=1200 config-interval=1
  ! udpsink host=<base_host> port=<udp_port> sync=false async=false
```

Current implementation details:

- `x264enc`
- `bframes=0`
- `byte-stream=true`
- `threads=1`
- periodic SPS/PPS insertion via `h264parse` and `rtph264pay`
- keyframe cadence driven by `key-int-max`

### 3. Base-side receive pipeline

The base receiver is implemented in:

- `base/gateway/src/video/receiver.js`

Node.js starts one `gst-launch-1.0` child per configured stream. Each child:

- listens on exactly one UDP port
- receives RTP/H.264
- applies a small jitter stage
- depayloads RTP
- reparses H.264
- outputs Annex B byte-stream data aligned to access units

Conceptual receive pipeline:

```text
udpsrc port=<udp_port> caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000"
  ! rtpjitterbuffer latency=<ms> drop-on-latency=true
  ! rtph264depay
  ! h264parse disable-passthrough=true
  ! video/x-h264,stream-format=byte-stream,alignment=au
  ! fdsink fd=1 sync=false
```

Important boundary:

- Node.js does **not** parse raw RTP
- Node.js sees **depayloaded H.264 Annex B**
- each child stdout chunk is treated as a candidate access-unit boundary stream

### 4. Node.js H.264 handling

The base video gateway lives in:

- `base/gateway/src/video/service.js`
- `base/gateway/src/video/h264.js`
- `base/gateway/src/video/protocol.js`

Per stream, the gateway maintains:

- latest SPS
- latest PPS
- current codec string
- latest decodable key access unit
- stream availability
- per-client bootstrap state

The Annex B parser performs:

- start-code scanning
- NAL extraction
- SPS/PPS detection
- IDR detection
- access-unit grouping
- codec string derivation from SPS

The gateway only sends one browser message per access unit.

### 5. Browser-side decode

The browser-side implementation lives in:

- `dashboard/src/lib/videoGateway.ts`
- `dashboard/src/lib/videoProtocol.ts`
- `dashboard/src/components/VideoStreamCard.tsx`

The browser:

- fetches stream metadata from `/video/streams`
- subscribes to one or more `stream_id`s on `/video-ws`
- receives `config` and `chunk` binary messages
- configures a `VideoDecoder`
- converts Annex B payloads to AVCC when required
- decodes with WebCodecs
- renders frames to `<canvas>`

The dashboard resolves `/video-ws` and `/video/streams` relative to the
current browser origin unless explicit environment overrides are set.

`/video/streams` exposes the configured source metadata per stream, including:

- `source_type`
- `ros_topic`
- `ros_encoding`
- `v4l2_device`
- `v4l2_pixel_format`

## Browser Caveats

### Safari raw-IP WebSocket instability

Observed during local testing:

- Firefox handled the temporary video viewer reliably, including gateway restart
  and reconnect behavior.
- Recent Safari builds were intermittently flaky when the viewer page and
  `/video-ws` were accessed over a raw IP address such as
  `http://100.x.y.z:18081` and `ws://100.x.y.z:18081/video-ws`.
- The failure mode looked like a first-load browser issue: the page could appear
  stuck even though the same stack worked in Firefox, and a refresh or a second
  navigation could unstick it.

Current interpretation:

- treat this as a Safari/browser-path issue before treating it as a rover/base
  video pipeline defect
- do not rely on Safari Web Inspector alone to prove that a subscribe message
  was or was not sent

Recommended workaround when testing with Safari:

- use a hostname instead of a raw IP address if possible
- prefer `https` + `wss` over plain `http` + `ws` when available

## WebSocket Protocol

The video socket is separate from the control socket:

- control: `/xbee-ws`
- video: `/video-ws`

### Client subscription message

Clients subscribe with JSON:

```json
{"type":"subscribe","stream_id":"front_nav_cam"}
```

Clients may unsubscribe with:

```json
{"type":"unsubscribe","stream_id":"front_nav_cam"}
```

### Binary message types

Binary framing is implemented in:

- `base/gateway/src/video/protocol.js`
- `dashboard/src/lib/videoProtocol.ts`

#### `config`

Sent before normal chunk delivery when a client first joins or needs decoder
re-bootstrap.

Fields:

- `type = 1`
- `stream_id`
- `codec`
- `sps`
- `pps`
- `width`
- `height`

#### `chunk`

One message per encoded access unit.

Fields:

- `type = 2`
- `stream_id`
- `flags`
- `timestamp_us`
- `payload`

Current flags:

- bit 0: key
- bit 1: delta

## Bootstrap Rules

A client does not start mid-stream on arbitrary delta units.

The gateway waits until it has:

- valid SPS
- valid PPS
- a decodable key access unit

Then it sends:

1. `config`
2. first key `chunk`
3. subsequent delta/key chunks

If codec state changes, the gateway bumps a per-stream config version and forces
subscribers back through the same bootstrap path.

## Low-Latency Behavior

The current implementation explicitly favors freshness:

- rover `appsrc` uses bounded buffering
- rover queue is `leaky=downstream`
- base receive pipeline uses a bounded jitter buffer
- per-client websocket sending checks `bufferedAmount`
- slow clients are marked for re-bootstrap rather than stalling ingest
- old keyframes may be replaced by newer ones in cache

Acceptable outcomes:

- dropped frames
- dropped deltas
- short visual gaps after loss
- waiting for next keyframe after reconnect

Unacceptable outcome:

- unbounded latency growth

## Current File Map

### Rover

- `rover/ros2_ws/src/mr2_video_streaming/src/video_streaming_node.cpp`
- `rover/ros2_ws/src/mr2_launch/config/video_streams.json`
- `rover/ros2_ws/src/mr2_launch/launch/video_streaming.launch.py`

### Base gateway

- `base/gateway/src/video/stream_config.js`
- `base/gateway/src/video/receiver.js`
- `base/gateway/src/video/h264.js`
- `base/gateway/src/video/protocol.js`
- `base/gateway/src/video/service.js`
- `base/gateway/src/runtime/ws_route_registry.js`
- `base/gateway/src/runtime/http_handlers.js`

### Dashboard

- `dashboard/src/lib/videoGateway.ts`
- `dashboard/src/lib/videoProtocol.ts`
- `dashboard/src/hooks/useVideoStreams.ts`
- `dashboard/src/components/ConfiguredVideoGrid.tsx`
- `dashboard/src/components/VideoStreamCard.tsx`

## Local End-to-End Runner

For local browser verification, the repository also includes:

- `scripts/tmp_run_video_e2e.sh`

That temporary runner:

- starts a fake local XBEE link
- starts the base gateway
- starts the rover video node
- publishes a synthetic ROS image stream
- serves a lightweight browser viewer
- proxies `/video/streams` and `/video-ws` through one browser-facing port

This script is for operator testing only. It is not part of the production
runtime path.
