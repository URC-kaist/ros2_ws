# Latency diagnostics MVP implementation plan

> Superseded for video-link measurement: the T6/motion-onset response-frame
> method described below was removed. Rocket M2 latency is now measured with
> matched rover/base RTP pcaps according to
> `docs/rtp-link-latency-implementation-plan.md`. T0/T1/T3/T4 command timing and
> T7a/T7b/T7c browser-boundary timing remain, but T6 is no longer implemented.

This document is the durable implementation plan for the manual-control latency
experiment. It is intentionally more detailed than a normal task checklist so
the work can resume after context loss without changing the timing semantics or
the scope of the MVP.

## 1. Goal and scope

The MVP must let an operator arm a latency trial from the dashboard, make one
drive input from neutral, and see/log enough source timestamps to divide the
perceived delay into these coarse regions:

1. computer to rover: `T3 - T0`;
2. rover execution: `T6 - T3`;
3. video back to the computer: approximately `T7c - T6`, with the post-base
   portion split into `T7a`, `T7b`, and `T7c`.

The timestamp contract is:

| Stage | Source | Definition |
| --- | --- | --- |
| `T0` | dashboard | First armed gamepad input crossing the drive threshold from neutral. |
| `T1` | base gateway | Receipt of the tagged `cmd_drive` WebSocket message. Used for correlation and extra diagnosis. |
| `T3` | rover XBEE bridge | Successful decode of the correlated `CMD_DRIVE` frame, before smoothing. |
| `T4` | rover XBEE bridge | Publication of the smoothed `/base/cmd_vel`; retained as an extra diagnostic stage. |
| `T6` | rover XBEE bridge/probe | Wheel odometry exceeds a configurable linear or angular threshold for a configurable number of consecutive samples. |
| `T7a` | base video receiver | Annex-B access unit emitted after RTP jitter buffering, depayloading, and H.264 parsing. This is not raw base-NIC arrival. |
| `T7b` | dashboard | Binary video WebSocket `message` callback entry. |
| `T7c` | dashboard | WebCodecs output frame has been drawn to the canvas. |

The MVP does not add rover capture/RTP-send or base-NIC receive timestamps.
Therefore `T7a - T6` is a coarse pre-browser video measurement, not a pure
Rocket M2 one-way measurement. The first frame whose base-ingest time is at or
after `T6` is used as the trial's response frame. This is an approximation: a
future LED/ROI detector is required to prove that the selected frame visibly
contains physical motion.

## 2. Non-goals and safety constraints

- Do not change existing XBEE payload layouts or message IDs.
- Do not change control smoothing, deadman behavior, controller limits, video
  bitrate, or jitter-buffer policy.
- Do not run a real rover, `can0`, homing, flashing, deployment, or calibration
  command as part of software validation.
- Diagnostics must be disabled on the rover by default and must not emit
  per-controller-cycle console logs.
- Do not log encoded video payloads, credentials, or full telemetry.
- Browser observation must use a bounded in-memory log and update React at a
  low rate so the measurement UI does not create the latency being measured.

## 3. Correlation and clock contract

### 3.1 Trial correlation without an XBEE protocol change

The dashboard adds diagnostic-only fields to the first non-zero `cmd_drive`
sent after an armed T0:

```json
{
  "type": "cmd_drive",
  "trial_id": "<session>-000001",
  "client_tx_epoch_us": 1786351234567000,
  "linear_x_m_s": 0.4,
  "linear_y_m_s": 0.0,
  "angular_z_rad_s": 0.0
}
```

The base gateway still encodes the existing CMD_DRIVE payload. It broadcasts a
diagnostic mapping containing `trial_id`, the encoded frame header sequence,
and the existing 32-bit wire timestamp. The correlation key is:

```text
(xbee_seq, wire_timestamp_ms)
```

The rover publishes the same key with T3/T4 and later T6 on
`/latency/trace`. Sequence wrap is safe because the timestamp is part of the
key. The dashboard must support either arrival order: gateway mapping first or
rover trace first.

### 3.2 Clock sources

- Browser timestamps use `performance.timeOrigin + performance.now()` and are
  converted to integer epoch microseconds.
- Gateway timestamps use epoch microseconds derived from `Date.now()`; 1 ms
  resolution is acceptable for the MVP.
- Rover timestamps use `std::chrono::system_clock` epoch microseconds. ROS time
  is not used for cross-host latency because simulation time is a separate
  clock domain.
- `/latency/time` provides an NTP-style browser-to-gateway offset estimate.
  The browser samples it several times and uses the median offset among the
  lowest-RTT samples.
- Rover and base system clocks must be synchronized externally with chrony/PTP
  for field measurements. Raw source timestamps are always retained so an
  offset can be corrected offline. The UI must label rover-derived deltas as
  requiring synchronized rover/base clocks.

### 3.3 Log schema

Every dashboard observation uses this shape and is exported as JSON Lines:

```json
{
  "schema_version": 1,
  "session_id": "latency-20260810T120000Z",
  "trial_id": "latency-20260810T120000Z-000001",
  "stage": "video_browser_render",
  "source": "dashboard",
  "stream_id": "front_left_cam",
  "sequence": 1842,
  "epoch_us": 1786351234567000,
  "delta_t0_ms": 287.4,
  "metadata": {}
}
```

Raw `epoch_us` is authoritative. `delta_t0_ms` is a derived convenience value.
The bounded dashboard log drops the oldest records when full and exposes a
drop count in the UI/export.

## 4. Large implementation plan

### Phase A: dashboard foundation and clock synchronization

Create the diagnostics store, trial state machine, T0 capture, JSONL export,
and persistent dashboard panel. Add the base time endpoint and browser offset
sampling. The panel must work even when the rover is offline.

Deliverable: an operator can arm a trial, move the gamepad from neutral, see T0,
select a video stream, view clock health, reset the session, and export JSONL.

### Phase B: gateway mapping and rover T3/T4 trace

Tag only the first command after T0. The gateway broadcasts the correlation
mapping. The rover publishes decoded command and `/base/cmd_vel` publish times
on `/latency/trace` when diagnostics are enabled. The dashboard correlates
out-of-order gateway/rover events and fills T1/T3/T4.

Deliverable: `T3 - T0` is visible and logged without changing the XBEE wire
protocol.

### Phase C: rover T6 motion onset

Extend the rover diagnostics with wheel-odometry onset detection associated
with the first non-zero command after neutral. Use linear and angular
thresholds plus consecutive-sample filtering. Reset the onset detector after
the raw target returns to neutral.

Deliverable: `T6 - T3` is visible and logged for drive trials.

### Phase D: video T7a/T7b/T7c

Preserve the existing base-ingest timestamp semantically, capture browser
receive time before decoding, and capture canvas-draw completion time. Associate
decoder output with chunk metadata using `(stream_id, chunk timestamp)`. Record
per-frame data only for the selected diagnostic stream. Once T6 exists, choose
the first rendered frame with `T7a >= T6` as the MVP response frame.

Deliverable: the panel shows `T7a-T0`, `T7b-T0`, `T7c-T0`, `T7b-T7a`, and
`T7c-T7b`, plus the coarse `T7c-T6` video-back value.

### Phase E: integration, performance guardrails, and operator documentation

Exercise reconnect, missing/out-of-order trace, decoder reset, log bounds, and
diagnostics-disabled behavior. Update the nearest READMEs with launch and field
usage. Run all applicable gateway/dashboard/ROS checks and report hardware
validation as not performed.

## 5. Small implementation plan

The following checklist is the exact intended implementation order.

### A1. Gateway time endpoint

Files:

- `base/gateway/src/runtime/http_handlers.js`
- `base/gateway/test/http_handlers.test.js`
- `scripts/nginx/mr2-dashboard-locations.conf`

Implement `GET /latency/time`. Capture receive time at handler entry and send
time immediately before JSON serialization. Return no-cache JSON with integer
`server_receive_epoch_us` and `server_send_epoch_us`. Add a gateway test for
status, headers, monotonic ordering, and fields. Add the same-origin nginx
location.

Test: `cd base/gateway && npm test`.

### A2. Dashboard diagnostics singleton

New files:

- `dashboard/src/lib/latencyDiagnostics.ts`
- `dashboard/src/hooks/useLatencyDiagnostics.ts`

The singleton owns:

- `sessionId`, incrementing trial counter, armed/neutral/active state;
- selected stream ID;
- a maximum 20,000-record ring buffer and dropped-record count;
- correlation maps keyed by `seq:wireTimestamp`;
- recent selected-stream frame metadata, bounded by count/time;
- clock offset/RTT status;
- subscribe/getSnapshot API with a stable cached snapshot;
- methods `armTrial`, `observeControlInput`, `claimPendingCommandTrace`,
  `ingestGatewayTrace`, `ingestRoverTrace`, `observeVideoReceive`,
  `observeVideoRender`, `selectStream`, `resetSession`, and `exportJsonl`.

`observeControlInput` triggers T0 only after an armed trial observes neutral and
then crosses the configurable magnitude threshold. It records one pending
command tag, which `claimPendingCommandTrace` consumes exactly once.

### A3. Diagnostics UI and T0 integration

Files:

- `dashboard/src/components/LatencyDiagnosticsPanel.tsx` (new)
- `dashboard/src/components/ControlPanel.tsx`
- `dashboard/src/components/ControlPanel/ControlPanel.css`

Add a compact panel to the persistent control sidebar. It must provide arm,
reset, export, and stream selection controls. Refresh the visible snapshot at
4 Hz rather than on every record. Fetch `/video/streams` for stream choices.

Call `observeControlInput` from the gamepad animation-frame loop after the
scaled drive command is calculated. On the 50 ms command timer, claim and attach
the pending trace metadata to only one `sendCmdDrive` call.

Test: `cd dashboard && npm run check`.

### A4. Browser/base clock sampling

Add an eight-sample clock sync method to the diagnostics singleton. Resolve the
time URL from `VITE_XBEE_WS_URL` when explicit, otherwise use same-origin
`/latency/time`. Store offset, minimum/selected RTT, sync time, and error. Run on
panel mount and on a manual resync button. Use offset when comparing gateway
T1/T7a with browser T0/T7b/T7c; never alter raw timestamps.

Test with dashboard typecheck/build and gateway endpoint tests.

### B1. Dashboard/gateway trace types

Files:

- `dashboard/src/lib/xbeeGateway.ts`
- `base/gateway/src/app/create_gateway_app.js`
- relevant gateway app tests

Extend `sendCmdDrive` with an optional trace object. Parse gateway
`latency_trace` messages and expose `onLatencyTrace` cleanup subscriptions.

In `handleDashboardMessage`, capture T1 before encoding. Use one local
`wireTimestampMs`, encode once, read `frame[3]` as the sequence, write the
unchanged frame, and broadcast a trace only if a valid `trial_id` was supplied.
Include `client_tx_epoch_us`, `gateway_rx_epoch_us`, `xbee_seq`, and
`wire_timestamp_ms`.

Do not add trace fields to periodic untagged commands and do not change the
binary protocol tests' expected payload sizes.

### B2. Rover T3/T4 publisher

Files:

- `rover/ros2_ws/src/mr2_xbee_bridge/src/xbee_bridge_node.cpp`
- rover launch entry points that need to forward a diagnostics launch argument
- `rover/ros2_ws/src/mr2_xbee_bridge/README.md`

Add parameters:

```text
latency_diagnostics_enabled=false
latency_trace_topic=/latency/trace
motion_odom_topic=/wheel_encoder/odometry
motion_linear_threshold_m_s=0.03
motion_angular_threshold_rad_s=0.03
motion_consecutive_samples=3
```

When enabled, create a `std_msgs/String` publisher. Pass the decoded frame
sequence into `handle_cmd_drive_`. Capture system epoch T3 after decode and T4
immediately after publish. Emit compact valid JSON containing the correlation
key, target, smoothed output, and both timestamps. No console log is added.

Expose `enable_latency_diagnostics` through `rover.launch.py` and forward it
from the real/sim wrappers so field launch uses:

```bash
ros2 launch mr2_launch rover_real.launch.py enable_latency_diagnostics:=true
```

### B3. Rover trace dashboard subscription

The diagnostics panel/hook connects the existing ROS bridge singleton and
subscribes to `/latency/trace` as `std_msgs/msg/String`. Parse defensively and
pass valid events to `ingestRoverTrace`. Gateway mappings and rover events may
arrive in either order. Once matched, create source records for T1/T3/T4 with
the trial's T0-derived delta.

Tests:

```bash
cd rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to mr2_xbee_bridge mr2_launch

cd ../../../base/gateway
npm test

cd ../../dashboard
npm run check
```

### C1. Rover motion onset state machine

Inside the XBEE bridge diagnostics code, arm a pending motion onset only on a
raw drive-target transition from neutral to active. Store the origin
correlation key and T3. Subscribe to wheel odometry only when diagnostics are
enabled. Count consecutive samples above either threshold. Emit exactly one
`motion_onset` trace with T6 and measured velocities, then disarm. Reset target
edge state only after a raw command returns below threshold. Protect shared
reader-thread/executor state with a dedicated mutex.

The dashboard maps the T6 event by the stored correlation key, records T6, and
shows `T6-T3`. Missing odometry leaves T6 in a waiting state; it must not invent
a timestamp.

Test the state transition as far as practical through a small pure helper/unit
test if extracting it is reasonable. Otherwise build with warnings enabled and
perform a simulation/PTY plus synthetic odometry smoke test only when the ROS
environment supports it. Never use `can0`.

### D1. Video timing metadata propagation

Files:

- `dashboard/src/lib/videoProtocol.ts`
- `dashboard/src/lib/videoGateway.ts`
- `dashboard/src/components/VideoStreamCard.tsx`
- gateway video protocol/service tests if wire semantics documentation changes

Keep the binary header unchanged. Rename the TypeScript meaning to
`baseIngestTimestampUs` or provide a compatibility alias while documenting that
it is post-GStreamer T7a. At WebSocket callback entry, capture T7b using the
browser epoch helper and attach it to the decoded chunk before invoking stream
listeners.

Before calling `decoder.decode`, store metadata by the encoded timestamp. In
the decoder output callback, read `frame.timestamp`, draw, capture T7c, close
the frame, and call `observeVideoRender`. Bound and clean the metadata map on
decoder reset/unmount.

Only call the diagnostics store for the currently selected stream. This avoids
13 streams times 30 fps worth of measurement work.

### D2. Trial response-frame selection

Maintain recent rendered-frame observations for the selected stream. When both
T6 and frames exist, choose the earliest rendered frame with normalized
`T7a >= T6`. Support either arrival order by re-running selection after a T6 or
new frame arrives. Assign its T7a/T7b/T7c to the trial once and never replace
them. Record the approximation in event metadata.

Display:

- `T3-T0` computer-to-rover;
- `T6-T3` rover execution;
- `T7c-T6` coarse video-back;
- `T7b-T7a` base-to-browser;
- `T7c-T7b` decode/render.

### E1. Integration and documentation

Update dashboard, gateway, and XBEE bridge READMEs with:

- diagnostics launch flag;
- clock-sync prerequisite;
- arm/neutral/move workflow;
- selected-stream behavior;
- JSONL export;
- T7a and response-frame approximation limitations.

Run:

```bash
cd base/gateway
npm test

cd ../../dashboard
npm run check

cd ../rover/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to mr2_xbee_bridge mr2_launch
colcon test --packages-select mr2_xbee_bridge mr2_launch
colcon test-result --verbose
```

If the full ROS dependency graph is unavailable, record the exact failure and
still run syntax/compile checks that are possible. Do not claim hardware or RF
validation.

## 6. MVP completion criteria

The MVP is complete when:

- the dashboard can arm a neutral-to-drive trial and captures T0 once;
- a tagged command is correlated through unchanged XBEE framing to rover T3;
- wheel odometry onset produces one correlated T6;
- the selected video stream produces T7a/T7b/T7c;
- the dashboard shows the three coarse regions and post-base subregions;
- raw events export as bounded JSONL with session/trial IDs;
- missing clocks/topics/frames show waiting/error state rather than fabricated
  values;
- diagnostics-disabled rover behavior remains unchanged;
- gateway tests, dashboard checks, and feasible ROS builds/tests pass;
- hardware/RF validation is explicitly left for the field experiment.

## 7. Historical implementation result (superseded 2026-08-10)

All phases A through E are implemented. The final implementation keeps React
updates event-driven: selected-stream frames are retained in bounded internal
maps, but do not publish a UI snapshot per frame. A snapshot is published only
for operator/clock/correlation state changes or when the one response frame is
selected. This is stricter than the planned 4 Hz cap and avoids adding a
measurement-side rendering load.

Implemented boundaries:

- Phase A: `/latency/time`, same-origin nginx routing, clock sampling, trial
  state, persistent panel, and bounded JSONL export;
- Phase B: optional dashboard tags, unchanged 16-byte XBEE drive frames,
  gateway correlation traces, rover T3/T4 traces, and out-of-order joining;
- Phase C: diagnostics-only odometry subscription and a tested, one-shot,
  neutral-rearmed motion-onset detector for T6;
- Phase D: unchanged video wire framing with explicit T7a semantics, browser
  callback T7b, canvas-draw T7c, selected-stream filtering, and response-frame
  association;
- Phase E: dashboard/gateway/rover/video documentation, launch forwarding, and
  cross-subsystem regression checks.

Validation completed:

```text
base/gateway npm test                         55 passed
dashboard npm run test:latency               passed
dashboard npm run check                      typecheck, lint, build passed
mr2_xbee_bridge ROS 2 Humble Docker build    passed
mr2_xbee_bridge colcon test                  1 passed
standalone motion detector C++ test          passed with -Wall/-Wextra/-Wpedantic
launch-file Python compilation               passed
gateway PTY startup + /latency/time smoke    passed
git diff --check                             passed
```

The host currently has Node 18.19.1, so Vite prints its Node 20.19+/22.12+
engine warning even though the production build succeeds. The ROS Docker image
also needed `ros-humble-control-msgs` installed inside the disposable container.
An earlier build including all `mr2_launch` dependencies stopped in the
unrelated `ntrip_client_node`; the targeted `mr2_xbee_bridge` Humble build and
test subsequently passed. No rover hardware, RF link, camera, or physical
motion validation was performed.
