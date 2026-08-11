# RTP link latency experiment

These tools measure the rover-to-base Rocket M2 path by matching the same RTP
packets in two classic pcap files. They do not modify RTP, H.264, UDP, or the
browser video protocol, and they do not use visual motion detection.

## Measurement boundary

Capture on the rover interface that sends traffic toward its Rocket M2 and the
base interface directly connected to the base Rocket M2. The result includes a
small amount of host NIC/kernel time around the radio link.

The match key is the existing RTP tuple:

```text
(destination UDP port, SSRC, sequence number, RTP timestamp)
```

The central `video_streams.json` supplies the UDP-port-to-stream-ID mapping.

## Prerequisites

1. Configure the base as the local chrony reference and the rover as its client
   using `scripts/latency/chrony/README.md`.
2. In the dashboard, press `Check chrony sync` and require `Ready for
   synchronized RTP capture` before capturing.
3. Find the concrete Rocket-facing interface with `ip -brief link` or
   `ip route get <peer-ip>`. Avoid `--interface any`.
4. Install `tcpdump`. Capturing normally requires root or appropriate
   `CAP_NET_RAW`/`CAP_NET_ADMIN` capabilities.

For automated browser-triggered capture, configure the capability once on both
base and rover:

```bash
scripts/latency/install_capture_permissions.bash --check
scripts/latency/install_capture_permissions.bash
```

Set these values in the base environment before starting the gateway:

```bash
MR2_LATENCY_DIAGNOSTICS_ENABLE=true
MR2_BASE_ROCKET_INTERFACE=<base-rocket-interface>
MR2_LATENCY_PUBLIC_BASE_URL=http://192.168.1.101
MR2_LATENCY_ARTIFACT_DIR=/tmp/mr2-latency-base
```

Set the rover interface before launching with diagnostics enabled:

```bash
export MR2_ROVER_ROCKET_INTERFACE=<rover-rocket-interface>
ros2 launch mr2_launch rover_real.launch.py \
  enable_latency_diagnostics:=true enable_video_streaming:=true
```

The automated Uplink workflow performs stream leasing, rover/base capture,
artifact upload, RTP marker-to-browser-frame matching, cancellation, and timeout
cleanup. It reports Rocket M2, base-to-browser, decode/render, and end-to-end
distributions from the same rendered frame samples.

The analyzer defines the optional correction as:

```text
clock_offset_us = base_clock - rover_clock
link_latency_us = base_capture_us - rover_capture_us - clock_offset_us
```

If both systems are verified synchronized within the accuracy needed for the
experiment, pass `--clock-offset-us 0`. Omitting the option also uses zero but
marks it as an assumption in the report.

The dashboard readiness gate requires fresh base/rover status, a rover residual
offset no larger than 2 ms, and chrony's rover error bound no larger than 2 ms.
It is read-only: chronyd performs continuous synchronization independently of
the browser.

## Capture

Start the base capture first, then the rover capture. Stop the rover capture
first, then the base capture, so capture-window edges are not counted as radio
loss. The examples make the base duration longer for this reason. Use the same
stream and a duration long enough to include steady-state video. These commands
capture only the first 192 bytes of RTP packets on the selected UDP port.

Base:

```bash
cd /path/to/mr2-stack
sudo python3 scripts/latency/rtp_capture.py \
  --role base \
  --interface <base-rocket-interface> \
  --stream-id front_nav_cam \
  --duration-s 70 \
  --output /tmp/mr2-base-front.pcap
```

Rover:

```bash
cd /path/to/mr2-stack
sudo python3 scripts/latency/rtp_capture.py \
  --role rover \
  --interface <rover-rocket-interface> \
  --stream-id front_nav_cam \
  --duration-s 60 \
  --output /tmp/mr2-rover-front.pcap
```

Each command writes a sibling `.metadata.json` file with the role, host,
interface, selected ports, capture interval, tcpdump exit status, and tcpdump's
packet/kernel-drop summary. A nonzero kernel-drop count invalidates loss-rate
interpretation and should trigger a lower capture load or larger host buffer.
Existing pcap or sidecar files are not overwritten unless `--overwrite` is
supplied.

Copy both pcaps to one analysis host after capture. Do not run the analyzer on
live files that tcpdump is still writing.

## Analyze

```bash
python3 scripts/latency/analyze_rtp_latency.py \
  --rover /tmp/mr2-rover-front.pcap \
  --base /tmp/mr2-base-front.pcap \
  --stream-id front_nav_cam \
  --clock-offset-us 0 \
  --output /tmp/mr2-front-rtp-report.json \
  --samples-csv /tmp/mr2-front-rtp-samples.csv
```

The JSON contains per-stream packet counts, loss, offered/delivered bitrate,
packet-delay variation, and link latency `min/mean/p50/p95/p99/max`. The CSV
contains each matched packet and its corrected latency. Negative latency
samples are retained and warned about because they normally indicate an
incorrect clock offset.

The parser supports classic pcap with Ethernet, VLAN, Linux SLL/SLL2, IPv4,
UDP, and RTP v2. It intentionally rejects pcapng. The supplied capture wrapper
uses tcpdump's classic pcap output.

## Dashboard

Select `Uplink feeds` and press `Measure uplink latency` to run the automated
capture workflow. The selected count enables exactly the first N streams in
central display order for the duration of the trial and restores the previous
stream state afterward. The panel renders every selected stream, waits for all
of them to produce a frame, captures browser receive/draw timestamps, and shows
aggregate and per-stream segment tables when analysis completes.

The automated report uses these measurement boundaries:

```text
Rocket M2:       rover marker packet capture -> base marker packet capture
Base -> browser: base marker packet capture -> browser WebSocket callback
Decode/render:   browser callback -> canvas draw completion
Total:           rover marker packet capture -> canvas draw completion
```

## Diagnosing a bandwidth limit

Repeat the capture at increasing aggregate encoder bitrates while keeping the
scene, interfaces and clocks unchanged. A Rocket-path bandwidth bottleneck is
supported when increasing offered bitrate produces rising p50/p95 latency,
packet loss or packet-delay variation, and delivered bitrate stops following
offered bitrate. Repeat with a direct wired path when possible for an A/B
control.

Signal/RSSI alone is not a throughput or queue-delay measurement. Always retain
the pcap, JSON report, selected video config, clock-sync evidence, and Rocket
status for each field run.
