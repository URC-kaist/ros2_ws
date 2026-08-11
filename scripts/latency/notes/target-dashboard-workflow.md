# Target dashboard latency workflow (working note)

Status: product-intent note only. This describes the agreed target behavior,
not the current implementation. It is deliberately kept outside the primary
README files so implementation details can evolve without presenting this as
an existing field procedure.

## Goal

An operator can start either direction of latency measurement from the browser
and receive segmented and end-to-end results without running capture or
analysis commands in a terminal or importing an RTP report manually.

The latency panel has three primary actions:

1. **Measure downlink latency** (`user -> rover`)
2. **Measure uplink latency** (`rover -> user`)
3. **Check chrony sync**

Downlink and uplink measurements are independent. Either can be run without
running the other.

## Check chrony sync

The operator presses **Check chrony sync** to verify that base and rover clock
status is fresh and within the accepted synchronization error bounds. The UI
shows ready/not-ready status and useful details such as clock offset, error
bound, and last update time.

Measurements that compare timestamps from different hosts must be blocked when
clock synchronization is not ready. This action only checks status; chronyd
continues to synchronize clocks independently of the browser.

## Downlink workflow (`user -> rover`)

1. The operator presses **Measure downlink latency**.
2. The UI asks the operator to leave the controller neutral while the trial is
   prepared.
3. When the UI indicates that it is armed, the operator makes one deliberate
   controller input.
4. The system automatically correlates timestamps along the command path.
5. The UI displays each measured segment and the total latency.

The intended boundary starts when the browser observes the controller input and
ends when the rover software hands the corresponding command to the actuator
command path. Expected segments include:

- browser input -> base receipt;
- base processing/transmission -> rover receipt;
- rover receipt -> actuator-command handoff;
- browser input -> actuator-command handoff total.

The result UI must show every segment above alongside the total. A total-only
downlink result does not satisfy the target workflow.

This is command-delivery latency, not verified physical actuator-motion
latency. Measuring actual motion would require actuator feedback or a separate
sensor.

## Uplink workflow (`rover -> user`)

1. The operator selects the number of simultaneous video feeds to include in
   the trial.
2. The operator presses **Measure uplink latency**.
3. The system automatically starts the required rover RTP capture, base RTP
   capture, and browser receive/render sampling.
4. It gathers a fixed measurement window, stops all participants, correlates
   matching RTP packets, and analyzes the samples.
5. The UI displays each measured segment, the total latency, distribution
   statistics, and packet-delivery information.

The intended boundary starts when an RTP packet is sent from the rover toward
the Rocket M2 link and ends when the corresponding video data is rendered in
the browser. Expected segments include:

- rover RTP send -> base RTP receive (Rocket M2 path);
- base RTP receive/ingest -> browser receive;
- browser receive -> decode/render completion;
- rover RTP send -> browser render total.

The result UI must show every segment above alongside the total, both for each
measured stream and for the trial summary. A total-only uplink result does not
satisfy the target workflow.

Results should include at least p50, p95, p99, maximum latency, packet loss,
and the selected video-feed count. Repeating trials with different feed counts
must make it possible to compare the effect of concurrent streams.

This is not camera-exposure-to-display (glass-to-glass) latency. Camera capture
and encoding time before the rover RTP-send boundary are outside the agreed
measurement.

## UI constraints

- The three actions above are the only primary latency controls.
- Manual **Arm command**, **Measure browser/base**, and **Import RTP report**
  steps are replaced by the two automated directional workflows.
- Progress, readiness, failure, and completion must be visible during a trial.
- A failed participant or invalid clock state must produce an explicit failed
  result rather than a partial value presented as end-to-end latency.
- Export and reset may remain as secondary result actions, but they are not
  primary measurement buttons.
- Results must identify their measurement boundaries so command delivery is
  not mistaken for physical motion and RTP-send latency is not mistaken for
  glass-to-glass latency.
- Neither directional result may hide its segment breakdown behind only one
  end-to-end latency value.
