# Base-local chrony implementation plan

## Goal

Use the base-station system clock as the local time reference and continuously
synchronize the rover system clock to it with chrony. Keep nginx as the HTTP and
WebSocket reverse proxy. Replace the temporary browser/ROSBridge NTP-style
base-rover offset estimator with chrony configuration and observable chrony
health.

The RTP analyzer contract remains:

```text
clock_offset_us = base_clock - rover_clock
link_latency_us = base_capture_us - rover_capture_us - clock_offset_us
```

When chrony reports the rover locked to the base within the experiment's error
budget, the analyzer should normally receive `--clock-offset-us 0`. The chrony
residual offset/root dispersion are recorded as measurement uncertainty, not
silently converted into a packet-latency correction.

## Large plan

### Phase 0: remove the temporary network-probe offset estimator

Remove only the recently added base-rover probe feature:

- rover `/latency/clock_probe/request` and `/response` interfaces;
- dashboard 10-sample probe state machine and offset-composition helper;
- `Measure base/rover offset`, uncertainty, and copy-option UI;
- command T3/T4 correction based on that estimated offset;
- probe-specific tests and documentation.

Preserve the earlier latency MVP: browser/base `/latency/time`, T0/T1/T3/T4,
matched RTP pcap analysis, report import, and browser render timing. Run the
dashboard tests/check and build `mr2_xbee_bridge` before Phase 1.

### Phase 1: deployable chrony configuration

Add a non-interactive configuration generator under `scripts/latency/chrony/`.
It accepts validated base/rover IPv4 addresses and renders complete role-specific
chrony configuration:

- base: no external time dependency, `local stratum 8`, and NTP access limited
  to the rover address;
- rover: the base address is the only server, with `iburst`, a short polling
  interval suitable for a mobile experiment, startup stepping, and normal
  continuous slewing afterward.

The tool defaults to printing/writing a candidate. Host installation requires
an explicit `--install`, root, a successful `chronyd -p` validation, an atomic
write, and a timestamped backup of an existing configuration. Restart only the
chrony service; never modify nginx. Unit-test rendering, address validation,
role requirements, and overwrite safeguards. Do not run `--install` as part of
repository validation.

### Phase 2: chrony status producers

Base gateway:

- add a bounded, shell-free `chronyc tracking` adapter with a short timeout;
- parse stable fields into JSON without exposing arbitrary command execution;
- serve read-only `GET /latency/clock-status` through the existing gateway and
  nginx location;
- return explicit unavailable/unsynchronized states when chrony is absent.

Rover:

- extend `mr2_system_status` with `/system_status/clock` diagnostics;
- run only the allow-listed `chronyc tracking` command with a timeout at a low
  rate, parse the same core fields, and publish an ERROR/WARN/OK diagnostic;
- do not call `makestep` or modify the clock from a ROS callback.

Tests use captured command output and injected runners. No test contacts a real
chronyd daemon.

### Phase 3: dashboard status and experiment gate

Replace the removed estimator UI with `Check chrony sync`:

- fetch base status from `/latency/clock-status`;
- consume the latest rover `/system_status/clock` diagnostic;
- show role, synchronization state, reference, stratum, system residual offset,
  root dispersion, last update age, and errors;
- mark RTP capture `Ready` only when both statuses are fresh, the rover is
  synchronized, and the rover uncertainty is within a documented threshold;
- present `--clock-offset-us 0` for copying only in the ready state.

The button is read-only. chronyd runs continuously; the UI verifies its state
instead of starting services or stepping clocks during an experiment.

### Phase 4: documentation and integrated validation

Document package installation, candidate generation, explicit installation,
service verification, UDP 123 requirements, rollback, dashboard workflow, and
the distinction between synchronization error and Rocket packet latency.

Run:

- chrony configuration/parser unit tests;
- gateway full test suite;
- dashboard latency tests and `npm run check`;
- `mr2_system_status` Python tests/lint where available;
- ROS Humble package builds for `mr2_system_status`, `mr2_xbee_bridge`, and
  affected launch packages;
- `git diff --check`.

Real chrony lock, firewall behavior, Rocket M2 transport, and hardware packet
captures remain field validations and must be reported as unverified locally.

## Small implementation plan

### 0.1 Remove rover clock probe

In `mr2_xbee_bridge/src/xbee_bridge_node.cpp`, remove the two clock-probe topic
parameters, publisher/subscriber, and Float64MultiArray callback. Keep
Float64MultiArray itself where it is already used for the gripper command. Keep
the command trace `system_epoch_us_()` helper because T3/T4 still require it.

### 0.2 Remove dashboard estimator

Delete `dashboard/src/lib/clockOffset.ts` and its test/import. In
`latencyDiagnostics.ts`, remove pending probe promises, rover clock snapshot
fields, `measureBaseRoverClock`, response ingestion, and probe export metadata.
Restore rover stage normalization to the pre-probe assumption and leave the
browser/base clock sampler untouched. In the panel, remove clock-probe ROS
topics, copy state, offset UI, and rover-specific T0 correction.

### 0.3 Prove clean removal

Search for `clock_probe`, `baseMinusRover`, and `Measure base/rover`. Only
historical plan text, if deliberately retained, may remain. Run dashboard
latency tests/check and the rover bridge package build.

### 1.1 Chrony configuration model

Create pure Python functions `render_base_config(base_ip, rover_ip)` and
`render_rover_config(base_ip)` plus an argparse CLI. Use `ipaddress.IPv4Address`;
reject unspecified, multicast, loopback, and invalid role combinations. Include
a generated-file header and explicit role/address comments.

### 1.2 Safe installation path

Support `--output PATH` with no overwrite unless `--force`, and `--install` as a
separate mutually exclusive mode. For installation, validate the temporary
candidate with `chronyd -p -f`, preserve owner/mode, back up the target, replace
atomically, then restart `chrony.service`. Print the backup path and rollback
command. Never install packages or open a firewall automatically.

### 2.1 Shared status schema

Use schema version 1 with:

```text
available, synchronized, reference_id, reference_name, stratum,
system_time_offset_s, last_offset_s, rms_offset_s, root_delay_s,
root_dispersion_s, update_interval_s, leap_status, sampled_at_epoch_ms, error
```

`system_time_offset_s` uses chrony's sign: positive when the local system clock
is fast relative to its reference and negative when slow. Preserve raw text only
in local logs/tests, not in browser responses.

### 2.2 Gateway adapter and endpoint

Use `execFile`/`spawn` argument arrays, never a shell. Cap stdout/stderr, kill on
timeout, and map missing executable/nonzero exit/parser failure to structured
status. Add unit tests for fast/slow signs, synchronized/unsynchronized leap
states, timeout/error, and HTTP response/cache headers.

### 2.3 Rover status publisher

Put parsing in a pure Python module and command execution behind an injectable
function. Publish KeyValue entries on `/system_status/clock`; use WARN when the
command is unavailable and ERROR when available but not synchronized. Poll no
faster than once every two seconds to avoid unnecessary processes.

### 3.1 Dashboard parser/store

Add strict parsers for the gateway JSON and rover DiagnosticArray values. Store
arrival timestamps and compute freshness independently of ROS header time so an
unsynchronized clock cannot make its own status appear fresh.

### 3.2 UI readiness

`Check chrony sync` refreshes the base fetch and evaluates the latest rover
sample. Default readiness requirements: both samples no older than 5 seconds,
rover synchronized, `abs(system_time_offset_s) <= 2 ms`, and
`root_dispersion_s <= 2 ms`. Display the actual values and failed conditions.
Keep thresholds constants in the pure dashboard library with tests.

### 3.3 Logging/report workflow

Include both chrony status snapshots in latency JSONL export. Do not rewrite an
already imported RTP report. The UI tells the operator to generate a new report
with `--clock-offset-us 0` only after the readiness gate passes.

## Completion criteria

- No active browser/ROSBridge NTP-style base-rover offset probe remains.
- Base and rover configurations can be generated deterministically without
  touching nginx.
- Installation is explicit, validated, backed up, and reversible.
- Dashboard shows independently sourced base and rover chrony health.
- A zero analyzer offset is offered only when the chrony readiness gate passes.
- All non-hardware tests/builds pass, and hardware/clock-lock limitations are
  explicitly documented.

## Completion report

Completed on 2026-08-10.

- Phase 0: removed all active `/latency/clock_probe/*` rover/dashboard code,
  estimator state, UI, helper, and tests. Preserved browser/base time sampling,
  T0/T1/T3/T4, and RTP pcap analysis. Dashboard checks and the ROS Humble
  `mr2_xbee_bridge` dependency build passed after removal.
- Phase 1: added the base-local/rover-client configuration generator, explicit
  validated installation, metadata-preserving backup, automatic restart
  rollback, documentation, and six unit tests. Both rendered roles passed an
  actual `chronyd -p` parse in a disposable container.
- Phase 2: added the fixed-command base chronyc adapter and
  `/latency/clock-status`, nginx routing, rover `/system_status/clock`, four JS
  adapter tests, one HTTP test, and four rover parser tests.
- Phase 3: added dashboard parsing, freshness/2 ms readiness gates, read-only
  `Check chrony sync`, zero-offset copy only when ready, and chrony snapshots in
  JSONL export. The dashboard latency suite, strict typecheck, ESLint, and Vite
  production build passed.
- Phase 4: gateway finished with 60/60 tests. `mr2_system_status` built under
  ROS 2 Humble and its four tests passed. Chrony configuration plus RTP tools
  finished with 14 Python tests. `git diff --check` passed.

Not performed: `--install` on either real host, firewall changes, real
base/rover chrony lock over Rocket M2, hardware packet capture, or physical
latency validation. The local dashboard build used Node 18.19.1 and completed,
but Vite reported its existing Node 20.19+/22.12+ requirement warning.
