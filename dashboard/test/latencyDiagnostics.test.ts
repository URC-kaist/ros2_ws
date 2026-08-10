import assert from 'node:assert/strict'

import { LatencyDiagnostics } from '../src/lib/latencyDiagnostics'

const diagnostics = new LatencyDiagnostics()
diagnostics.selectStream('front_test_cam')
diagnostics.armTrial()
diagnostics.observeControlInput({ x: 0, y: 0, yaw: 0 })
assert.equal(diagnostics.getSnapshot().neutralSeen, true)
diagnostics.observeControlInput({ x: 0.4, y: 0, yaw: 0 })

const tag = diagnostics.claimPendingCommandTrace()
assert.ok(tag)
assert.equal(diagnostics.claimPendingCommandTrace(), undefined)
const t0 = tag.client_tx_epoch_us

// Rover traces may arrive before the gateway correlation mapping.
diagnostics.ingestRoverTrace({
  event: 'command',
  xbee_seq: 7,
  wire_timestamp_ms: 1234,
  rover_rx_epoch_us: t0 + 100_000,
  cmd_publish_epoch_us: t0 + 110_000,
})
diagnostics.ingestGatewayTrace({
  type: 'latency_trace',
  trial_id: tag.trial_id,
  client_tx_epoch_us: t0,
  gateway_rx_epoch_us: t0 + 10_000,
  xbee_seq: 7,
  wire_timestamp_ms: 1234,
})

diagnostics.observeVideoReceive('front_test_cam', t0 + 300_000, t0 + 320_000)
diagnostics.observeVideoRender('front_test_cam', t0 + 300_000, t0 + 340_000)

const snapshot = diagnostics.getSnapshot()
assert.equal(snapshot.currentTrial?.stages.gateway_command_received_t1, t0 + 10_000)
assert.equal(snapshot.currentTrial?.stages.rover_command_received_t3, t0 + 100_000)
assert.equal(snapshot.currentTrial?.stages.rover_cmd_vel_published_t4, t0 + 110_000)
assert.equal(snapshot.currentTrial?.metrics.computerToGatewayMs, 10)
assert.equal(snapshot.currentTrial?.metrics.computerToRoverMs, 100)
assert.equal(snapshot.currentTrial?.metrics.roverPublishMs, 10)
assert.equal(snapshot.videoTiming.sampleCount, 1)
assert.equal(snapshot.videoTiming.baseToBrowser.count, 0)
assert.equal(snapshot.videoTiming.decodeRender.p50Ms, 20)
assert.equal(snapshot.videoTiming.decodeRender.p95Ms, 20)

console.log('latency diagnostics state-machine test passed')
