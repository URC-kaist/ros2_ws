import assert from 'node:assert/strict'
import process from 'node:process'

import { LatencyDiagnostics } from '../src/lib/latencyDiagnostics'

const originalFetch = globalThis.fetch
const originalWindow = globalThis.window
let baseClockDisplacementUs = 0
Object.defineProperty(globalThis, 'window', {
  configurable: true,
  value: { location: { origin: 'http://localhost:5173' } },
})
globalThis.fetch = async () => {
  const serverEpochUs =
    Math.round((performance.timeOrigin + performance.now()) * 1000) +
    baseClockDisplacementUs
  return new Response(
    JSON.stringify({
      server_receive_epoch_us: serverEpochUs,
      server_send_epoch_us: serverEpochUs,
    }),
    { status: 200, headers: { 'content-type': 'application/json' } }
  )
}

async function runLatencyDiagnosticsTest() {
  try {
    const diagnostics = new LatencyDiagnostics()
    diagnostics.selectStream('front_test_cam')
    assert.equal(diagnostics.startDownlinkPreflight(), true)
    assert.equal(diagnostics.getSnapshot().downlink.phase, 'checking_clocks')
    assert.equal(await diagnostics.synchronizeGatewayClock(2), true)
    assert.equal(diagnostics.armDownlinkTrial(), true)
    assert.equal(diagnostics.getSnapshot().downlink.phase, 'waiting_for_neutral')
    diagnostics.observeControlInput({ x: 0, y: 0, yaw: 0 })
    assert.equal(diagnostics.getSnapshot().downlink.neutralSeen, true)
    assert.equal(diagnostics.getSnapshot().downlink.phase, 'waiting_for_input')
    diagnostics.observeControlInput({ x: 0.4, y: 0, yaw: 0 })
    assert.equal(diagnostics.getSnapshot().downlink.phase, 'waiting_for_trace')

    const tag = diagnostics.claimPendingCommandTrace()
    assert.ok(tag)
    assert.equal(diagnostics.claimPendingCommandTrace(), undefined)
    const t0 = tag.client_tx_epoch_us
    const browserToBaseOffsetUs = diagnostics.getSnapshot().clock.offsetUs
    assert.notEqual(browserToBaseOffsetUs, null)
    const t0Base = t0 + (browserToBaseOffsetUs ?? 0)

    // Rover traces may arrive before the gateway correlation mapping.
    diagnostics.ingestRoverTrace({
      event: 'command',
      xbee_seq: 7,
      wire_timestamp_ms: 1234,
      rover_rx_epoch_us: t0Base + 100_000,
      cmd_publish_epoch_us: t0Base + 110_000,
    })
    diagnostics.ingestGatewayTrace({
      type: 'latency_trace',
      trial_id: tag.trial_id,
      client_tx_epoch_us: t0,
      gateway_rx_epoch_us: t0Base + 10_000,
      xbee_seq: 7,
      wire_timestamp_ms: 1234,
    })
    assert.equal(diagnostics.getSnapshot().downlink.phase, 'verifying_clocks')
    assert.equal(await diagnostics.synchronizeGatewayClock(2), true)
    assert.equal(diagnostics.completeDownlinkClockVerification(), true)

    diagnostics.observeVideoReceive(
      'front_test_cam',
      t0 + 300_000,
      t0 + 300_000,
      t0 + 320_000,
      null
    )
    diagnostics.observeVideoRender('front_test_cam', t0 + 300_000, t0 + 340_000)

    const snapshot = diagnostics.getSnapshot()
    assert.equal(snapshot.downlink.phase, 'completed')
    assert.equal(snapshot.currentTrial?.stages.gateway_command_received_t1, t0Base + 10_000)
    assert.equal(snapshot.currentTrial?.stages.rover_command_received_t3, t0Base + 100_000)
    assert.equal(snapshot.currentTrial?.stages.rover_cmd_vel_published_t4, t0Base + 110_000)
    assert.equal(snapshot.currentTrial?.metrics.browserToGatewayMs, 10)
    assert.equal(snapshot.currentTrial?.metrics.gatewayToRoverMs, 90)
    assert.equal(snapshot.currentTrial?.metrics.roverReceiveToPublishMs, 10)
    assert.equal(snapshot.currentTrial?.metrics.totalMs, 110)
    assert.equal(snapshot.currentTrial?.metrics.valid, true)
    assert.deepEqual(
      snapshot.currentTrial?.segments.map((segment) => segment.distribution.p50Ms),
      [10, 90, 10, 110]
    )
    assert.equal(snapshot.videoTiming.sampleCount, 1)
    assert.equal(snapshot.videoTiming.baseToBrowser.count, 1)
    assert.ok(snapshot.videoTiming.baseToBrowser.p50Ms != null)
    assert.equal(snapshot.videoTiming.decodeRender.p50Ms, 20)
    assert.equal(snapshot.videoTiming.decodeRender.p95Ms, 20)

    // A wall-clock correction after the tab opened can put the browser's
    // monotonic epoch more than five seconds away from Base. A fresh snapshot
    // must still be stamped in the Base clock domain.
    baseClockDisplacementUs = 30_000_000
    assert.equal(await diagnostics.synchronizeGatewayClock(2), true)
    const displacedSnapshotEpochUs = diagnostics.getSnapshot().clock.syncedAtEpochUs
    assert.ok(displacedSnapshotEpochUs != null)
    assert.ok(
      Math.abs(
        displacedSnapshotEpochUs -
        (Math.round((performance.timeOrigin + performance.now()) * 1000) +
          baseClockDisplacementUs)
      ) < 1_000_000
    )
    baseClockDisplacementUs = 0

    assert.equal(diagnostics.startUplinkPreflight(1, ['front_test_cam']), true)
    diagnostics.bindUplinkTrial('uplink-1')
    assert.equal(
      diagnostics.beginUplinkBrowserCapture('uplink-1', ['front_test_cam']),
      true
    )
    diagnostics.observeVideoReceive(
      'front_test_cam',
      777,
      t0 + 500_000,
      t0 + 520_000,
      { ssrc: 42, rtpTimestamp: 90_000, markerSequence: 7 }
    )
    diagnostics.observeVideoRender('front_test_cam', 777, t0 + 525_000)
    assert.deepEqual(diagnostics.finishUplinkBrowserCapture('uplink-1'), [
      {
        stream_id: 'front_test_cam',
        ssrc: 42,
        rtp_timestamp: 90_000,
        marker_sequence: 7,
        browser_receive_epoch_us: t0 + 520_000,
        browser_render_epoch_us: t0 + 525_000,
      },
    ])

    console.log('latency diagnostics state-machine test passed')
  } finally {
    globalThis.fetch = originalFetch
    Object.defineProperty(globalThis, 'window', {
      configurable: true,
      value: originalWindow,
    })
  }
}

void runLatencyDiagnosticsTest().catch((error: unknown) => {
  process.nextTick(() => {
    throw error
  })
})
