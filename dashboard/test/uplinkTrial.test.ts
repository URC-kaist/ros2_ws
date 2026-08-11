import assert from 'node:assert/strict'

import { LatencyDiagnostics } from '../src/lib/latencyDiagnostics'
import { mapGatewayUplinkPhase, uplinkPhaseLabel } from '../src/lib/latency/uplinkTrial'

assert.equal(mapGatewayUplinkPhase('created'), 'preparing')
assert.equal(mapGatewayUplinkPhase('waiting_for_rover_artifacts'), 'capturing')
assert.equal(mapGatewayUplinkPhase('stopping_base_capture'), 'uploading')
assert.equal(mapGatewayUplinkPhase('analyzing'), 'analyzing')
assert.equal(uplinkPhaseLabel('capturing'), 'Capturing RTP')

const diagnostics = new LatencyDiagnostics()
assert.equal(diagnostics.startUplinkPreflight(2, ['front', 'rear']), true)
assert.equal(diagnostics.startUplinkPreflight(1, ['front']), false)
diagnostics.bindUplinkTrial('trial-1')
diagnostics.updateUplinkPhase('capturing', 0.4)
let snapshot = diagnostics.getSnapshot()
assert.equal(snapshot.uplink.active, true)
assert.equal(snapshot.uplink.trialId, 'trial-1')
assert.equal(snapshot.uplink.feedCount, 2)
assert.deepEqual(snapshot.uplink.streamIds, ['front', 'rear'])
assert.equal(snapshot.uplink.progress, 0.4)
diagnostics.cancelUplinkMeasurement()
snapshot = diagnostics.getSnapshot()
assert.equal(snapshot.uplink.phase, 'cancelled')
assert.equal(snapshot.uplink.active, false)

console.log('uplink trial state test passed')
