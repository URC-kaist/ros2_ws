import assert from 'node:assert/strict'

import { calculateDownlinkMetrics } from '../src/lib/latency/downlinkTrial'

const stages = {
  input_t0: 1_000_000,
  gateway_command_received_t1: 1_012_000,
  rover_command_received_t3: 1_100_000,
  rover_cmd_vel_published_t4: 1_115_000,
}

assert.deepEqual(calculateDownlinkMetrics(stages, 2_000), {
  browserToGatewayMs: 10,
  gatewayToRoverMs: 88,
  roverReceiveToPublishMs: 15,
  totalMs: 113,
  browserClockDriftUs: null,
  valid: true,
  warning: null,
})

const negative = calculateDownlinkMetrics(stages, 20_000)
assert.equal(negative.valid, false)
assert.ok(negative.warning?.includes('negative'))

const drifted = calculateDownlinkMetrics(stages, 2_000, 2_001)
assert.equal(drifted.valid, false)
assert.ok(drifted.warning?.includes('drift'))

console.log('downlink segment calculation test passed')
