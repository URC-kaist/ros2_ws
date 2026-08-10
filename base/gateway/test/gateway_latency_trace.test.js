'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { encodeDashboardDriveMessage } = require('../src/app/create_gateway_app')
const { consumeFrames, createSequencer, MsgId } = require('../src/protocol/xbee')

test('tagged drive command preserves wire layout and returns a correlation trace', () => {
  const nextSeq = createSequencer(41)
  const gatewayRxEpochUs = 1786351234567000
  const encoded = encodeDashboardDriveMessage(
    {
      trial_id: 'trial-000001',
      client_tx_epoch_us: gatewayRxEpochUs - 5000,
      linear_x_m_s: 0.4,
      linear_y_m_s: 0,
      angular_z_rad_s: 0.1,
    },
    nextSeq,
    gatewayRxEpochUs
  )

  let decoded = null
  const remainder = consumeFrames(encoded.frame, (msgId, payload) => {
    decoded = { msgId, payload }
  })

  assert.equal(remainder.length, 0)
  assert.equal(decoded.msgId, MsgId.CMD_DRIVE)
  assert.equal(decoded.payload.length, 16)
  assert.equal(encoded.frame[3], 41)
  assert.deepEqual(encoded.trace, {
    type: 'latency_trace',
    trial_id: 'trial-000001',
    client_tx_epoch_us: gatewayRxEpochUs - 5000,
    gateway_rx_epoch_us: gatewayRxEpochUs,
    xbee_seq: 41,
    wire_timestamp_ms: Math.floor(gatewayRxEpochUs / 1000) >>> 0,
  })
})

test('untagged drive command does not create a trace', () => {
  const encoded = encodeDashboardDriveMessage(
    {
      linear_x_m_s: 0.2,
      linear_y_m_s: 0,
      angular_z_rad_s: 0,
    },
    createSequencer(),
    1000000
  )

  assert.equal(encoded.trace, null)
})
