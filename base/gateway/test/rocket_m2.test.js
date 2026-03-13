'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  createRocketM2Status,
  formatRocketM2Error,
  parseRocketM2Signal,
  redactRocketM2Secrets,
} = require('../src/rocket_m2')

test('parseRocketM2Signal coerces numeric fields and arrays', () => {
  const status = parseRocketM2Signal(
    JSON.stringify({
      signal: '-62',
      rssi: -61,
      noisef: '-95',
      chwidth: '20',
      rx_chainmask: '3',
      chainrssi: ['-61', '-63'],
      chainrssimgmt: ['-60'],
      chainrssiext: [null, '-64'],
    })
  )

  assert.deepEqual(status, {
    signal: -62,
    rssi: -61,
    noisef: -95,
    chwidth: 20,
    rx_chainmask: 3,
    chainrssi: [-61, -63],
    chainrssimgmt: [-60],
    chainrssiext: [-64],
  })
})

test('formatRocketM2Error redacts password-like content', () => {
  const message = formatRocketM2Error({
    code: 28,
    message: 'curl failed password=secret123&foo=1',
  })

  assert.match(message, /code=28/)
  assert.doesNotMatch(message, /secret123/)
  assert.match(message, /password=\*\*\*/)
})

test('createRocketM2Status uses explicit timing defaults', () => {
  const status = createRocketM2Status(
    { connected: true, signal: -60 },
    { nowMs: 1000, lastSuccessMs: 900 }
  )

  assert.deepEqual(status, {
    connected: true,
    updated_at_ms: 1000,
    last_success_ms: 900,
    signal: -60,
    rssi: null,
    noisef: null,
    chwidth: null,
    rx_chainmask: null,
    chainrssi: [],
    chainrssimgmt: [],
    chainrssiext: [],
    error: null,
  })
})

test('redactRocketM2Secrets leaves unrelated text unchanged', () => {
  assert.equal(redactRocketM2Secrets('plain text'), 'plain text')
})
