'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  createChronyStatusProvider,
  parseChronycTracking,
} = require('../src/runtime/chrony_status')

const TRACKING = `Reference ID    : 7F7F0101 (LOCAL)
Stratum         : 8
Ref time (UTC)  : Mon Aug 10 01:02:03 2026
System time     : 0.000125000 seconds slow of NTP time
Last offset     : -0.000010000 seconds
RMS offset      : 0.000020000 seconds
Frequency       : 1.000 ppm fast
Residual freq   : 0.000 ppm
Skew            : 0.010 ppm
Root delay      : 0.000100000 seconds
Root dispersion : 0.000200000 seconds
Update interval : 4.0 seconds
Leap status     : Normal
`

test('parseChronycTracking maps synchronized base status and slow sign', () => {
  const status = parseChronycTracking(TRACKING, 1234)
  assert.equal(status.available, true)
  assert.equal(status.synchronized, true)
  assert.equal(status.reference_id, '7F7F0101')
  assert.equal(status.reference_name, 'LOCAL')
  assert.equal(status.stratum, 8)
  assert.equal(status.system_time_offset_s, -0.000125)
  assert.equal(status.root_dispersion_s, 0.0002)
  assert.equal(status.sampled_at_epoch_ms, 1234)
})

test('parseChronycTracking accepts an empty reference name', () => {
  const status = parseChronycTracking(TRACKING.replace('(LOCAL)', '()'), 1234)

  assert.equal(status.available, true)
  assert.equal(status.synchronized, true)
  assert.equal(status.reference_id, '7F7F0101')
  assert.equal(status.reference_name, null)
})

test('parseChronycTracking maps fast sign and unsynchronized leap state', () => {
  const status = parseChronycTracking(
    TRACKING.replace('0.000125000 seconds slow', '0.000300000 seconds fast')
      .replace('Stratum         : 8', 'Stratum         : 0')
      .replace('Leap status     : Normal', 'Leap status     : Not synchronised')
  )
  assert.equal(status.system_time_offset_s, 0.0003)
  assert.equal(status.synchronized, false)
})

test('createChronyStatusProvider uses an allow-listed shell-free command', async () => {
  const calls = []
  const provider = createChronyStatusProvider({
    execFileAsync: async (...args) => {
      calls.push(args)
      return { stdout: TRACKING, stderr: '' }
    },
  })
  const status = await provider()
  assert.equal(status.available, true)
  assert.equal(calls[0][0], 'chronyc')
  assert.deepEqual(calls[0][1], ['-n', 'tracking'])
  assert.equal(calls[0][2].timeout, 1500)
})

test('createChronyStatusProvider reports missing chronyc without throwing', async () => {
  const provider = createChronyStatusProvider({
    execFileAsync: async () => {
      const error = new Error('missing')
      error.code = 'ENOENT'
      throw error
    },
  })
  const status = await provider()
  assert.equal(status.available, false)
  assert.equal(status.synchronized, false)
  assert.equal(status.error, 'chronyc is not installed')
})
