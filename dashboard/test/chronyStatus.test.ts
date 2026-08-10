import assert from 'node:assert/strict'

import {
  evaluateChronyReadiness,
  parseRoverChronyDiagnostic,
  type ChronyStatus,
} from '../src/lib/chronyStatus'

const values = {
  schema_version: '1',
  role: 'rover',
  available: 'True',
  synchronized: 'True',
  reference_id: 'C0A8010A',
  reference_name: '192.168.1.10',
  stratum: '9',
  system_time_offset_s: '0.00025',
  last_offset_s: '0.00002',
  rms_offset_s: '0.00003',
  root_delay_s: '0.0005',
  root_dispersion_s: '0.0003',
  update_interval_s: '4',
  leap_status: 'Normal',
  sampled_at_epoch_ms: '1000',
  error: '',
}

const rover = parseRoverChronyDiagnostic(
  {
    status: [
      {
        level: 0,
        name: 'clock',
        message: 'synchronized',
        values: Object.entries(values).map(([key, value]) => ({ key, value })),
      },
    ],
  },
  2_000
)
assert.equal(rover.role, 'rover')
assert.equal(rover.synchronized, true)
assert.equal(rover.systemTimeOffsetS, 0.00025)

const base: ChronyStatus = {
  ...rover,
  role: 'base',
  referenceName: 'LOCAL',
  stratum: 8,
}
let readiness = evaluateChronyReadiness(base, rover, 2_100)
assert.equal(readiness.ready, true)
assert.ok(Math.abs((readiness.roverErrorBoundS ?? 0) - 0.00055) < 1e-12)

readiness = evaluateChronyReadiness(
  base,
  { ...rover, systemTimeOffsetS: 0.003, rootDispersionS: 0.003 },
  2_100
)
assert.equal(readiness.ready, false)
assert.ok(readiness.reasons.some((reason) => reason.includes('offset exceeds')))
assert.ok(readiness.reasons.some((reason) => reason.includes('error bound exceeds')))

readiness = evaluateChronyReadiness(base, rover, 8_000)
assert.equal(readiness.ready, false)
assert.ok(readiness.reasons.some((reason) => reason.includes('stale')))

console.log('chrony status parser/readiness test passed')
