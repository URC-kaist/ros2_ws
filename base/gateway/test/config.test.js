'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { parseGatewayConfig } = require('../src/config')

test('parseGatewayConfig prefers argv over env and coerces values', () => {
  const config = parseGatewayConfig(
    [
      '--device',
      '/tmp/sik0',
      '--baud',
      '115200',
      '--heartbeat-hz',
      '5',
      '--antenna-enable',
      'true',
      '--rocket-m2-enable',
      'yes',
    ],
    {
      SIK_DEVICE: '/tmp/ignored',
      SIK_BAUD: '9600',
      SIK_HEARTBEAT_HZ: '2',
      BASE_ANTENNA_ENABLE: 'false',
      ROCKET_M2_ENABLE: 'false',
    }
  )

  assert.equal(config.device, '/tmp/sik0')
  assert.equal(config.baud, 115200)
  assert.equal(config.heartbeatHz, 5)
  assert.equal(config.antennaEnable, true)
  assert.equal(config.rocketM2Enable, true)
  assert.equal('cmdHz' in config, false)
  assert.equal('cmdTimeoutMs' in config, false)
})

test('parseGatewayConfig falls back to defaults when values are absent', () => {
  const config = parseGatewayConfig([], {})

  assert.equal(config.device, '/dev/ttySIK')
  assert.equal(config.port, 8081)
  assert.equal(config.linkTimeoutMs, 2000)
  assert.equal(config.antennaEnable, false)
})
