'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { parseGatewayConfig } = require('../src/config')

test('parseGatewayConfig prefers argv over env and coerces values', () => {
  const config = parseGatewayConfig(
    [
      '--device',
      '/tmp/xbee0',
      '--heartbeat-hz',
      '5',
      '--host',
      '0.0.0.0',
      '--antenna-enable',
      'true',
      '--rocket-m2-enable',
      'yes',
      '--video-config',
      '/tmp/video_streams.json',
      '--video-jitter-ms',
      '55',
      '--video-availability-stale-ms',
      '2222',
    ],
    {
      XBEE_DEVICE: '/tmp/ignored',
      XBEE_WS_HOST: '127.0.0.1',
      XBEE_HEARTBEAT_HZ: '2',
      BASE_ANTENNA_ENABLE: 'false',
      ROCKET_M2_ENABLE: 'false',
      VIDEO_CONFIG_PATH: '/tmp/ignored_video_streams.json',
      VIDEO_JITTER_LATENCY_MS: '90',
      VIDEO_AVAILABILITY_STALE_MS: '9999',
    }
  )

  assert.equal(config.device, '/tmp/xbee0')
  assert.equal('baud' in config, false)
  assert.equal(config.host, '0.0.0.0')
  assert.equal(config.heartbeatHz, 5)
  assert.equal(config.antennaEnable, true)
  assert.equal(config.rocketM2Enable, true)
  assert.equal(config.videoConfigPath, '/tmp/video_streams.json')
  assert.equal(config.videoJitterLatencyMs, 55)
  assert.equal(config.videoAvailabilityStaleMs, 2222)
  assert.equal('cmdHz' in config, false)
  assert.equal('cmdTimeoutMs' in config, false)
})

test('parseGatewayConfig falls back to defaults when values are absent', () => {
  const config = parseGatewayConfig([], {})

  assert.equal(config.device, '/dev/ttyXBEE')
  assert.equal('baud' in config, false)
  assert.equal(config.host, '0.0.0.0')
  assert.equal(config.port, 8081)
  assert.equal(config.linkTimeoutMs, 2000)
  assert.equal(config.antennaEnable, false)
  assert.equal(config.rocketM2Ip, '192.168.1.100')
  assert.match(config.videoConfigPath, /video_streams\.json$/)
  assert.equal(config.videoReceiverRestartMs, 1000)
  assert.equal(config.videoAvailabilityStaleMs, 1500)
})
