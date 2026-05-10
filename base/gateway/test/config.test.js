'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { parseGatewayConfig } = require('../src/config')

test('parseGatewayConfig prefers argv over env and coerces values', () => {
  const config = parseGatewayConfig(
    [
      '--base-xbee-device',
      '/tmp/xbee0',
      '--base-xbee-heartbeat-hz',
      '5',
      '--gateway-host',
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
      '--mavproxy-enable',
      'false',
      '--mavproxy-master-device',
      '/tmp/sik0',
      '--mavproxy-master-baud',
      '115200',
      '--mavproxy-out',
      'udp:127.0.0.1:14550',
      '--mavproxy-default-modules',
      'link,wp',
    ],
    {
      BASE_XBEE_DEVICE: '/tmp/ignored',
      MR2_GATEWAY_HOST: '127.0.0.1',
      BASE_XBEE_HEARTBEAT_HZ: '2',
      BASE_ANTENNA_ENABLE: 'false',
      ROCKET_M2_ENABLE: 'false',
      VIDEO_CONFIG_PATH: '/tmp/ignored_video_streams.json',
      VIDEO_JITTER_LATENCY_MS: '90',
      VIDEO_AVAILABILITY_STALE_MS: '9999',
      MAVPROXY_ENABLE: 'true',
      MAVPROXY_MASTER_DEVICE: '/dev/ignored',
      MAVPROXY_MASTER_BAUD: '57600',
      MAVPROXY_OUT: 'udp:192.168.1.108:14550',
      MAVPROXY_DEFAULT_MODULES: 'adsb',
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
  assert.equal(config.mavproxyEnable, false)
  assert.equal(config.mavproxyMasterDevice, '/tmp/sik0')
  assert.equal(config.mavproxyMasterBaud, 115200)
  assert.equal(config.mavproxyOut, 'udp:127.0.0.1:14550')
  assert.equal(config.mavproxyDefaultModules, 'link,wp')
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
  assert.equal(config.rocketM2Ip, '')
  assert.match(config.videoConfigPath, /video_streams\.json$/)
  assert.equal(config.videoReceiverRestartMs, 1000)
  assert.equal(config.videoAvailabilityStaleMs, 1500)
  assert.equal(config.mavproxyEnable, true)
  assert.equal(config.mavproxyBinary, 'mavproxy.py')
  assert.equal(config.mavproxyMasterDevice, '/dev/ttySIK')
  assert.equal(config.mavproxyMasterBaud, 57600)
  assert.equal(config.mavproxyOut, 'udp:192.168.1.108:14550')
  assert.equal(config.mavproxyDefaultModules, '')
})

test('parseGatewayConfig accepts top-level MR2 network env fallbacks', () => {
  const config = parseGatewayConfig([], {
    MR2_GATEWAY_HOST: '127.0.0.2',
    MR2_GATEWAY_PORT: '18081',
    MR2_BASE_ROCKET_IP: '192.168.1.110',
  })

  assert.equal(config.host, '127.0.0.2')
  assert.equal(config.port, 18081)
  assert.equal(config.rocketM2Ip, '192.168.1.110')
  assert.deepEqual(
    config.rocketM2Targets.map((target) => [target.target, target.ip]),
    [
      ['base', '192.168.1.110'],
      ['drone', ''],
      ['rover', ''],
    ]
  )
})

test('parseGatewayConfig ignores removed legacy gateway env and flags', () => {
  const config = parseGatewayConfig(
    [
      '--device',
      '/tmp/legacy_device',
      '--host',
      '127.0.0.9',
      '--port',
      '19090',
      '--heartbeat-hz',
      '9',
      '--link-timeout-ms',
      '9000',
    ],
    {
      XBEE_DEVICE: '/tmp/legacy_env_device',
      XBEE_WS_HOST: '127.0.0.8',
      XBEE_WS_PORT: '18080',
      XBEE_HEARTBEAT_HZ: '8',
      XBEE_LINK_TIMEOUT_MS: '8000',
      ROCKET_M2_IP: '192.168.1.200',
    }
  )

  assert.equal(config.device, '/dev/ttyXBEE')
  assert.equal(config.host, '0.0.0.0')
  assert.equal(config.port, 8081)
  assert.equal(config.heartbeatHz, 2)
  assert.equal(config.linkTimeoutMs, 2000)
  assert.equal(config.rocketM2Ip, '')
})
