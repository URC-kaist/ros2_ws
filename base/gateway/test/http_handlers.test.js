'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  handleRocketM2Status,
  createGatewayHttpHandler,
} = require('../src/runtime/http_handlers')

function createResponseRecorder() {
  return {
    statusCode: null,
    headers: null,
    body: '',
    writeHead(statusCode, headers) {
      this.statusCode = statusCode
      this.headers = headers
    },
    end(body = '') {
      this.body = body
    },
  }
}

test('handleRocketM2Status reports disabled gateway state', () => {
  const res = createResponseRecorder()

  handleRocketM2Status({}, res, {
    enabled: false,
    configured: false,
    status: null,
  })

  assert.equal(res.statusCode, 503)
  assert.deepEqual(JSON.parse(res.body), { error: 'Rocket M2 disabled' })
})

test('handleRocketM2Status returns current proxied status', () => {
  const res = createResponseRecorder()

  handleRocketM2Status({}, res, {
    enabled: true,
    configured: true,
    status: {
      connected: true,
      updated_at_ms: 100,
      last_success_ms: 90,
      signal: -60,
      rssi: -59,
      noisef: -95,
      chwidth: 20,
      rx_chainmask: 3,
      chainrssi: [-60, -61],
      chainrssimgmt: [],
      chainrssiext: [],
      error: null,
    },
  })

  assert.equal(res.statusCode, 200)
  assert.equal(JSON.parse(res.body).type, 'rocket_m2_status')
})

test('handleRocketM2Status returns current fleet statuses', () => {
  const res = createResponseRecorder()

  handleRocketM2Status({}, res, {
    enabled: true,
    configured: true,
    statuses: [
      {
        target: 'base',
        label: 'Base',
        connected: true,
        updated_at_ms: 100,
        last_success_ms: 90,
        signal: -60,
        rssi: -59,
        noisef: -95,
        chwidth: 20,
        rx_chainmask: 3,
        chainrssi: [-60, -61],
        chainrssimgmt: [],
        chainrssiext: [],
        error: null,
      },
    ],
  })

  const body = JSON.parse(res.body)
  assert.equal(res.statusCode, 200)
  assert.equal(body.type, 'rocket_m2_statuses')
  assert.equal(body.statuses[0].target, 'base')
})

test('createGatewayHttpHandler serves video stream metadata', () => {
  const handler = createGatewayHttpHandler({
    getVideoStreams: () => [
      {
        stream_id: 'front_nav_cam',
        udp_port: 5000,
      },
    ],
  })
  const res = createResponseRecorder()

  handler(
    {
      method: 'GET',
      url: '/video/streams',
    },
    res
  )

  assert.equal(res.statusCode, 200)
  assert.deepEqual(JSON.parse(res.body), {
    streams: [
      {
        stream_id: 'front_nav_cam',
        udp_port: 5000,
      },
    ],
  })
})

test('createGatewayHttpHandler serves latency clock samples without caching', () => {
  const handler = createGatewayHttpHandler()
  const res = createResponseRecorder()

  handler(
    {
      method: 'GET',
      url: '/latency/time',
    },
    res
  )

  const body = JSON.parse(res.body)
  assert.equal(res.statusCode, 200)
  assert.equal(res.headers['Cache-Control'], 'no-store')
  assert.equal(Number.isInteger(body.server_receive_epoch_us), true)
  assert.equal(Number.isInteger(body.server_send_epoch_us), true)
  assert.equal(body.server_send_epoch_us >= body.server_receive_epoch_us, true)
})

test('createGatewayHttpHandler serves read-only chrony status without caching', async () => {
  const handler = createGatewayHttpHandler({
    getChronyStatus: async () => ({
      schema_version: 1,
      role: 'base',
      available: true,
      synchronized: true,
      reference_id: '7F7F0101',
    }),
  })
  const res = createResponseRecorder()

  await handler({ method: 'GET', url: '/latency/clock-status' }, res)

  assert.equal(res.statusCode, 200)
  assert.equal(res.headers['Cache-Control'], 'no-store')
  assert.equal(JSON.parse(res.body).reference_id, '7F7F0101')
})
