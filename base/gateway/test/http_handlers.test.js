'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  handleRocketM2Status,
  handleTransitiveToken,
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

test('handleTransitiveToken fails cleanly when secret is missing', () => {
  const res = createResponseRecorder()
  const logs = []

  handleTransitiveToken(
    {
      url: '/transitive/token',
      headers: { host: 'localhost' },
      socket: { remoteAddress: '127.0.0.1' },
    },
    res,
    {
      env: {},
      log: (message) => logs.push(message),
    }
  )

  assert.equal(res.statusCode, 500)
  assert.deepEqual(JSON.parse(res.body), { error: 'TRANSITIVE_JWT_SECRET not set' })
  assert.ok(logs.some((entry) => entry.includes('TRANSITIVE_JWT_SECRET not set')))
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
