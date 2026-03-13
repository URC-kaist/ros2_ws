'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const { EventEmitter } = require('events')

const { createWsHub } = require('../src/runtime/ws_hub')

class FakeClient {
  constructor() {
    this.readyState = 1
    this.sent = []
    this.handlers = new Map()
    this.terminated = false
  }

  on(event, handler) {
    this.handlers.set(event, handler)
  }

  send(payload) {
    this.sent.push(payload)
  }

  emitMessage(payload) {
    const handler = this.handlers.get('message')
    if (handler) {
      handler(Buffer.from(payload))
    }
  }

  terminate() {
    this.terminated = true
  }
}

class FakeWebSocketServer extends EventEmitter {
  constructor() {
    super()
    this.clients = new Set()
    this.closed = false
  }

  close() {
    this.closed = true
  }
}

test('createWsHub sends initial messages, forwards JSON, and broadcasts', () => {
  const received = []
  const hub = createWsHub({
    server: {},
    WebSocketServerImpl: FakeWebSocketServer,
    getInitialMessages: () => [{ type: 'link_status', connected: true }],
    onMessage: (message) => received.push(message),
  })

  const client = new FakeClient()
  hub.wss.emit('connection', client)
  hub.wss.clients.add(client)

  assert.equal(client.sent.length, 1)
  assert.deepEqual(JSON.parse(client.sent[0]), { type: 'link_status', connected: true })

  client.emitMessage(JSON.stringify({ type: 'heartbeat' }))
  assert.deepEqual(received, [{ type: 'heartbeat' }])

  hub.broadcast({ type: 'telem_nav', heading_deg: 90 })
  assert.deepEqual(JSON.parse(client.sent[1]), { type: 'telem_nav', heading_deg: 90 })

  hub.close()
  assert.equal(client.terminated, true)
  assert.equal(hub.wss.closed, true)
})
