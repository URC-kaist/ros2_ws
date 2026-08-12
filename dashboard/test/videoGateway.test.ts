import assert from 'node:assert/strict'

import { VideoGatewayClient } from '../src/lib/videoGateway'
import { VIDEO_MESSAGE_CHUNK, VIDEO_MESSAGE_CONFIG } from '../src/lib/videoProtocol'

type SocketListener = (event: { data?: ArrayBuffer }) => void

class FakeWebSocket {
  static readonly CONNECTING = 0
  static readonly OPEN = 1
  static readonly CLOSING = 2
  static readonly CLOSED = 3
  static instances: FakeWebSocket[] = []

  binaryType = ''
  readyState = FakeWebSocket.CONNECTING
  sent: string[] = []
  private listeners = new Map<string, Set<SocketListener>>()

  constructor(readonly url: string) {
    FakeWebSocket.instances.push(this)
  }

  addEventListener(type: string, listener: SocketListener) {
    const listeners = this.listeners.get(type) ?? new Set<SocketListener>()
    listeners.add(listener)
    this.listeners.set(type, listeners)
  }

  send(message: string) {
    this.sent.push(message)
  }

  close() {
    this.readyState = FakeWebSocket.CLOSED
    this.emit('close')
  }

  open() {
    this.readyState = FakeWebSocket.OPEN
    this.emit('open')
  }

  message(data: ArrayBuffer) {
    this.emit('message', { data })
  }

  private emit(type: string, event: { data?: ArrayBuffer } = {}) {
    for (const listener of this.listeners.get(type) ?? []) {
      listener(event)
    }
  }
}

function encodeConfig(streamId: string) {
  const stream = new TextEncoder().encode(streamId)
  const codec = new TextEncoder().encode('avc1.42E01F')
  const sps = new Uint8Array([0x67, 0x42, 0xe0, 0x1f])
  const pps = new Uint8Array([0x68, 0xce, 0x06, 0xe2])
  const buffer = new ArrayBuffer(17 + stream.length + codec.length + sps.length + pps.length)
  const view = new DataView(buffer)
  view.setUint8(0, VIDEO_MESSAGE_CONFIG)
  view.setUint16(1, stream.length, true)
  view.setUint16(3, codec.length, true)
  view.setUint32(5, sps.length, true)
  view.setUint32(9, pps.length, true)
  view.setUint16(13, 320, true)
  view.setUint16(15, 180, true)
  const bytes = new Uint8Array(buffer)
  let offset = 17
  bytes.set(stream, offset)
  offset += stream.length
  bytes.set(codec, offset)
  offset += codec.length
  bytes.set(sps, offset)
  offset += sps.length
  bytes.set(pps, offset)
  return buffer
}

function encodeKeyChunk(streamId: string) {
  const stream = new TextEncoder().encode(streamId)
  const payload = new Uint8Array([0, 0, 0, 1, 0x65, 1, 2, 3])
  const buffer = new ArrayBuffer(16 + stream.length + payload.length)
  const view = new DataView(buffer)
  view.setUint8(0, VIDEO_MESSAGE_CHUNK)
  view.setUint16(1, stream.length, true)
  view.setUint8(3, 1)
  view.setBigUint64(4, 123n, true)
  view.setUint32(12, payload.length, true)
  const bytes = new Uint8Array(buffer)
  bytes.set(stream, 16)
  bytes.set(payload, 16 + stream.length)
  return buffer
}

const originalWindow = globalThis.window
const originalWebSocket = globalThis.WebSocket
Object.defineProperty(globalThis, 'window', {
  configurable: true,
  value: {
    location: { origin: 'http://localhost', protocol: 'http:', host: 'localhost' },
    setTimeout,
    clearTimeout,
  },
})
Object.defineProperty(globalThis, 'WebSocket', {
  configurable: true,
  value: FakeWebSocket,
})

try {
  const client = new VideoGatewayClient('ws://localhost/video-ws')
  const liveFeedMessages: string[] = []
  const uplinkMessages: string[] = []
  const unsubscribeLiveFeed = client.subscribe('front', (message) => {
    liveFeedMessages.push(message.kind === 'config' ? 'config' : message.key ? 'key' : 'delta')
  })
  const socket = FakeWebSocket.instances[0]
  assert.ok(socket)
  socket.open()
  assert.deepEqual(socket.sent.map((message) => JSON.parse(message)), [
    { type: 'subscribe', stream_id: 'front' },
  ])

  socket.message(encodeConfig('front'))
  socket.message(encodeKeyChunk('front'))
  assert.deepEqual(liveFeedMessages, ['config', 'key'])

  const unsubscribeUplink = client.subscribe('front', (message) => {
    uplinkMessages.push(message.kind === 'config' ? 'config' : message.key ? 'key' : 'delta')
  })
  assert.deepEqual(uplinkMessages, ['config', 'key'])
  assert.equal(socket.sent.length, 1)

  unsubscribeUplink()
  assert.equal(socket.sent.length, 1)
  unsubscribeLiveFeed()
  assert.deepEqual(JSON.parse(socket.sent[1]), {
    type: 'unsubscribe',
    stream_id: 'front',
  })

  console.log('video gateway late-subscriber bootstrap test passed')
} finally {
  Object.defineProperty(globalThis, 'window', {
    configurable: true,
    value: originalWindow,
  })
  Object.defineProperty(globalThis, 'WebSocket', {
    configurable: true,
    value: originalWebSocket,
  })
}
