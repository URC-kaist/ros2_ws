'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const { EventEmitter } = require('events')

const { createVideoGateway } = require('../src/video/service')

class FakeClient {
  constructor() {
    this.readyState = 1
    this.bufferedAmount = 0
    this.sent = []
    this.handlers = new Map()
    this.terminated = false
  }

  on(event, handler) {
    this.handlers.set(event, handler)
  }

  send(payload) {
    this.sent.push(Buffer.from(payload))
  }

  emitJson(message) {
    const handler = this.handlers.get('message')
    if (handler) {
      handler(Buffer.from(JSON.stringify(message)))
    }
  }

  terminate() {
    this.terminated = true
  }
}

class FakeWebSocketServer extends EventEmitter {
  close() {
    this.closed = true
  }
}

function createReceiverHarness() {
  const callbacksByStream = new Map()

  return {
    callbacksByStream,
    factory(options) {
      callbacksByStream.set(options.stream.stream_id, {
        onAccessUnit: options.onAccessUnit,
        onAvailabilityChange: options.onAvailabilityChange,
      })
      return {
        start() {},
        stop() {},
      }
    },
  }
}

function createRouteRegistry() {
  return {
    path: null,
    wss: null,
    register(path, wss) {
      this.path = path
      this.wss = wss
    },
    unregister(path) {
      if (this.path === path) {
        this.path = null
        this.wss = null
      }
    },
  }
}

function createVideoConfig(overrides = {}) {
  return {
    streams: [
      {
        stream_id: 'front_nav_cam',
        source_type: 'ros_topic',
        ros_topic: '/front_camera/image_raw',
        ros_encoding: 'rgb8',
        v4l2_device: null,
        v4l2_pixel_format: null,
        udp_port: 5000,
        width: 640,
        height: 480,
        framerate: 15,
        display: {},
        ...overrides,
      },
    ],
  }
}

test('video gateway drops non-picture access units and re-sends config after restart', () => {
  const routeRegistry = createRouteRegistry()
  const receiverHarness = createReceiverHarness()
  const gateway = createVideoGateway({
    WebSocketServerImpl: FakeWebSocketServer,
    createVideoStreamReceiverImpl: receiverHarness.factory,
    routeRegistry,
    videoConfig: createVideoConfig(),
  })

  const client = new FakeClient()
  routeRegistry.wss.emit('connection', client)
  client.emitJson({ type: 'subscribe', stream_id: 'front_nav_cam' })

  const receiver = receiverHarness.callbacksByStream.get('front_nav_cam')
  assert.ok(receiver)

  receiver.onAccessUnit('front_nav_cam', {
    codec: 'avc1.42E01F',
    delta: false,
    key: false,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0xe0, 0x1f]),
    pps: Buffer.from([0x68, 0xce, 0x38, 0x80]),
    sps: Buffer.from([0x67, 0x42, 0xe0, 0x1f]),
    timestamp_us: 1000,
  })
  assert.equal(client.sent.length, 0)

  receiver.onAccessUnit('front_nav_cam', {
    codec: 'avc1.42E01F',
    delta: false,
    key: true,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x65, 0xe0]),
    pps: Buffer.from([0x68, 0xce, 0x38, 0x80]),
    sps: Buffer.from([0x67, 0x42, 0xe0, 0x1f]),
    timestamp_us: 2000,
  })
  assert.equal(client.sent.length, 2)
  assert.equal(client.sent[0].readUInt8(0), 1)
  assert.equal(client.sent[1].readUInt8(0), 2)

  receiver.onAccessUnit('front_nav_cam', {
    codec: 'avc1.42E01F',
    delta: false,
    key: false,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x09, 0x10]),
    pps: null,
    sps: null,
    timestamp_us: 2500,
  })
  assert.equal(client.sent.length, 2)

  receiver.onAvailabilityChange('front_nav_cam', false)
  receiver.onAvailabilityChange('front_nav_cam', true)

  receiver.onAccessUnit('front_nav_cam', {
    codec: null,
    delta: false,
    key: true,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x65, 0xe1]),
    pps: null,
    sps: null,
    timestamp_us: 3000,
  })
  assert.equal(client.sent.length, 4)
  assert.equal(client.sent[2].readUInt8(0), 1)
  assert.equal(client.sent[3].readUInt8(0), 2)

  gateway.stop()
  assert.equal(client.terminated, true)
})

test('video gateway exposes SPS-derived dimensions through browser stream metadata', () => {
  const routeRegistry = createRouteRegistry()
  const receiverHarness = createReceiverHarness()
  const gateway = createVideoGateway({
    WebSocketServerImpl: FakeWebSocketServer,
    createVideoStreamReceiverImpl: receiverHarness.factory,
    routeRegistry,
    videoConfig: createVideoConfig({ width: null, height: null }),
  })

  const receiver = receiverHarness.callbacksByStream.get('front_nav_cam')
  assert.ok(receiver)

  receiver.onAccessUnit('front_nav_cam', {
    codec: 'avc1.F4001E',
    delta: false,
    height: 480,
    key: false,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x67]),
    pps: null,
    sps: Buffer.from(
      '67f4001e90d9680a03db016a0c0c0c80000003008000001e478b1750',
      'hex'
    ),
    timestamp_us: 1000,
    width: 640,
  })

  assert.deepEqual(gateway.getBrowserStreams(), [
    {
      available: false,
      display: {},
      encoded_height: 480,
      encoded_width: 640,
      framerate: 15,
      height: 480,
      ros_encoding: 'rgb8',
      ros_topic: '/front_camera/image_raw',
      source_type: 'ros_topic',
      stream_id: 'front_nav_cam',
      udp_port: 5000,
      v4l2_device: null,
      v4l2_pixel_format: null,
      width: 640,
    },
  ])

  gateway.stop()
})

test('video gateway does not report configured dimensions as encoded dimensions before ingest', () => {
  const routeRegistry = createRouteRegistry()
  const receiverHarness = createReceiverHarness()
  const gateway = createVideoGateway({
    WebSocketServerImpl: FakeWebSocketServer,
    createVideoStreamReceiverImpl: receiverHarness.factory,
    routeRegistry,
    videoConfig: createVideoConfig({ width: 848, height: 480 }),
  })

  const [stream] = gateway.getBrowserStreams()
  assert.equal(stream.width, 848)
  assert.equal(stream.height, 480)
  assert.equal(stream.encoded_width, null)
  assert.equal(stream.encoded_height, null)

  gateway.stop()
})

test('video gateway preserves configured display dimensions separately from encoded dimensions', () => {
  const routeRegistry = createRouteRegistry()
  const receiverHarness = createReceiverHarness()
  const gateway = createVideoGateway({
    WebSocketServerImpl: FakeWebSocketServer,
    createVideoStreamReceiverImpl: receiverHarness.factory,
    routeRegistry,
    videoConfig: createVideoConfig({ width: 848, height: 480 }),
  })

  const receiver = receiverHarness.callbacksByStream.get('front_nav_cam')
  assert.ok(receiver)

  receiver.onAccessUnit('front_nav_cam', {
    codec: 'avc1.F4001E',
    delta: false,
    height: 480,
    key: false,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x67]),
    pps: null,
    sps: Buffer.from(
      '67f4001e90d9680a03db016a0c0c0c80000003008000001e478b1750',
      'hex'
    ),
    timestamp_us: 1000,
    width: 640,
  })

  const [stream] = gateway.getBrowserStreams()
  assert.equal(stream.width, 848)
  assert.equal(stream.height, 480)
  assert.equal(stream.encoded_width, 640)
  assert.equal(stream.encoded_height, 480)

  gateway.stop()
})
