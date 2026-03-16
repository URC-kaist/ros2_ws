'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const { EventEmitter } = require('events')

const { buildGstReceiveArgs, createVideoStreamReceiver } = require('../src/video/receiver')

test('buildGstReceiveArgs uses the configured jitter latency and supported drop policy', () => {
  const args = buildGstReceiveArgs(
    {
      stream_id: 'front_nav_cam',
      udp_port: 5000,
    },
    {
      jitterLatencyMs: 40,
    }
  )

  assert.deepEqual(args.slice(0, 8), [
    '-q',
    'udpsrc',
    'port=5000',
    'caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000',
    '!',
    'rtpjitterbuffer',
    'latency=40',
    'drop-on-latency=true',
  ])
})

test('createVideoStreamReceiver drops trailing partial access units when the child exits', () => {
  let child = null
  const accessUnits = []
  const receiver = createVideoStreamReceiver({
    stream: {
      stream_id: 'front_nav_cam',
      udp_port: 5000,
    },
    idleFlushMs: 100,
    onAccessUnit: (_streamId, accessUnit) => accessUnits.push(accessUnit),
    restartMs: 1000,
    spawnImpl: () => {
      child = new EventEmitter()
      child.stdout = new EventEmitter()
      child.stderr = new EventEmitter()
      return child
    },
  })

  receiver.start()
  child.stdout.emit('data', Buffer.from([0x00, 0x00, 0x00, 0x01, 0x65, 0x88, 0x84]))
  child.emit('exit', 1, null)
  receiver.stop()

  assert.deepEqual(accessUnits, [])
})
