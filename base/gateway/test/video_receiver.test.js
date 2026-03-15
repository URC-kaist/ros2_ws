'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { buildGstReceiveArgs } = require('../src/video/receiver')

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
