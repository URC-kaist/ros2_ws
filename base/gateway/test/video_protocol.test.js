'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  CHUNK_FLAG_KEY,
  MESSAGE_TYPE_CHUNK,
  MESSAGE_TYPE_CONFIG,
  encodeChunkMessage,
  encodeConfigMessage,
} = require('../src/video/protocol')

test('encodeConfigMessage packs the expected binary header fields', () => {
  const payload = encodeConfigMessage({
    stream_id: 'front_nav_cam',
    width: 640,
    height: 480,
    codec: 'AVC1.42E01F',
    sps: Buffer.from([0x67, 0x42, 0xe0, 0x1f]),
    pps: Buffer.from([0x68, 0xce, 0x38, 0x80]),
  })

  assert.equal(payload.readUInt8(0), MESSAGE_TYPE_CONFIG)
  assert.equal(payload.readUInt16LE(1), 'front_nav_cam'.length)
  assert.equal(payload.readUInt16LE(3), 'AVC1.42E01F'.length)
  assert.equal(payload.readUInt16LE(13), 640)
  assert.equal(payload.readUInt16LE(15), 480)
})

test('encodeChunkMessage marks key frames and preserves timestamps', () => {
  const payload = encodeChunkMessage({
    stream_id: 'front_nav_cam',
    timestamp_us: 123456,
    key: true,
    payload: Buffer.from([0x00, 0x00, 0x00, 0x01, 0x65, 0xe0]),
  })

  assert.equal(payload.readUInt8(0), MESSAGE_TYPE_CHUNK)
  assert.equal(payload.readUInt16LE(1), 'front_nav_cam'.length)
  assert.equal(payload.readUInt8(3), CHUNK_FLAG_KEY)
  assert.equal(Number(payload.readBigUInt64LE(4)), 123456)
})
