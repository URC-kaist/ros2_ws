'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { AnnexBAccessUnitParser, deriveCodecString } = require('../src/video/h264')

function annexb(bytes) {
  return Buffer.from([0x00, 0x00, 0x00, 0x01, ...bytes])
}

test('AnnexBAccessUnitParser groups SPS/PPS/IDR into one key access unit', () => {
  const parser = new AnnexBAccessUnitParser()
  const sps = annexb([0x67, 0x42, 0xe0, 0x1f])
  const pps = annexb([0x68, 0xce, 0x38, 0x80])
  const idr = annexb([0x65, 0xe0])

  const emitted = parser.push(Buffer.concat([sps, pps, idr]))
  assert.equal(emitted.length, 0)

  const flushed = parser.flush()
  assert.equal(flushed.length, 1)
  assert.equal(flushed[0].key, true)
  assert.equal(flushed[0].delta, false)
  assert.equal(flushed[0].codec, 'avc1.42E01F')
  assert.deepEqual(flushed[0].sps, Buffer.from([0x67, 0x42, 0xe0, 0x1f]))
  assert.deepEqual(flushed[0].pps, Buffer.from([0x68, 0xce, 0x38, 0x80]))
})

test('AnnexBAccessUnitParser splits on new VCL access units', () => {
  const parser = new AnnexBAccessUnitParser()
  const idr = annexb([0x65, 0xe0])
  const deltaOne = annexb([0x41, 0xe0])
  const deltaTwo = annexb([0x41, 0xe0])

  const emitted = parser.push(Buffer.concat([idr, deltaOne, deltaTwo]))
  assert.equal(emitted.length, 1)
  assert.equal(emitted[0].key, true)

  const flushed = parser.flush()
  assert.equal(flushed.length, 2)
  assert.equal(flushed[0].delta, true)
  assert.equal(flushed[1].delta, true)
})

test('deriveCodecString returns null when SPS bytes are incomplete', () => {
  assert.equal(deriveCodecString(Buffer.from([0x67, 0x42, 0xe0])), null)
})
