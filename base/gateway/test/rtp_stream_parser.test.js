'use strict'

const assert = require('node:assert/strict')
const test = require('node:test')

const { RtpStreamParser, parseRtpPacket } = require('../src/video/rtp_stream_parser')

function makePacket() {
  const packet = Buffer.alloc(24)
  packet.writeUInt8(0x91, 0)
  packet.writeUInt8(0x80 | 96, 1)
  packet.writeUInt16BE(65535, 2)
  packet.writeUInt32BE(123456, 4)
  packet.writeUInt32BE(0x10203040, 8)
  packet.writeUInt32BE(0x01020304, 12)
  packet.writeUInt16BE(0x1000, 16)
  packet.writeUInt16BE(1, 18)
  return packet
}

test('parseRtpPacket reads marker key through CSRC and extension headers', () => {
  assert.deepEqual(parseRtpPacket(makePacket()), {
    marker: true,
    payload_type: 96,
    sequence: 65535,
    rtp_timestamp: 123456,
    ssrc: 0x10203040,
  })
})

test('RtpStreamParser accepts split RFC4571 packets', () => {
  const packet = makePacket()
  const framed = Buffer.alloc(packet.length + 2)
  framed.writeUInt16BE(packet.length, 0)
  packet.copy(framed, 2)
  const parser = new RtpStreamParser()
  assert.deepEqual(parser.push(framed.subarray(0, 7)), [])
  assert.equal(parser.push(framed.subarray(7))[0].sequence, 65535)
})
