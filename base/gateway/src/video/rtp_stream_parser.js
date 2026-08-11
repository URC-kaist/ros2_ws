'use strict'

const MAX_RTP_PACKET_BYTES = 65535

function parseRtpPacket(packet) {
  if (!Buffer.isBuffer(packet) || packet.length < 12) return null
  const first = packet.readUInt8(0)
  const second = packet.readUInt8(1)
  if (first >> 6 !== 2) return null
  const csrcCount = first & 0x0f
  let headerLength = 12 + csrcCount * 4
  if (packet.length < headerLength) return null
  if ((first & 0x10) !== 0) {
    if (packet.length < headerLength + 4) return null
    const extensionWords = packet.readUInt16BE(headerLength + 2)
    headerLength += 4 + extensionWords * 4
    if (packet.length < headerLength) return null
  }
  return {
    marker: (second & 0x80) !== 0,
    payload_type: second & 0x7f,
    sequence: packet.readUInt16BE(2),
    rtp_timestamp: packet.readUInt32BE(4),
    ssrc: packet.readUInt32BE(8),
  }
}

class RtpStreamParser {
  constructor(options = {}) {
    this.maximumPacketBytes = options.maximumPacketBytes || MAX_RTP_PACKET_BYTES
    this.buffer = Buffer.alloc(0)
  }

  push(chunk) {
    if (!chunk || chunk.length === 0) return []
    this.buffer = this.buffer.length === 0
      ? Buffer.from(chunk)
      : Buffer.concat([this.buffer, chunk])
    const packets = []
    while (this.buffer.length >= 2) {
      const packetLength = this.buffer.readUInt16BE(0)
      if (packetLength < 12 || packetLength > this.maximumPacketBytes) {
        this.buffer = Buffer.alloc(0)
        throw new Error(`invalid RTP stream packet length ${packetLength}`)
      }
      if (this.buffer.length < packetLength + 2) break
      const parsed = parseRtpPacket(this.buffer.subarray(2, packetLength + 2))
      this.buffer = this.buffer.subarray(packetLength + 2)
      if (parsed) packets.push(parsed)
    }
    return packets
  }

  reset() {
    this.buffer = Buffer.alloc(0)
  }
}

module.exports = {
  MAX_RTP_PACKET_BYTES,
  RtpStreamParser,
  parseRtpPacket,
}
