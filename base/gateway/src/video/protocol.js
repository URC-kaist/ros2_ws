'use strict'

const MESSAGE_TYPE_CONFIG = 1
const MESSAGE_TYPE_CHUNK = 2
const CHUNK_FLAG_KEY = 1 << 0
const CHUNK_FLAG_DELTA = 1 << 1

function toBuffer(value) {
  if (Buffer.isBuffer(value)) return value
  if (value instanceof Uint8Array) return Buffer.from(value)
  if (value == null) return Buffer.alloc(0)
  return Buffer.from(String(value), 'utf8')
}

function encodeConfigMessage(message) {
  const streamId = toBuffer(message.stream_id)
  const codec = toBuffer(message.codec || '')
  const sps = toBuffer(message.sps)
  const pps = toBuffer(message.pps)
  const totalLength =
    1 + 2 + 2 + 4 + 4 + 2 + 2 + streamId.length + codec.length + sps.length + pps.length
  const payload = Buffer.alloc(totalLength)

  let offset = 0
  payload.writeUInt8(MESSAGE_TYPE_CONFIG, offset)
  offset += 1
  payload.writeUInt16LE(streamId.length, offset)
  offset += 2
  payload.writeUInt16LE(codec.length, offset)
  offset += 2
  payload.writeUInt32LE(sps.length, offset)
  offset += 4
  payload.writeUInt32LE(pps.length, offset)
  offset += 4
  payload.writeUInt16LE(message.width || 0, offset)
  offset += 2
  payload.writeUInt16LE(message.height || 0, offset)
  offset += 2

  streamId.copy(payload, offset)
  offset += streamId.length
  codec.copy(payload, offset)
  offset += codec.length
  sps.copy(payload, offset)
  offset += sps.length
  pps.copy(payload, offset)

  return payload
}

function encodeChunkMessage(message) {
  const streamId = toBuffer(message.stream_id)
  const accessUnit = toBuffer(message.payload)
  const flags = message.key ? CHUNK_FLAG_KEY : CHUNK_FLAG_DELTA
  const timestampUs = BigInt(Math.max(0, Math.floor(message.timestamp_us || 0)))
  const totalLength = 1 + 2 + 1 + 8 + 4 + streamId.length + accessUnit.length
  const payload = Buffer.alloc(totalLength)

  let offset = 0
  payload.writeUInt8(MESSAGE_TYPE_CHUNK, offset)
  offset += 1
  payload.writeUInt16LE(streamId.length, offset)
  offset += 2
  payload.writeUInt8(flags, offset)
  offset += 1
  payload.writeBigUInt64LE(timestampUs, offset)
  offset += 8
  payload.writeUInt32LE(accessUnit.length, offset)
  offset += 4

  streamId.copy(payload, offset)
  offset += streamId.length
  accessUnit.copy(payload, offset)

  return payload
}

module.exports = {
  CHUNK_FLAG_DELTA,
  CHUNK_FLAG_KEY,
  MESSAGE_TYPE_CHUNK,
  MESSAGE_TYPE_CONFIG,
  encodeChunkMessage,
  encodeConfigMessage,
}
