export const VIDEO_MESSAGE_CONFIG = 1
export const VIDEO_MESSAGE_CHUNK = 2
export const VIDEO_MESSAGE_TIMED_CHUNK = 3

export type VideoFrameCorrelation = {
  ssrc: number
  rtpTimestamp: number
  markerSequence: number
}

export type VideoConfigMessage = {
  kind: 'config'
  streamId: string
  width: number
  height: number
  codec: string
  sps: Uint8Array
  pps: Uint8Array
}

export type VideoChunkMessage = {
  kind: 'chunk'
  streamId: string
  /** Base gateway time after RTP jitter/depay/H.264 parsing (T7a). */
  baseIngestTimestampUs: number
  /** Compatibility/decode timestamp; identical to baseIngestTimestampUs. */
  timestampUs: number
  /** Set by videoGateway at WebSocket message callback entry (T7b). */
  browserReceiveEpochUs: number | null
  correlation: VideoFrameCorrelation | null
  key: boolean
  delta: boolean
  payload: Uint8Array
}

export type VideoGatewayMessage = VideoConfigMessage | VideoChunkMessage

const CHUNK_FLAG_KEY = 1 << 0
const CHUNK_FLAG_DELTA = 1 << 1

const textDecoder = new TextDecoder()

function decodeText(bytes: Uint8Array) {
  return textDecoder.decode(bytes)
}

function ensureBounds(view: DataView, offset: number, length: number) {
  return offset + length <= view.byteLength
}

export function decodeVideoMessage(buffer: ArrayBuffer): VideoGatewayMessage | null {
  const view = new DataView(buffer)
  if (view.byteLength < 1) return null

  const type = view.getUint8(0)
  if (type === VIDEO_MESSAGE_CONFIG) {
    if (!ensureBounds(view, 1, 16)) return null
    const streamIdLength = view.getUint16(1, true)
    const codecLength = view.getUint16(3, true)
    const spsLength = view.getUint32(5, true)
    const ppsLength = view.getUint32(9, true)
    const width = view.getUint16(13, true)
    const height = view.getUint16(15, true)
    const bodyOffset = 17
    const totalLength = streamIdLength + codecLength + spsLength + ppsLength
    if (!ensureBounds(view, bodyOffset, totalLength)) return null

    const bytes = new Uint8Array(buffer)
    let offset = bodyOffset
    const streamId = decodeText(bytes.slice(offset, offset + streamIdLength))
    offset += streamIdLength
    const codec = decodeText(bytes.slice(offset, offset + codecLength))
    offset += codecLength
    const sps = bytes.slice(offset, offset + spsLength)
    offset += spsLength
    const pps = bytes.slice(offset, offset + ppsLength)

    return {
      kind: 'config',
      streamId,
      width,
      height,
      codec,
      sps,
      pps,
    }
  }

  if (type === VIDEO_MESSAGE_CHUNK) {
    if (!ensureBounds(view, 1, 15)) return null
    const streamIdLength = view.getUint16(1, true)
    const flags = view.getUint8(3)
    const timestampUs = Number(view.getBigUint64(4, true))
    const payloadLength = view.getUint32(12, true)
    const bodyOffset = 16
    const totalLength = streamIdLength + payloadLength
    if (!ensureBounds(view, bodyOffset, totalLength)) return null

    const bytes = new Uint8Array(buffer)
    const streamId = decodeText(bytes.slice(bodyOffset, bodyOffset + streamIdLength))
    const payload = bytes.slice(bodyOffset + streamIdLength, bodyOffset + streamIdLength + payloadLength)

    return {
      kind: 'chunk',
      streamId,
      baseIngestTimestampUs: timestampUs,
      timestampUs,
      browserReceiveEpochUs: null,
      correlation: null,
      key: (flags & CHUNK_FLAG_KEY) !== 0,
      delta: (flags & CHUNK_FLAG_DELTA) !== 0,
      payload,
    }
  }

  if (type === VIDEO_MESSAGE_TIMED_CHUNK) {
    if (!ensureBounds(view, 1, 33)) return null
    const streamIdLength = view.getUint16(1, true)
    const flags = view.getUint8(3)
    const decodeTimestampUs = Number(view.getBigUint64(4, true))
    const ssrc = view.getUint32(12, true)
    const rtpTimestamp = view.getUint32(16, true)
    const markerSequence = view.getUint16(20, true)
    const baseAccessUnitEpochUs = Number(view.getBigUint64(22, true))
    const payloadLength = view.getUint32(30, true)
    const bodyOffset = 34
    const totalLength = streamIdLength + payloadLength
    if (!ensureBounds(view, bodyOffset, totalLength)) return null
    const bytes = new Uint8Array(buffer)
    const streamId = decodeText(bytes.slice(bodyOffset, bodyOffset + streamIdLength))
    const payload = bytes.slice(
      bodyOffset + streamIdLength,
      bodyOffset + streamIdLength + payloadLength
    )
    return {
      kind: 'chunk',
      streamId,
      baseIngestTimestampUs: baseAccessUnitEpochUs,
      timestampUs: decodeTimestampUs,
      browserReceiveEpochUs: null,
      correlation: { ssrc, rtpTimestamp, markerSequence },
      key: (flags & CHUNK_FLAG_KEY) !== 0,
      delta: (flags & CHUNK_FLAG_DELTA) !== 0,
      payload,
    }
  }

  return null
}
