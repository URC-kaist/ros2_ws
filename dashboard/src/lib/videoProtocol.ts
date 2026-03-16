export const VIDEO_MESSAGE_CONFIG = 1
export const VIDEO_MESSAGE_CHUNK = 2

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
  timestampUs: number
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
      timestampUs,
      key: (flags & CHUNK_FLAG_KEY) !== 0,
      delta: (flags & CHUNK_FLAG_DELTA) !== 0,
      payload,
    }
  }

  return null
}
