import { useEffect, useMemo, useRef, useState, type CSSProperties } from 'react'
import { getVideoGatewayClient, type VideoStreamInfo } from '../lib/videoGateway'
import type { VideoChunkMessage, VideoConfigMessage } from '../lib/videoProtocol'
import './VideoStreamCard.css'

type VideoStreamCardProps = {
  stream: VideoStreamInfo
  videoWidth?: number
  videoHeight?: number
}

type H264PayloadFormat = 'annexb' | 'avcc'

function findStartCodeLength(bytes: Uint8Array, offset: number) {
  if (offset + 3 > bytes.length) return 0
  if (bytes[offset] !== 0 || bytes[offset + 1] !== 0) return 0
  if (bytes[offset + 2] === 1) return 3
  if (offset + 4 <= bytes.length && bytes[offset + 2] === 0 && bytes[offset + 3] === 1) return 4
  return 0
}

function splitAnnexBNalus(bytes: Uint8Array) {
  const starts: number[] = []
  for (let index = 0; index < bytes.length - 2; index += 1) {
    const length = findStartCodeLength(bytes, index)
    if (length > 0) {
      starts.push(index)
      index += length - 1
    }
  }

  const nals: Uint8Array[] = []
  for (let index = 0; index < starts.length; index += 1) {
    const start = starts[index]
    const startCodeLength = findStartCodeLength(bytes, start)
    const nextStart = index + 1 < starts.length ? starts[index + 1] : bytes.length
    nals.push(bytes.slice(start + startCodeLength, nextStart))
  }
  return nals.filter((nal) => nal.length > 0)
}

function annexBToAvcc(bytes: Uint8Array) {
  const nals = splitAnnexBNalus(bytes)
  const totalLength = nals.reduce((sum, nal) => sum + 4 + nal.length, 0)
  const output = new Uint8Array(totalLength)
  const view = new DataView(output.buffer)
  let offset = 0

  for (const nal of nals) {
    view.setUint32(offset, nal.length, false)
    offset += 4
    output.set(nal, offset)
    offset += nal.length
  }

  return output
}

function buildAvcDescription(config: VideoConfigMessage) {
  const sps = config.sps
  const pps = config.pps
  const output = new Uint8Array(11 + sps.length + pps.length)
  let offset = 0
  output[offset++] = 1
  output[offset++] = sps[1] ?? 0x42
  output[offset++] = sps[2] ?? 0xe0
  output[offset++] = sps[3] ?? 0x1f
  output[offset++] = 0xff
  output[offset++] = 0xe1
  output[offset++] = (sps.length >> 8) & 0xff
  output[offset++] = sps.length & 0xff
  output.set(sps, offset)
  offset += sps.length
  output[offset++] = 1
  output[offset++] = (pps.length >> 8) & 0xff
  output[offset++] = pps.length & 0xff
  output.set(pps, offset)
  return output
}

function deriveAvcCodecString(sps: Uint8Array) {
  if (sps.length < 4) return 'avc1.42E01F'
  const profile = sps[1].toString(16).padStart(2, '0').toUpperCase()
  const constraints = sps[2].toString(16).padStart(2, '0').toUpperCase()
  const level = sps[3].toString(16).padStart(2, '0').toUpperCase()
  return `avc1.${profile}${constraints}${level}`
}

function normalizeCodecString(codec: string | null | undefined, sps: Uint8Array) {
  if (!codec) return deriveAvcCodecString(sps)
  if (/^avc1\./i.test(codec)) {
    return `avc1.${codec.slice(5)}`
  }
  return deriveAvcCodecString(sps)
}

function buildDecoderCandidates(message: VideoConfigMessage) {
  const codec = normalizeCodecString(message.codec, message.sps)
  const baseConfig: VideoDecoderConfig = {
    codec,
    hardwareAcceleration: 'prefer-hardware',
  }
  if (message.width > 0) {
    baseConfig.codedWidth = message.width
  }
  if (message.height > 0) {
    baseConfig.codedHeight = message.height
  }

  return [
    {
      config: {
        ...baseConfig,
        description: buildAvcDescription(message),
      },
      payloadFormat: 'avcc' as H264PayloadFormat,
    },
    {
      config: baseConfig,
      payloadFormat: 'annexb' as H264PayloadFormat,
    },
  ]
}

async function selectDecoderConfiguration(message: VideoConfigMessage) {
  const candidates = buildDecoderCandidates(message)
  if (typeof VideoDecoder.isConfigSupported !== 'function') {
    return candidates[0]
  }

  for (const candidate of candidates) {
    try {
      const result = await VideoDecoder.isConfigSupported(candidate.config)
      if (result.supported) {
        return candidate
      }
    } catch {
      continue
    }
  }

  return null
}

const VideoStreamCard = ({ stream, videoWidth, videoHeight }: VideoStreamCardProps) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const decoderRef = useRef<VideoDecoder | null>(null)
  const decoderConfigRef = useRef<VideoDecoderConfig | null>(null)
  const payloadFormatRef = useRef<H264PayloadFormat>('avcc')
  const [status, setStatus] = useState(
    stream.available ? 'Waiting for codec config...' : 'Waiting for video ingest...'
  )
  const [hasFrame, setHasFrame] = useState(false)

  const videoStyle = useMemo(
    () =>
      ({
        ...(videoWidth ? { '--video-width': `${videoWidth}px` } : {}),
        ...(videoWidth && videoHeight ? { '--video-aspect': `${videoWidth} / ${videoHeight}` } : {}),
      }) as CSSProperties,
    [videoHeight, videoWidth]
  )

  useEffect(() => {
    let active = true

    if (typeof VideoDecoder === 'undefined') {
      setStatus('WebCodecs is unavailable in this browser')
      return
    }

    const decoder = new VideoDecoder({
      output: (frame) => {
        const canvas = canvasRef.current
        if (!canvas) {
          frame.close()
          return
        }

        const width = frame.displayWidth || stream.width || 640
        const height = frame.displayHeight || stream.height || 360
        if (canvas.width !== width || canvas.height !== height) {
          canvas.width = width
          canvas.height = height
        }

        const context = canvas.getContext('2d')
        if (!context) {
          frame.close()
          return
        }
        context.drawImage(frame, 0, 0, canvas.width, canvas.height)
        frame.close()
        setHasFrame(true)
        setStatus('Live')
      },
      error: (error) => {
        setStatus(`Decoder error: ${error.message}`)
      },
    })
    decoderRef.current = decoder

    const unsubscribe = getVideoGatewayClient().subscribe(stream.stream_id, (message) => {
      if (message.kind === 'config') {
        void (async () => {
          const supported = await selectDecoderConfiguration(message)
          if (!active) return
          if (!supported) {
            decoderConfigRef.current = null
            setStatus('WebCodecs does not support this H.264 stream')
            return
          }

          decoder.reset()
          decoder.configure(supported.config)
          decoderConfigRef.current = supported.config
          payloadFormatRef.current = supported.payloadFormat
          setStatus('Waiting for keyframe...')
        })()
        return
      }

      const chunk = message as VideoChunkMessage
      if (!decoderConfigRef.current) {
        return
      }
      if (decoder.decodeQueueSize > 3 && !chunk.key) {
        return
      }
      if (decoder.decodeQueueSize > 5 && chunk.key) {
        decoder.reset()
        decoder.configure(decoderConfigRef.current)
      }

      const payload =
        payloadFormatRef.current === 'annexb' ? chunk.payload : annexBToAvcc(chunk.payload)
      decoder.decode(
        new EncodedVideoChunk({
          type: chunk.key ? 'key' : 'delta',
          timestamp: chunk.timestampUs,
          data: payload,
        })
      )
    })

    return () => {
      active = false
      unsubscribe()
      decoder.close()
      decoderRef.current = null
      decoderConfigRef.current = null
      payloadFormatRef.current = 'avcc'
    }
  }, [stream.available, stream.height, stream.stream_id, stream.width])

  return (
    <div className="video-stream-card" style={videoStyle}>
      <canvas ref={canvasRef} className="video-stream-canvas" />
      <div className={`video-stream-overlay ${hasFrame ? 'video-stream-overlay--live' : ''}`}>
        {status}
      </div>
    </div>
  )
}

export default VideoStreamCard
