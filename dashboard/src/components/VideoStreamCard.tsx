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
  const configGenerationRef = useRef(0)
  const waitingForKeyframeRef = useRef(true)
  const [status, setStatus] = useState(
    stream.available ? 'Waiting for codec config...' : 'Waiting for video ingest...'
  )
  const [hasFrame, setHasFrame] = useState(false)
  const [frameSize, setFrameSize] = useState<{ width: number; height: number } | null>(null)

  const videoStyle = useMemo(
    () => {
      const width = frameSize?.width ?? videoWidth
      const height = frameSize?.height ?? videoHeight
      return ({
        ...(width && height ? { '--video-aspect': `${width} / ${height}` } : {}),
      }) as CSSProperties
    },
    [frameSize, videoHeight, videoWidth]
  )

  useEffect(() => {
    let active = true
    let decoderGeneration = 0

    if (typeof VideoDecoder === 'undefined') {
      setStatus('WebCodecs is unavailable in this browser')
      return
    }

    function describeError(error: unknown) {
      return error instanceof Error ? error.message : String(error)
    }

    function closeDecoder(decoder: VideoDecoder | null) {
      if (!decoder) return
      try {
        decoder.close()
      } catch {
        // Ignore repeated-close and invalid-state cleanup paths.
      }
    }

    function createDecoder() {
      decoderGeneration += 1
      const generation = decoderGeneration
      const decoder = new VideoDecoder({
        output: (frame) => {
          if (!active || generation !== decoderGeneration) {
            frame.close()
            return
          }

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
            setFrameSize({ width, height })
          }

          const context = canvas.getContext('2d')
          if (!context) {
            frame.close()
            return
          }
          context.fillStyle = '#060910'
          context.fillRect(0, 0, canvas.width, canvas.height)
          const frameRatio = width / height
          const canvasRatio = canvas.width / canvas.height
          const drawWidth = frameRatio > canvasRatio ? canvas.width : canvas.height * frameRatio
          const drawHeight = frameRatio > canvasRatio ? canvas.width / frameRatio : canvas.height
          const offsetX = (canvas.width - drawWidth) * 0.5
          const offsetY = (canvas.height - drawHeight) * 0.5
          context.drawImage(frame, offsetX, offsetY, drawWidth, drawHeight)
          frame.close()
          waitingForKeyframeRef.current = false
          setHasFrame(true)
          setStatus('Live')
        },
        error: (error) => {
          if (!active || generation !== decoderGeneration) return
          recoverDecoder(`Decoder error: ${error.message}`)
        },
      })
      decoderRef.current = decoder
      return decoder
    }

    function recoverDecoder(reason: string) {
      waitingForKeyframeRef.current = true
      const decoderConfig = decoderConfigRef.current
      const payloadFormat = payloadFormatRef.current
      closeDecoder(decoderRef.current)
      decoderRef.current = null

      if (!decoderConfig) {
        setStatus(reason)
        return
      }

      try {
        const recovered = createDecoder()
        recovered.configure(decoderConfig)
        payloadFormatRef.current = payloadFormat
        setStatus(`${reason}; waiting for keyframe...`)
      } catch (recoveryError) {
        decoderConfigRef.current = null
        setStatus(`Decoder error: ${describeError(recoveryError)}`)
      }
    }

    createDecoder()

    const unsubscribe = getVideoGatewayClient().subscribe(stream.stream_id, (message) => {
      if (message.kind === 'config') {
        const generation = configGenerationRef.current + 1
        configGenerationRef.current = generation
        decoderConfigRef.current = null
        waitingForKeyframeRef.current = true
        setStatus('Configuring decoder...')

        void (async () => {
          const supported = await selectDecoderConfiguration(message)
          if (!active || generation !== configGenerationRef.current) return
          if (!supported) {
            decoderConfigRef.current = null
            setStatus('WebCodecs does not support this H.264 stream')
            return
          }

          try {
            const decoder =
              decoderRef.current && decoderRef.current.state !== 'closed'
                ? decoderRef.current
                : createDecoder()
            decoder.reset()
            decoder.configure(supported.config)
            decoderConfigRef.current = supported.config
            payloadFormatRef.current = supported.payloadFormat
            waitingForKeyframeRef.current = true
            setStatus('Waiting for keyframe...')
          } catch (error) {
            decoderConfigRef.current = null
            setStatus(`Decoder error: ${describeError(error)}`)
          }
        })()
        return
      }

      const chunk = message as VideoChunkMessage
      const decoderConfig = decoderConfigRef.current
      const decoder = decoderRef.current
      if (!decoderConfig || !decoder) {
        return
      }
      if (waitingForKeyframeRef.current && !chunk.key) {
        return
      }
      if (decoder.decodeQueueSize > 3 && !chunk.key) {
        return
      }
      if (decoder.decodeQueueSize > 5 && chunk.key) {
        try {
          decoder.reset()
          decoder.configure(decoderConfig)
        } catch (error) {
          recoverDecoder(`Decoder error: ${describeError(error)}`)
          return
        }
      }

      const payload =
        payloadFormatRef.current === 'annexb' ? chunk.payload : annexBToAvcc(chunk.payload)
      if (chunk.key) {
        waitingForKeyframeRef.current = false
      }
      try {
        decoder.decode(
          new EncodedVideoChunk({
            type: chunk.key ? 'key' : 'delta',
            timestamp: chunk.timestampUs,
            data: payload,
          })
        )
      } catch (error) {
        recoverDecoder(`Decoder error: ${describeError(error)}`)
      }
    })

    return () => {
      active = false
      unsubscribe()
      closeDecoder(decoderRef.current)
      decoderRef.current = null
      decoderConfigRef.current = null
      payloadFormatRef.current = 'avcc'
      waitingForKeyframeRef.current = true
    }
  }, [stream.stream_id])

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
