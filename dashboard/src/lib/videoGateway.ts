import { decodeVideoMessage, type VideoGatewayMessage } from './videoProtocol'
import { browserEpochUs, latencyDiagnostics } from './latencyDiagnostics'

export type VideoStreamInfo = {
  stream_id: string
  source_type: 'ros_topic' | 'v4l2'
  ros_topic: string | null
  udp_port: number
  ros_encoding: string | null
  v4l2_device: string | null
  v4l2_pixel_format: string | null
  width: number | null
  height: number | null
  encoded_width?: number | null
  encoded_height?: number | null
  framerate: number | null
  available: boolean
  display: {
    label?: string
    panel?: string
    order?: number
  }
}

type StreamListener = (message: VideoGatewayMessage) => void

const RECONNECT_BASE_MS = 500
const RECONNECT_MAX_MS = 5000

function resolveVideoGatewayUrl() {
  const explicit = import.meta.env.VITE_VIDEO_WS_URL as string | undefined
  if (explicit) return explicit
  if (typeof window === 'undefined') {
    return 'ws://localhost:8081/video-ws'
  }
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${protocol}//${window.location.host}/video-ws`
}

function resolveVideoStreamsUrl() {
  const explicit = import.meta.env.VITE_VIDEO_STREAMS_URL as string | undefined
  if (explicit) return explicit
  if (typeof window === 'undefined') {
    return 'http://localhost:8081/video/streams'
  }
  return `${window.location.origin}/video/streams`
}

class VideoGatewayClient {
  private ws: WebSocket | null = null
  private readonly url: string
  private reconnectTimer: number | null = null
  private reconnectDelayMs = RECONNECT_BASE_MS
  private listeners = new Map<string, Set<StreamListener>>()
  private subscribedStreams = new Set<string>()

  constructor(url: string) {
    this.url = url
  }

  subscribe(streamId: string, listener: StreamListener) {
    let streamListeners = this.listeners.get(streamId)
    if (!streamListeners) {
      streamListeners = new Set()
      this.listeners.set(streamId, streamListeners)
    }
    streamListeners.add(listener)
    this.connect()
    if (streamListeners.size === 1) {
      this.sendSubscription('subscribe', streamId)
    }

    return () => {
      const current = this.listeners.get(streamId)
      if (!current) return
      current.delete(listener)
      if (current.size === 0) {
        this.listeners.delete(streamId)
        this.sendSubscription('unsubscribe', streamId)
      }
      if (this.listeners.size === 0) {
        this.disconnect()
      }
    }
  }

  reconnect() {
    this.clearReconnectTimer()
    this.reconnectDelayMs = RECONNECT_BASE_MS
    this.subscribedStreams.clear()

    if (this.ws) {
      const ws = this.ws
      this.ws = null
      ws.close()
    }

    if (this.listeners.size > 0) {
      this.connect()
    }
  }

  private connect() {
    if (this.ws) return
    this.clearReconnectTimer()

    const ws = new WebSocket(this.url)
    ws.binaryType = 'arraybuffer'
    ws.addEventListener('open', () => {
      if (this.ws !== ws) return
      this.reconnectDelayMs = RECONNECT_BASE_MS
      this.subscribedStreams.clear()
      for (const streamId of this.listeners.keys()) {
        this.sendSubscription('subscribe', streamId)
      }
    })
    ws.addEventListener('message', (event) => {
      const browserReceiveEpochUs = browserEpochUs()
      if (this.ws !== ws) return
      if (!(event.data instanceof ArrayBuffer)) return
      const message = decodeVideoMessage(event.data)
      if (!message) return
      if (message.kind === 'chunk') {
        message.browserReceiveEpochUs = browserReceiveEpochUs
        latencyDiagnostics.observeVideoReceive(
          message.streamId,
          message.baseIngestTimestampUs,
          browserReceiveEpochUs
        )
      }
      const streamListeners = this.listeners.get(message.streamId)
      if (!streamListeners) return
      for (const listener of streamListeners) {
        listener(message)
      }
    })
    ws.addEventListener('close', () => {
      if (this.ws !== ws) return
      this.ws = null
      this.subscribedStreams.clear()
      if (this.listeners.size > 0) {
        this.scheduleReconnect()
      }
    })
    ws.addEventListener('error', () => {
      if (this.ws !== ws) return
      ws.close()
    })

    this.ws = ws
  }

  private disconnect() {
    this.clearReconnectTimer()
    if (!this.ws) return
    this.ws.close()
    this.ws = null
    this.subscribedStreams.clear()
  }

  private scheduleReconnect() {
    if (this.reconnectTimer != null) return
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null
      this.connect()
      this.reconnectDelayMs = Math.min(this.reconnectDelayMs * 2, RECONNECT_MAX_MS)
    }, this.reconnectDelayMs)
  }

  private clearReconnectTimer() {
    if (this.reconnectTimer == null) return
    window.clearTimeout(this.reconnectTimer)
    this.reconnectTimer = null
  }

  private sendSubscription(type: 'subscribe' | 'unsubscribe', streamId: string) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return
    if (type === 'subscribe' && this.subscribedStreams.has(streamId)) return
    if (type === 'subscribe') {
      this.subscribedStreams.add(streamId)
    } else {
      this.subscribedStreams.delete(streamId)
    }
    this.ws.send(
      JSON.stringify({
        type,
        stream_id: streamId,
      })
    )
  }
}

let gatewayClient: VideoGatewayClient | null = null

export function getVideoGatewayClient() {
  if (!gatewayClient) {
    gatewayClient = new VideoGatewayClient(resolveVideoGatewayUrl())
  }
  return gatewayClient
}

export async function fetchVideoStreams() {
  const response = await fetch(resolveVideoStreamsUrl(), {
    cache: 'no-store',
  })
  if (!response.ok) {
    throw new Error(`Failed to load video streams (${response.status})`)
  }
  const body = (await response.json()) as { streams?: VideoStreamInfo[] }
  return Array.isArray(body.streams) ? body.streams : []
}
