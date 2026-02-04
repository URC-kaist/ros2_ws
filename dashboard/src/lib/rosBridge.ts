type MessageHandler<T> = (message: T) => void

type SubscribeOptions = {
  throttleRate?: number
  queueSize?: number
  compression?: 'none' | 'png' | 'cbor'
}

type TopicEntry<T> = {
  topic: ROSLIB.Topic<T>
  handlers: Set<MessageHandler<T>>
}

const DEFAULT_PATH = '/rosbridge-ws'
const RECONNECT_BASE_MS = 500
const RECONNECT_MAX_MS = 5000

const resolveRosBridgeUrl = () => {
  const explicit = import.meta.env.VITE_ROSBRIDGE_URL as string | undefined
  if (explicit) return explicit
  if (typeof window === 'undefined') {
    return `ws://localhost:9090`
  }
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${protocol}//${window.location.host}${DEFAULT_PATH}`
}

class RosBridgeClient {
  private ros: ROSLIB.Ros
  private url: string
  private connected = false
  private connecting = false
  private reconnectTimer: number | null = null
  private reconnectDelayMs = RECONNECT_BASE_MS
  private connectionListeners = new Set<MessageHandler<boolean>>()
  private topics = new Map<string, TopicEntry<unknown>>()

  constructor(url: string) {
    this.url = url
    const roslib = this.getRosLib()
    this.ros = new roslib.Ros()
    this.ros.on('connection', () => {
      this.connected = true
      this.connecting = false
      this.reconnectDelayMs = RECONNECT_BASE_MS
      this.emitConnectionStatus(true)
      this.resubscribeAll()
    })
    this.ros.on('close', () => {
      this.connected = false
      this.connecting = false
      this.emitConnectionStatus(false)
      this.scheduleReconnect()
    })
    this.ros.on('error', () => {
      this.connected = false
      this.connecting = false
      this.emitConnectionStatus(false)
      this.scheduleReconnect()
    })
  }

  connect() {
    if (this.connected || this.connecting) return
    this.connecting = true
    this.clearReconnectTimer()
    this.ros.connect(this.url)
  }

  isConnected() {
    return this.connected
  }

  onConnectionStatus(handler: MessageHandler<boolean>) {
    this.connectionListeners.add(handler)
    return () => this.connectionListeners.delete(handler)
  }

  subscribe<T>(
    name: string,
    messageType: string,
    handler: MessageHandler<T>,
    options: SubscribeOptions = {}
  ) {
    const entry = this.getOrCreateTopic<T>(name, messageType, options)
    entry.handlers.add(handler)
    if (this.connected) {
      entry.topic.subscribe(handler)
    }
    return () => {
      entry.handlers.delete(handler)
      this.safeTopicUnsubscribe(entry, handler)
      if (entry.handlers.size === 0) {
        this.safeTopicUnsubscribe(entry)
        this.topics.delete(this.topicKey(name, messageType, options))
      }
    }
  }

  publish<T>(
    name: string,
    messageType: string,
    message: T,
    options: SubscribeOptions = {}
  ) {
    const entry = this.getOrCreateTopic<T>(name, messageType, options)
    entry.topic.publish(message)
  }

  callService<TRequest, TResponse>(
    name: string,
    serviceType: string,
    request: TRequest
  ): Promise<TResponse> {
    if (!this.connected) {
      return Promise.reject(new Error('ROS bridge is not connected'))
    }
    const roslib = this.getRosLib()
    const service = new roslib.Service({
      ros: this.ros,
      name,
      serviceType,
    })
    const serviceRequest = new roslib.ServiceRequest(request as Record<string, unknown>)
    return new Promise((resolve, reject) => {
      service.callService(
        serviceRequest,
        (response: TResponse) => resolve(response),
        (error?: unknown) => {
          if (error instanceof Error) {
            reject(error)
          } else if (typeof error === 'string') {
            reject(new Error(error))
          } else {
            reject(new Error('Service call failed'))
          }
        }
      )
    })
  }

  private emitConnectionStatus(connected: boolean) {
    for (const handler of this.connectionListeners) {
      handler(connected)
    }
  }

  private getOrCreateTopic<T>(
    name: string,
    messageType: string,
    options: SubscribeOptions
  ): TopicEntry<T> {
    const key = this.topicKey(name, messageType, options)
    const existing = this.topics.get(key) as TopicEntry<T> | undefined
    if (existing) return existing
    const roslib = this.getRosLib()
    const topic = new roslib.Topic<T>({
      ros: this.ros,
      name,
      messageType,
      throttle_rate: options.throttleRate,
      queue_size: options.queueSize,
      compression: options.compression,
    })
    const entry: TopicEntry<T> = { topic, handlers: new Set() }
    this.topics.set(key, entry as TopicEntry<unknown>)
    return entry
  }

  private getRosLib() {
    if (typeof window === 'undefined' || !window.ROSLIB) {
      throw new Error('ROSLIB not available. Ensure /roslib.min.js is loaded.')
    }
    return window.ROSLIB
  }

  private topicKey(name: string, messageType: string, options: SubscribeOptions) {
    return JSON.stringify({
      name,
      messageType,
      throttleRate: options.throttleRate ?? null,
      queueSize: options.queueSize ?? null,
      compression: options.compression ?? null,
    })
  }

  private resubscribeAll() {
    for (const entry of this.topics.values()) {
      this.safeTopicUnsubscribe(entry)
      for (const handler of entry.handlers) {
        entry.topic.subscribe(handler)
      }
    }
  }

  /**
   * roslib's Topic.unsubscribe expects its internal EventEmitter
   * state (`_events`) to exist. If we call it before any subscribe()
   * was made (e.g., when ROS is offline and we clean up), it throws.
   */
  private safeTopicUnsubscribe(
    entry: TopicEntry<unknown>,
    handler?: MessageHandler<unknown>
  ) {
    const topicAny = entry.topic as any
    const hasEvents = !!topicAny?._events
    if (!hasEvents) return
    if (handler) {
      entry.topic.unsubscribe(handler)
    } else {
      entry.topic.unsubscribe()
    }
  }

  private scheduleReconnect() {
    if (typeof window === 'undefined') return
    if (this.reconnectTimer != null) return
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null
      this.connect()
      this.reconnectDelayMs = Math.min(
        this.reconnectDelayMs * 2,
        RECONNECT_MAX_MS
      )
    }, this.reconnectDelayMs)
  }

  private clearReconnectTimer() {
    if (this.reconnectTimer == null) return
    window.clearTimeout(this.reconnectTimer)
    this.reconnectTimer = null
  }
}

let singleton: RosBridgeClient | null = null

export const getRosBridgeClient = () => {
  if (!singleton) {
    singleton = new RosBridgeClient(resolveRosBridgeUrl())
  }
  return singleton
}
