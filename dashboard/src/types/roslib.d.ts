export {}

declare global {
  interface Window {
    ROSLIB?: typeof ROSLIB
  }
}

declare namespace ROSLIB {
  type ConnectionEvent = 'connection' | 'close' | 'error'

  class Ros {
    constructor(options?: { url?: string })
    connect(url: string): void
    close(): void
    on(event: ConnectionEvent, callback: (event?: unknown) => void): void
  }

  interface TopicOptions {
    ros: Ros
    name: string
    messageType: string
    throttle_rate?: number
    queue_size?: number
    compression?: 'none' | 'png' | 'cbor'
  }

  class Topic<T = unknown> {
    constructor(options: TopicOptions)
    subscribe(callback: (message: T) => void): void
    unsubscribe(callback?: (message: T) => void): void
    publish(message: T): void
  }
}
