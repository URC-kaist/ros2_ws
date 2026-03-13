export {}

declare global {
  interface Window {
    ROSLIB?: typeof ROSLIB
    uPlot?: unknown
  }
  namespace ROSLIB {
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

    interface ServiceOptions {
      ros: Ros
      name: string
      serviceType: string
    }

    class ServiceRequest<T = Record<string, unknown>> {
      constructor(values?: T)
    }

    class Service<TRequest = Record<string, unknown>, TResponse = unknown> {
      constructor(options: ServiceOptions)
      callService(
        request: ServiceRequest<TRequest>,
        callback: (response: TResponse) => void,
        failedCallback?: (error?: unknown) => void
      ): void
    }
  }
}
