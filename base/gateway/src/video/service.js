'use strict'

const { WebSocketServer } = require('ws')

const { encodeChunkMessage, encodeConfigMessage } = require('./protocol')
const { createVideoStreamReceiver } = require('./receiver')
const { listBrowserStreams } = require('./stream_config')

function buffersEqual(left, right) {
  if (!left && !right) return true
  if (!left || !right) return false
  return Buffer.compare(left, right) === 0
}

function createStreamState(stream) {
  return {
    available: false,
    codec: null,
    configVersion: 0,
    encodedHeight: 0,
    encodedWidth: 0,
    latestKeyAccessUnit: null,
    pps: null,
    sps: null,
    stream,
  }
}

function createVideoGateway(options = {}) {
  const log = typeof options.log === 'function' ? options.log : () => {}
  const path = options.path || '/video-ws'
  const routeRegistry = options.routeRegistry
  const videoConfig = options.videoConfig

  if (!routeRegistry || typeof routeRegistry.register !== 'function') {
    throw new Error('createVideoGateway requires a routeRegistry')
  }
  if (!videoConfig || !Array.isArray(videoConfig.streams)) {
    throw new Error('createVideoGateway requires a loaded videoConfig')
  }

  const clientMaxBufferedBytes = Math.max(
    4096,
    Math.floor(options.clientMaxBufferedBytes || 1048576)
  )
  const WebSocketServerImpl = options.WebSocketServerImpl || WebSocketServer
  const createVideoStreamReceiverImpl =
    options.createVideoStreamReceiverImpl || createVideoStreamReceiver
  const spawnImpl = options.spawnImpl

  const streamStates = new Map(
    videoConfig.streams.map((stream) => [stream.stream_id, createStreamState(stream)])
  )
  const subscriptionsByStream = new Map(
    videoConfig.streams.map((stream) => [stream.stream_id, new Set()])
  )
  const clients = new Set()
  const wss = new WebSocketServerImpl({ noServer: true })

  routeRegistry.register(path, wss)

  const receivers = videoConfig.streams.map((stream) =>
    createVideoStreamReceiverImpl({
      availabilityStaleMs: options.availabilityStaleMs,
      gstBinary: options.gstBinary,
      idleFlushMs: options.idleFlushMs,
      jitterLatencyMs: options.jitterLatencyMs,
      log,
      onAccessUnit: handleAccessUnit,
      onAvailabilityChange: handleAvailabilityChange,
      restartMs: options.restartMs,
      spawnImpl,
      stream,
    })
  )

  wss.on('connection', (ws) => {
    const client = {
      subscriptions: new Map(),
      ws,
    }
    clients.add(client)

    ws.on('message', (data) => {
      handleClientMessage(client, data)
    })

    ws.on('close', () => {
      removeClient(client)
    })
  })

  function handleAvailabilityChange(streamId, available) {
    const state = streamStates.get(streamId)
    if (!state) return
    state.available = available
    if (!available) {
      state.latestKeyAccessUnit = null
      state.configVersion += 1
      for (const client of subscriptionsByStream.get(streamId) || []) {
        const subscription = client.subscriptions.get(streamId)
        if (subscription) {
          subscription.needsBootstrap = true
        }
      }
    }
  }

  function updateCodecState(state, accessUnit) {
    let changed = false

    if (accessUnit.sps && !buffersEqual(state.sps, accessUnit.sps)) {
      state.sps = Buffer.from(accessUnit.sps)
      changed = true
    }

    if (accessUnit.pps && !buffersEqual(state.pps, accessUnit.pps)) {
      state.pps = Buffer.from(accessUnit.pps)
      changed = true
    }

    if (accessUnit.codec && state.codec !== accessUnit.codec) {
      state.codec = accessUnit.codec
      changed = true
    }

    if (
      Number.isInteger(accessUnit.width) &&
      accessUnit.width > 0 &&
      state.encodedWidth !== accessUnit.width
    ) {
      state.encodedWidth = accessUnit.width
      changed = true
    }

    if (
      Number.isInteger(accessUnit.height) &&
      accessUnit.height > 0 &&
      state.encodedHeight !== accessUnit.height
    ) {
      state.encodedHeight = accessUnit.height
      changed = true
    }

    if (changed && state.sps && state.pps) {
      state.latestKeyAccessUnit = null
      state.configVersion += 1
      for (const client of subscriptionsByStream.get(state.stream.stream_id) || []) {
        const subscription = client.subscriptions.get(state.stream.stream_id)
        if (subscription) {
          subscription.needsBootstrap = true
        }
      }
    }
  }

  function trySend(client, payload, streamId) {
    if (client.ws.readyState !== 1) return false
    const subscription = client.subscriptions.get(streamId)
    if (!subscription) return false
    if (client.ws.bufferedAmount > clientMaxBufferedBytes) {
      subscription.needsBootstrap = true
      return false
    }
    client.ws.send(payload)
    return true
  }

  function maybeSendBootstrap(client, state, subscription, keyAccessUnit) {
    if (!state.sps || !state.pps) return false
    const keyUnit = keyAccessUnit || state.latestKeyAccessUnit
    if (!keyUnit) return false

    if (subscription.configVersionSent !== state.configVersion) {
      const configMessage = encodeConfigMessage({
        codec: state.codec || '',
        height: state.encodedHeight,
        pps: state.pps,
        sps: state.sps,
        stream_id: state.stream.stream_id,
        width: state.encodedWidth,
      })
      if (!trySend(client, configMessage, state.stream.stream_id)) {
        return false
      }
      subscription.configVersionSent = state.configVersion
    }

    const chunkMessage = encodeChunkMessage({
      key: true,
      payload: keyUnit.payload,
      stream_id: state.stream.stream_id,
      timestamp_us: keyUnit.timestamp_us,
    })
    if (!trySend(client, chunkMessage, state.stream.stream_id)) {
      return false
    }

    subscription.needsBootstrap = false
    return true
  }

  function dispatchAccessUnit(state, accessUnit) {
    const subscribers = subscriptionsByStream.get(state.stream.stream_id)
    if (!subscribers || subscribers.size === 0) return

    for (const client of subscribers) {
      const subscription = client.subscriptions.get(state.stream.stream_id)
      if (!subscription) continue

      if (subscription.needsBootstrap || subscription.configVersionSent !== state.configVersion) {
        if (!accessUnit.key) {
          maybeSendBootstrap(client, state, subscription, null)
          continue
        }
        maybeSendBootstrap(client, state, subscription, accessUnit)
        continue
      }

      trySend(
        client,
        encodeChunkMessage({
          key: accessUnit.key,
          payload: accessUnit.payload,
          stream_id: state.stream.stream_id,
          timestamp_us: accessUnit.timestamp_us,
        }),
        state.stream.stream_id
      )
    }
  }

  function handleAccessUnit(streamId, accessUnit) {
    const state = streamStates.get(streamId)
    if (!state) return

    updateCodecState(state, accessUnit)

    if (!accessUnit.key && !accessUnit.delta) {
      return
    }

    if (accessUnit.key && state.sps && state.pps) {
      state.latestKeyAccessUnit = {
        payload: Buffer.from(accessUnit.payload),
        timestamp_us: accessUnit.timestamp_us,
      }
    }

    dispatchAccessUnit(state, accessUnit)
  }

  function subscribeClient(client, streamId) {
    const state = streamStates.get(streamId)
    if (!state) return
    if (client.subscriptions.has(streamId)) return

    const subscription = {
      configVersionSent: 0,
      needsBootstrap: true,
    }
    client.subscriptions.set(streamId, subscription)
    subscriptionsByStream.get(streamId).add(client)
    maybeSendBootstrap(client, state, subscription, null)
  }

  function unsubscribeClient(client, streamId) {
    client.subscriptions.delete(streamId)
    const subscribers = subscriptionsByStream.get(streamId)
    if (subscribers) {
      subscribers.delete(client)
    }
  }

  function removeClient(client) {
    for (const streamId of client.subscriptions.keys()) {
      unsubscribeClient(client, streamId)
    }
    clients.delete(client)
  }

  function handleClientMessage(client, data) {
    let message
    try {
      message = JSON.parse(Buffer.isBuffer(data) ? data.toString('utf8') : String(data))
    } catch {
      return
    }

    if (message.type === 'subscribe' && typeof message.stream_id === 'string') {
      subscribeClient(client, message.stream_id)
      return
    }

    if (message.type === 'unsubscribe' && typeof message.stream_id === 'string') {
      unsubscribeClient(client, message.stream_id)
    }
  }

  return {
    getBrowserStreams() {
      return listBrowserStreams(videoConfig).map((stream) => {
        const state = streamStates.get(stream.stream_id)
        const encodedWidth = state && state.encodedWidth > 0 ? state.encodedWidth : null
        const encodedHeight = state && state.encodedHeight > 0 ? state.encodedHeight : null
        return {
          ...stream,
          width: stream.width || encodedWidth,
          height: stream.height || encodedHeight,
          encoded_width: encodedWidth,
          encoded_height: encodedHeight,
          available: state?.available === true,
        }
      })
    },
    start() {
      for (const receiver of receivers) {
        receiver.start()
      }
    },
    stop() {
      routeRegistry.unregister(path)
      for (const receiver of receivers) {
        receiver.stop()
      }
      for (const client of clients) {
        client.ws.terminate()
      }
      wss.close()
    },
  }
}

module.exports = {
  createVideoGateway,
}
