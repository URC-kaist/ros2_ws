'use strict'

// Minimal WebSocket wrapper for dashboard clients. Keep message handling
// centralized in the app layer rather than burying business logic here.
function createWsHub(options = {}) {
  const onMessage = typeof options.onMessage === 'function' ? options.onMessage : () => {}
  const getInitialMessages =
    typeof options.getInitialMessages === 'function' ? options.getInitialMessages : () => []
  const routeRegistry = options.routeRegistry
  const path = options.path || '/sik-ws'
  const WebSocketServerImpl = options.WebSocketServerImpl || require('ws').WebSocketServer

  if (!routeRegistry || typeof routeRegistry.register !== 'function') {
    throw new Error('createWsHub requires a routeRegistry')
  }

  const wss = new WebSocketServerImpl({ noServer: true })
  routeRegistry.register(path, wss)
  wss.on('connection', (ws) => {
    ws.on('message', (data) => {
      let message
      try {
        message = JSON.parse(data.toString())
      } catch {
        return
      }
      onMessage(message)
    })

    for (const message of getInitialMessages()) {
      ws.send(JSON.stringify(message))
    }
  })

  return {
    wss,
    broadcast(obj) {
      const payload = JSON.stringify(obj)
      for (const client of wss.clients) {
        if (client.readyState === 1) {
          client.send(payload)
        }
      }
    },
    close() {
      routeRegistry.unregister(path)
      for (const client of wss.clients) {
        client.terminate()
      }
      wss.close()
    },
  }
}

module.exports = {
  createWsHub,
}
