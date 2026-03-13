'use strict'

const { WebSocketServer } = require('ws')

function createWsHub(options = {}) {
  const onMessage = typeof options.onMessage === 'function' ? options.onMessage : () => {}
  const getInitialMessages =
    typeof options.getInitialMessages === 'function' ? options.getInitialMessages : () => []

  const wss = new WebSocketServer({ server: options.server })
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
    broadcast(obj) {
      const payload = JSON.stringify(obj)
      for (const client of wss.clients) {
        if (client.readyState === 1) {
          client.send(payload)
        }
      }
    },
    close() {
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
