'use strict'

function resolvePath(request) {
  try {
    return new URL(request.url || '/', 'http://localhost').pathname
  } catch {
    return '/'
  }
}

function rejectUpgrade(socket) {
  socket.write('HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n')
  socket.destroy()
}

function createWsRouteRegistry(options = {}) {
  const server = options.server
  if (!server || typeof server.on !== 'function') {
    throw new Error('createWsRouteRegistry requires an HTTP server')
  }

  const routes = new Map()

  server.on('upgrade', (request, socket, head) => {
    const pathname = resolvePath(request)
    const entry = routes.get(pathname)
    if (!entry) {
      rejectUpgrade(socket)
      return
    }

    entry.wss.handleUpgrade(request, socket, head, (ws) => {
      entry.wss.emit('connection', ws, request)
    })
  })

  return {
    register(pathname, wss) {
      if (!pathname || typeof pathname !== 'string') {
        throw new Error('WebSocket route path must be a non-empty string')
      }
      if (!wss || typeof wss.handleUpgrade !== 'function') {
        throw new Error('WebSocket route requires a WebSocketServer instance')
      }
      if (routes.has(pathname)) {
        throw new Error(`WebSocket route already registered: ${pathname}`)
      }
      routes.set(pathname, { wss })
    },
    unregister(pathname) {
      routes.delete(pathname)
    },
    close() {
      routes.clear()
    },
  }
}

module.exports = {
  createWsRouteRegistry,
}
