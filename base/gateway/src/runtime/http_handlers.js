'use strict'

// HTTP surface for small base-side utilities. The main command/telemetry path
// remains WebSocket plus XBEE, so these handlers stay intentionally narrow.
function handleRocketM2Status(_req, res, options = {}) {
  const status = options.status || null
  const statuses = Array.isArray(options.statuses) ? options.statuses : null
  const enabled = options.enabled === true
  const configured = options.configured === true

  if (!enabled) {
    res.writeHead(503, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'Rocket M2 disabled' }))
    return
  }
  if (!configured) {
    res.writeHead(500, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'Rocket M2 missing configuration' }))
    return
  }
  if (statuses) {
    if (statuses.length === 0) {
      res.writeHead(503, { 'Content-Type': 'application/json' })
      res.end(JSON.stringify({ error: 'Rocket M2 status not ready' }))
      return
    }

    res.writeHead(200, {
      'Content-Type': 'application/json',
      'Cache-Control': 'no-store',
    })
    res.end(
      JSON.stringify({
        type: 'rocket_m2_statuses',
        statuses,
      })
    )
    return
  }

  if (!status) {
    res.writeHead(503, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'Rocket M2 status not ready' }))
    return
  }

  res.writeHead(200, {
    'Content-Type': 'application/json',
    'Cache-Control': 'no-store',
  })
  res.end(
    JSON.stringify({
      type: 'rocket_m2_status',
      ...status,
    })
  )
}

function createGatewayHttpHandler(options = {}) {
  const getRocketM2State =
    typeof options.getRocketM2State === 'function'
      ? options.getRocketM2State
      : () => ({ enabled: false, configured: false, status: null })
  const getVideoStreams =
    typeof options.getVideoStreams === 'function' ? options.getVideoStreams : () => []

  return function gatewayHttpHandler(req, res) {
    if (req.method === 'GET' && req.url && req.url.startsWith('/rocket-m2/status')) {
      const rocketM2State = getRocketM2State()
      handleRocketM2Status(req, res, rocketM2State)
      return
    }

    if (req.method === 'GET' && req.url && req.url.startsWith('/video/streams')) {
      res.writeHead(200, {
        'Cache-Control': 'no-store',
        'Content-Type': 'application/json',
      })
      res.end(JSON.stringify({ streams: getVideoStreams() }))
      return
    }

    res.writeHead(404)
    res.end()
  }
}

module.exports = {
  createGatewayHttpHandler,
  handleRocketM2Status,
}
