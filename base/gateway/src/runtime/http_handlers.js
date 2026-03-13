'use strict'

const { toInt } = require('../config')

// HTTP surface for small base-side utilities. The main command/telemetry path
// remains WebSocket plus SiK, so these handlers stay intentionally narrow.
function handleRocketM2Status(_req, res, options = {}) {
  const status = options.status || null
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

function handleTransitiveToken(req, res, options = {}) {
  const env = options.env || process.env
  const log = typeof options.log === 'function' ? options.log : () => {}

  log(`Transitive token request from ${req.socket.remoteAddress || 'unknown'}`)
  const secret = env.TRANSITIVE_JWT_SECRET
  if (!secret) {
    log('TRANSITIVE_JWT_SECRET not set')
    res.writeHead(500, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'TRANSITIVE_JWT_SECRET not set' }))
    return
  }

  const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`)
  const id = url.searchParams.get('id') || env.TRANSITIVE_ID || 'unknown'
  const device = url.searchParams.get('device') || env.TRANSITIVE_DEVICE || 'unknown'
  const capability =
    url.searchParams.get('capability') ||
    env.TRANSITIVE_CAPABILITY ||
    '@transitive-robotics/webrtc-video'
  const userId = url.searchParams.get('userId') || env.TRANSITIVE_USER_ID || 'operator'
  const validity = toInt(url.searchParams.get('validity') || env.TRANSITIVE_VALIDITY || 86400)

  const issuedAt = Math.floor(Date.now() / 1000)
  const payload = {
    id,
    device,
    capability,
    userId,
    validity,
    iat: issuedAt,
  }

  let token
  try {
    const jwt = require('jsonwebtoken')
    token = jwt.sign(payload, secret)
  } catch (err) {
    log(`Failed to sign token: ${err.message || err}`)
    res.writeHead(500, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'Failed to sign token' }))
    return
  }

  res.writeHead(200, { 'Content-Type': 'application/json' })
  res.end(
    JSON.stringify({
      token,
      issued_at: issuedAt,
      validity_sec: validity,
    })
  )
}

function createGatewayHttpHandler(options = {}) {
  const env = options.env || process.env
  const log = typeof options.log === 'function' ? options.log : () => {}
  const getRocketM2State =
    typeof options.getRocketM2State === 'function'
      ? options.getRocketM2State
      : () => ({ enabled: false, configured: false, status: null })

  return function gatewayHttpHandler(req, res) {
    if (req.method === 'GET' && req.url && req.url.startsWith('/transitive/token')) {
      handleTransitiveToken(req, res, { env, log })
      return
    }

    if (req.method === 'GET' && req.url && req.url.startsWith('/rocket-m2/status')) {
      const rocketM2State = getRocketM2State()
      handleRocketM2Status(req, res, rocketM2State)
      return
    }

    res.writeHead(404)
    res.end()
  }
}

module.exports = {
  createGatewayHttpHandler,
  handleRocketM2Status,
  handleTransitiveToken,
}
