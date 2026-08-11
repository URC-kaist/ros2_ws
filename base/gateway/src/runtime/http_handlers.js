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

function sendJson(res, statusCode, body) {
  res.writeHead(statusCode, {
    'Cache-Control': 'no-store',
    'Content-Type': 'application/json',
  })
  res.end(JSON.stringify(body))
}

function readJsonBody(req, maximumBytes = 64 * 1024) {
  return new Promise((resolve, reject) => {
    const chunks = []
    let size = 0
    req.on('data', (chunk) => {
      size += chunk.length
      if (size > maximumBytes) {
        reject(new Error('request body is too large'))
        req.destroy()
        return
      }
      chunks.push(chunk)
    })
    req.once('error', reject)
    req.once('end', () => {
      try {
        const text = Buffer.concat(chunks).toString('utf8')
        resolve(text ? JSON.parse(text) : {})
      } catch (_) {
        reject(new Error('request body is not valid JSON'))
      }
    })
  })
}

async function handleUplinkTrialRequest(req, res, manager, parsedUrl) {
  const parts = parsedUrl.pathname.split('/').filter(Boolean)
  const trialId = parts[3] || null
  const action = parts[4] || null
  try {
    if (req.method === 'POST' && !trialId) {
      sendJson(res, 201, manager.createTrial(await readJsonBody(req)))
      return
    }
    if (req.method === 'POST' && trialId && action === 'start') {
      sendJson(res, 200, manager.startTrial(trialId, await readJsonBody(req)))
      return
    }
    if (req.method === 'GET' && trialId && !action) {
      sendJson(res, 200, manager.getTrial(trialId))
      return
    }
    if (req.method === 'DELETE' && trialId && !action) {
      sendJson(res, 200, await manager.cancelTrial(trialId))
      return
    }
    if (req.method === 'PUT' && trialId &&
        (action === 'rover-metadata' || action === 'rover-capture')) {
      sendJson(
        res,
        200,
        await manager.receiveArtifact(trialId, action, req, req.headers)
      )
      return
    }
    if (req.method === 'POST' && trialId && action === 'browser-samples') {
      const body = await readJsonBody(req, 2 * 1024 * 1024)
      sendJson(res, 200, manager.saveBrowserSamples(trialId, body.samples))
      return
    }
    sendJson(res, 404, { error: 'uplink endpoint not found' })
  } catch (error) {
    const message = error.message || String(error)
    const notFound = message.includes('not found')
    const unavailable = error.code === 'not_configured' || message.includes('another uplink')
    sendJson(res, notFound ? 404 : unavailable ? 503 : 400, {
      error: message,
      code: error.code || 'invalid_request',
    })
  }
}

function createGatewayHttpHandler(options = {}) {
  const getRocketM2State =
    typeof options.getRocketM2State === 'function'
      ? options.getRocketM2State
      : () => ({ enabled: false, configured: false, status: null })
  const getVideoStreams =
    typeof options.getVideoStreams === 'function' ? options.getVideoStreams : () => []
  const getChronyStatus =
    typeof options.getChronyStatus === 'function'
      ? options.getChronyStatus
      : async () => ({
          schema_version: 1,
          role: 'base',
          available: false,
          synchronized: false,
          error: 'chrony status provider is not configured',
        })
  const uplinkTrialManager = options.uplinkTrialManager || null

  return async function gatewayHttpHandler(req, res) {
    const requestEpochUs = Date.now() * 1000
    const parsedUrl = new URL(req.url || '/', 'http://gateway.local')

    if (parsedUrl.pathname.startsWith('/latency/uplink/trials')) {
      if (!uplinkTrialManager) {
        sendJson(res, 503, { error: 'uplink diagnostics are unavailable' })
        return
      }
      await handleUplinkTrialRequest(req, res, uplinkTrialManager, parsedUrl)
      return
    }

    if (req.method === 'GET' && req.url && req.url.startsWith('/latency/time')) {
      const sendEpochUs = Date.now() * 1000
      res.writeHead(200, {
        'Cache-Control': 'no-store',
        'Content-Type': 'application/json',
      })
      res.end(
        JSON.stringify({
          server_receive_epoch_us: requestEpochUs,
          server_send_epoch_us: sendEpochUs,
        })
      )
      return
    }

    if (
      req.method === 'GET' &&
      req.url &&
      req.url.startsWith('/latency/clock-status')
    ) {
      const status = await getChronyStatus()
      res.writeHead(200, {
        'Cache-Control': 'no-store',
        'Content-Type': 'application/json',
      })
      res.end(JSON.stringify(status))
      return
    }

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
  handleUplinkTrialRequest,
  handleRocketM2Status,
  readJsonBody,
  sendJson,
}
