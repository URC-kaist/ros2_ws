'use strict'

const path = require('path')
const { execFile } = require('child_process')
const os = require('os')
const { promisify } = require('util')

const http = require('http')
const jwt = require('jsonwebtoken')
const { SerialPort } = require('serialport')
const { WebSocketServer } = require('ws')
const { AntennaTracker } = require('./antenna_tracker')
const { loadGatewayEnv, parseGatewayConfig, toInt } = require('./src/config')
const {
  MsgId,
  consumeFrames,
  createSequencer,
  decodeTelemBattery,
  decodeTelemNav,
  encodeBaseRtcm,
  encodeBaseSvin,
  encodeCmdArmGripper,
  encodeCmdArmTwist,
  encodeCmdDrive,
  encodeHeartbeat,
  encodeMissionControl,
} = require('./src/protocol/sik')
const {
  createRocketM2Status,
  formatRocketM2Error,
  parseRocketM2Signal,
} = require('./src/rocket_m2')

loadGatewayEnv(__dirname)

const execFileAsync = promisify(execFile)
const config = parseGatewayConfig(process.argv.slice(2), process.env)

const nextSeq = createSequencer()
let serialReady = false
let lastRxMs = 0
let lastTxMs = 0
let lastHeartbeatRxMs = 0
let port = null
let portReconnectTimer = null
let antennaTracker = null
let antennaStatusTimer = null
let rosBridgeReady = false
let rosNode = null
let rclnodejs = null
let lastSvinTxMs = 0
let rocketM2Status = null
let rocketM2LastSuccessMs = 0
let rocketM2PollTimer = null
let rocketM2PollInFlight = false
let rocketM2CookiePath = null
let rocketM2LastError = null

function schedulePortReconnect() {
  if (portReconnectTimer) return
  portReconnectTimer = setTimeout(() => {
    portReconnectTimer = null
    setupPort()
  }, 1000)
}

function setupPort() {
  if (port) {
    try {
      port.removeAllListeners()
      port.destroy()
    } catch (_) {
      /* ignore */
    }
    port = null
  }

  port = new SerialPort({
    path: config.device,
    baudRate: config.baud,
    autoOpen: true,
  })

  port.on('open', () => {
    serialReady = true
    rxBuffer = Buffer.alloc(0)
    log(`Serial open ${config.device} @ ${config.baud}`)
  })

  port.on('error', (err) => {
    serialReady = false
    log(`Serial error: ${err.message}`)
    schedulePortReconnect()
  })

  port.on('close', () => {
    serialReady = false
    log('Serial closed')
    schedulePortReconnect()
  })

  port.on('data', (data) => {
    rxBuffer = Buffer.concat([rxBuffer, data])
    parseFrames()
  })
}

setupPort()

if (config.antennaEnable) {
  antennaTracker = new AntennaTracker({
    enabled: true,
    device: config.antennaDevice,
    baud: config.antennaBaud,
    cmdHz: config.antennaCmdHz,
    staleMs: config.antennaStaleMs,
    autoHome: config.antennaHome,
    maxRad: (Math.max(config.antennaMaxDeg, 0) * Math.PI) / 180,
    smoothing: config.antennaSmoothing,
    bootWaitMs: config.antennaBootWaitMs,
    logHeadingMs: config.antennaLogMs,
    allowProvisional: config.antennaAllowProvisional,
    headingOffsetDeg: config.baseHeadingOffsetDeg,
    log,
  })
  antennaTracker.start()

  if (config.antennaStatusMs > 0) {
    antennaStatusTimer = setInterval(() => {
      if (!antennaTracker) return
      const status = antennaTracker.getStatus()
      broadcast({ type: 'base_status', ...status })
    }, Math.max(config.antennaStatusMs, 200))
  }
}

startRosBridge()

const server = http.createServer((req, res) => {
  if (req.method === 'GET' && req.url && req.url.startsWith('/transitive/token')) {
    handleTransitiveToken(req, res)
    return
  }
  if (req.method === 'GET' && req.url && req.url.startsWith('/rocket-m2/status')) {
    handleRocketM2Status(req, res)
    return
  }
  res.writeHead(404)
  res.end()
})

function shutdown() {
  if (antennaStatusTimer) {
    clearInterval(antennaStatusTimer)
    antennaStatusTimer = null
  }
  if (rocketM2PollTimer) {
    clearInterval(rocketM2PollTimer)
    rocketM2PollTimer = null
  }
  if (antennaTracker) {
    antennaTracker.stop()
  }
  if (port) {
    try {
      port.close()
    } catch (_) {
      /* ignore */
    }
  }
  process.exit(0)
}

process.on('SIGINT', () => {
  log('SIGINT received, shutting down')
  shutdown()
})

process.on('SIGTERM', () => {
  log('SIGTERM received, shutting down')
  shutdown()
})

const wss = new WebSocketServer({ server })

server.listen(config.port, () => {
  log(`HTTP/WebSocket listening on ${config.port}`)
})

wss.on('connection', (ws) => {
  ws.on('message', (data) => {
    const text = data.toString()
    let msg
    try {
      msg = JSON.parse(text)
    } catch {
      return
    }
    handleDashboardMessage(msg)
  })

  ws.send(
    JSON.stringify({
      type: 'link_status',
      connected: isLinkAlive(),
      last_rx_ms: lastRxMs,
      last_tx_ms: lastTxMs,
    })
  )
  if (rocketM2Status) {
    ws.send(
      JSON.stringify({
        type: 'rocket_m2_status',
        ...rocketM2Status,
      })
    )
  }
})

startRocketM2Polling()

function broadcast(obj) {
  const payload = JSON.stringify(obj)
  for (const client of wss.clients) {
    if (client.readyState === 1) {
      client.send(payload)
    }
  }
}

function log(message) {
  // eslint-disable-next-line no-console
  console.log(`[gateway] ${message}`)
}

function startRocketM2Polling() {
  const configured =
    config.rocketM2Ip && config.rocketM2User && config.rocketM2Pass
  const enabled = config.rocketM2Enable || configured
  if (!enabled) return
  if (!configured) {
    log('Rocket M2 enabled but missing ROCKET_M2_IP/USER/PASS')
    return
  }
  const pollMs = Math.max(config.rocketM2PollMs, 0)
  if (pollMs <= 0) {
    log('Rocket M2 polling disabled (interval <= 0)')
    return
  }
  if (!rocketM2CookiePath) {
    rocketM2CookiePath = path.join(os.tmpdir(), `rocket_m2_${process.pid}.cookies`)
  }
  pollRocketM2Status()
  rocketM2PollTimer = setInterval(pollRocketM2Status, Math.max(pollMs, 500))
  log(`Rocket M2 polling every ${Math.max(pollMs, 500)} ms`)
}

async function pollRocketM2Status() {
  if (rocketM2PollInFlight) return
  rocketM2PollInFlight = true
  const nowMs = Date.now()
  try {
    const data = await fetchRocketM2Signal()
    rocketM2LastSuccessMs = nowMs
    rocketM2Status = createRocketM2Status({
      connected: true,
      updated_at_ms: nowMs,
      last_success_ms: nowMs,
      error: null,
      ...data,
    }, {
      nowMs,
      lastSuccessMs: nowMs,
    })
    if (rocketM2LastError) {
      log('Rocket M2 polling recovered')
      rocketM2LastError = null
    }
  } catch (err) {
    const error = formatRocketM2Error(err)
    const previous = rocketM2Status
    rocketM2Status = createRocketM2Status({
      connected: false,
      updated_at_ms: nowMs,
      last_success_ms: rocketM2LastSuccessMs || null,
      signal: previous?.signal ?? null,
      rssi: previous?.rssi ?? null,
      noisef: previous?.noisef ?? null,
      chwidth: previous?.chwidth ?? null,
      rx_chainmask: previous?.rx_chainmask ?? null,
      chainrssi: previous?.chainrssi ?? [],
      chainrssimgmt: previous?.chainrssimgmt ?? [],
      chainrssiext: previous?.chainrssiext ?? [],
      error,
    }, {
      nowMs,
      lastSuccessMs: rocketM2LastSuccessMs || null,
    })
    if (error && error !== rocketM2LastError) {
      log(`Rocket M2 poll failed: ${error}`)
      rocketM2LastError = error
    }
  } finally {
    rocketM2PollInFlight = false
  }

  if (rocketM2Status) {
    broadcast({ type: 'rocket_m2_status', ...rocketM2Status })
  }
}

async function fetchRocketM2Signal() {
  const cookiePath = rocketM2CookiePath
  if (!cookiePath) {
    throw new Error('Rocket M2 cookie path not initialized')
  }
  const loginPayload = new URLSearchParams({
    username: config.rocketM2User,
    password: config.rocketM2Pass,
    uri: '/index.cgi',
  }).toString()

  await runCurl(
    [
      '-k',
      '--ciphers',
      'DEFAULT:@SECLEVEL=0',
      '-sS',
      '-L',
      '-c',
      cookiePath,
      '-b',
      cookiePath,
      '-X',
      'POST',
      `https://${config.rocketM2Ip}/login.cgi`,
      '-H',
      'Content-Type: application/x-www-form-urlencoded',
      '--data',
      loginPayload,
      '-o',
      '/dev/null',
    ],
    config.rocketM2TimeoutMs
  )

  const signalRaw = await runCurl(
    [
      '-k',
      '--ciphers',
      'DEFAULT:@SECLEVEL=0',
      '-sS',
      '-L',
      '-b',
      cookiePath,
      '-c',
      cookiePath,
      '-H',
      'Accept: application/json',
      `https://${config.rocketM2Ip}/signal.cgi?_=${Date.now()}`,
    ],
    config.rocketM2TimeoutMs
  )

  return parseRocketM2Signal(signalRaw)
}

function runCurl(args, timeoutMs) {
  return execFileAsync('curl', args, {
    timeout: Math.max(timeoutMs || 0, 1000),
    maxBuffer: 1024 * 1024,
    encoding: 'utf8',
  }).then(({ stdout }) => stdout)
}

function writeFrame(frame) {
  if (!serialReady || !port) return
  port.write(frame)
  lastTxMs = Date.now()
}

function coerceNumber(value) {
  const num = Number(value)
  return Number.isFinite(num) ? num : 0
}

function handleDashboardMessage(msg) {
  if (!msg || typeof msg !== 'object') return

  const type = msg.type || msg.event
  if (!type) return

  if (type === 'cmd_drive') {
    const linear = coerceNumber(msg.linear_x_m_s)
    const lateral = coerceNumber(msg.linear_y_m_s)
    const angular = coerceNumber(msg.angular_z_rad_s)
    log(`cmd_drive rx x=${linear} y=${lateral} yaw=${angular}`)
    const frame = encodeCmdDrive({
      timestamp_ms: Date.now() >>> 0,
      linear_x_m_s: linear,
      linear_y_m_s: lateral,
      angular_z_rad_s: angular,
    }, nextSeq)
    writeFrame(frame)
    return
  }

  if (type === 'cmd_arm_twist') {
    log(
      `cmd_arm_twist rx lin=(${coerceNumber(msg.lin_x_m_s)}, ${coerceNumber(
        msg.lin_y_m_s
      )}, ${coerceNumber(msg.lin_z_m_s)}) ang=(${coerceNumber(
        msg.ang_x_rad_s
      )}, ${coerceNumber(msg.ang_y_rad_s)}, ${coerceNumber(msg.ang_z_rad_s)})`
    )
    const frame = encodeCmdArmTwist({
      timestamp_ms: Date.now() >>> 0,
      lin_x_m_s: coerceNumber(msg.lin_x_m_s),
      lin_y_m_s: coerceNumber(msg.lin_y_m_s),
      lin_z_m_s: coerceNumber(msg.lin_z_m_s),
      ang_x_rad_s: coerceNumber(msg.ang_x_rad_s),
      ang_y_rad_s: coerceNumber(msg.ang_y_rad_s),
      ang_z_rad_s: coerceNumber(msg.ang_z_rad_s),
    }, nextSeq)
    writeFrame(frame)
    return
  }

  if (type === 'heartbeat') {
    const frame = encodeHeartbeat({
      timestamp_ms: Date.now() >>> 0,
    }, nextSeq)
    writeFrame(frame)
    return
  }

  if (type === 'mission_control') {
    const command = Math.min(255, Math.max(0, Math.floor(coerceNumber(msg.command))))
    const missionId = Math.min(
      0xffffffff,
      Math.max(0, Math.floor(coerceNumber(msg.mission_id)))
    )
    const clearCostmap = Boolean(msg.clear_costmap)
    log(
      `mission_control rx cmd=${command} clear=${clearCostmap ? 'true' : 'false'} mission_id=${missionId}`
    )
    const frame = encodeMissionControl({
      command,
      clear_costmap: clearCostmap,
      mission_id: missionId,
    }, nextSeq)
    writeFrame(frame)
    return
  }

  if (type === 'cmd_arm_gripper') {
    const positionNorm = coerceNumber(msg.position_norm)
    log(`cmd_arm_gripper rx pos_norm=${positionNorm}`)
    const frame = encodeCmdArmGripper({
      timestamp_ms: Date.now() >>> 0,
      position_norm: positionNorm,
    }, nextSeq)
    writeFrame(frame)
    return
  }

  if (type === 'base_heading') {
    const heading = coerceNumber(msg.heading_deg)
    if (antennaTracker) {
      antennaTracker.setBaseHeadingOffsetDeg(heading)
    }
  }
}

async function startRosBridge() {
  try {
    // eslint-disable-next-line global-require
    rclnodejs = require('rclnodejs')
  } catch (err) {
    log(`ROS bridge disabled: rclnodejs not available (${err.message})`)
    return
  }

  try {
    if (!rclnodejs.isInitialized || !rclnodejs.isInitialized()) {
      await rclnodejs.init()
    }
  } catch (err) {
    // If already initialised elsewhere, ignore.
    if (!/already been initialized/i.test(err.message || '')) {
      log(`ROS bridge init failed: ${err.message}`)
      return
    }
  }

  const nodeName = `gateway_ros_${process.pid || Math.floor(Math.random() * 1e5)}`
  rosNode = new rclnodejs.Node(nodeName)

  rosNode.createSubscription(
    'ublox_ubx_msgs/msg/UBXNavSvin',
    '/base/ubx_nav_svin',
    (msg) => {
      if (!msg) return
      const nowMs = Date.now()
      if (nowMs - lastSvinTxMs < 500) return // throttle ~2 Hz max
      lastSvinTxMs = nowMs
      const frame = encodeBaseSvin({
        mean_x_cm: msg.mean_x,
        mean_y_cm: msg.mean_y,
        mean_z_cm: msg.mean_z,
        mean_x_hp: msg.mean_x_hp,
        mean_y_hp: msg.mean_y_hp,
        mean_z_hp: msg.mean_z_hp,
        valid: !!msg.valid,
        active: !!msg.active,
        mean_acc_0p1mm: msg.mean_acc >>> 0,
        obs: msg.obs >>> 0,
      }, nextSeq)
      if (frame) writeFrame(frame)
    }
  )

  rosNode.createSubscription('rtcm_msgs/msg/Message', '/base/rtcm', (msg) => {
    if (!msg) return
    const frames = encodeBaseRtcm(msg, nextSeq, { log })
    for (const frame of frames) {
      writeFrame(frame)
    }
  })

  rclnodejs.spin(rosNode)
  rosBridgeReady = true
  log('ROS bridge started (SVIN + RTCM over SiK)')
}

let rxBuffer = Buffer.alloc(0)

function parseFrames() {
  rxBuffer = consumeFrames(rxBuffer, (msgId, payload) => {
    handleFrame(msgId, payload)
  })
}

function handleFrame(msgId, payload) {
  lastRxMs = Date.now()

  if (msgId === MsgId.HEARTBEAT) {
    lastHeartbeatRxMs = Date.now()
  }

  if (msgId === MsgId.TELEM_BATTERY_1 || msgId === MsgId.TELEM_BATTERY_2) {
    const telem = decodeTelemBattery(payload)
    if (!telem) return
    const batteryId = msgId === MsgId.TELEM_BATTERY_2 ? 2 : 1
    broadcast({ type: 'telem_battery', battery_id: batteryId, ...telem })
    return
  }

  if (msgId === MsgId.TELEM_NAV) {
    const nav = decodeTelemNav(payload)
    if (!nav) return
    if (antennaTracker) {
      antennaTracker.updateRoverNav(nav)
    }
    broadcast({ type: 'telem_nav', ...nav })
  }
}

function isLinkAlive() {
  if (!serialReady) return false
  if (lastHeartbeatRxMs === 0) return false
  return Date.now() - lastHeartbeatRxMs <= config.linkTimeoutMs
}

function handleRocketM2Status(_req, res) {
  const configured =
    config.rocketM2Ip && config.rocketM2User && config.rocketM2Pass
  const enabled = config.rocketM2Enable || configured
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
  if (!rocketM2Status) {
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
      ...rocketM2Status,
    })
  )
}

function handleTransitiveToken(req, res) {
  log(`Transitive token request from ${req.socket.remoteAddress || 'unknown'}`)
  const secret = process.env.TRANSITIVE_JWT_SECRET
  if (!secret) {
    log('TRANSITIVE_JWT_SECRET not set')
    res.writeHead(500, { 'Content-Type': 'application/json' })
    res.end(JSON.stringify({ error: 'TRANSITIVE_JWT_SECRET not set' }))
    return
  }

  const url = new URL(req.url, `http://${req.headers.host || 'localhost'}`)
  const id = url.searchParams.get('id') || process.env.TRANSITIVE_ID || 'unknown'
  const device = url.searchParams.get('device') || process.env.TRANSITIVE_DEVICE || 'unknown'
  const capability =
    url.searchParams.get('capability') ||
    process.env.TRANSITIVE_CAPABILITY ||
    '@transitive-robotics/webrtc-video'
  const userId = url.searchParams.get('userId') || process.env.TRANSITIVE_USER_ID || 'operator'
  const validity = toInt(
    url.searchParams.get('validity') || process.env.TRANSITIVE_VALIDITY || 86400
  )

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

if (config.heartbeatHz > 0) {
  const periodMs = Math.max(1000 / config.heartbeatHz, 100)
  setInterval(() => {
    const frame = encodeHeartbeat({
      timestamp_ms: Date.now() >>> 0,
    }, nextSeq)
    writeFrame(frame)
    broadcast({
      type: 'link_status',
      connected: isLinkAlive(),
      last_rx_ms: lastRxMs,
      last_tx_ms: lastTxMs,
    })
  }, periodMs)
}
