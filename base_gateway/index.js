'use strict'

const fs = require('fs')
const path = require('path')
const dotenv = require('dotenv')

const envLocalPath = path.join(__dirname, '.env.local')
const envPath = path.join(__dirname, '.env')
if (fs.existsSync(envLocalPath)) {
  dotenv.config({ path: envLocalPath })
} else if (fs.existsSync(envPath)) {
  dotenv.config({ path: envPath })
}

const http = require('http')
const jwt = require('jsonwebtoken')
const { SerialPort } = require('serialport')
const { WebSocketServer } = require('ws')

const DEFAULT_DEVICE = '/dev/ttyUSB0'
const DEFAULT_BAUD = 57600
const DEFAULT_PORT = 8081
const DEFAULT_HEARTBEAT_HZ = 2
const DEFAULT_CMD_HZ = 10
const DEFAULT_CMD_TIMEOUT_MS = 500
const DEFAULT_LINK_TIMEOUT_MS = 2000

const args = process.argv.slice(2)
const config = {
  device: getArg('--device') || process.env.SIK_DEVICE || DEFAULT_DEVICE,
  baud: toInt(getArg('--baud') || process.env.SIK_BAUD || DEFAULT_BAUD),
  port: toInt(getArg('--port') || process.env.SIK_WS_PORT || DEFAULT_PORT),
  heartbeatHz: toFloat(
    getArg('--heartbeat-hz') || process.env.SIK_HEARTBEAT_HZ || DEFAULT_HEARTBEAT_HZ
  ),
  cmdHz: toFloat(getArg('--cmd-hz') || process.env.SIK_CMD_HZ || DEFAULT_CMD_HZ),
  cmdTimeoutMs: toInt(
    getArg('--cmd-timeout-ms') || process.env.SIK_CMD_TIMEOUT_MS || DEFAULT_CMD_TIMEOUT_MS
  ),
  linkTimeoutMs: toInt(
    getArg('--link-timeout-ms') || process.env.SIK_LINK_TIMEOUT_MS || DEFAULT_LINK_TIMEOUT_MS
  ),
}

function getArg(name) {
  const index = args.indexOf(name)
  if (index === -1) return null
  const value = args[index + 1]
  if (!value || value.startsWith('--')) return ''
  return value
}

function toInt(value) {
  const parsed = Number.parseInt(String(value), 10)
  return Number.isFinite(parsed) ? parsed : 0
}

function toFloat(value) {
  const parsed = Number.parseFloat(String(value))
  return Number.isFinite(parsed) ? parsed : 0
}

const MsgId = {
  CMD_DRIVE: 0x01,
  CMD_ARM_TWIST: 0x02,
  HEARTBEAT: 0x03,
  TELEM_BATTERY: 0x10,
}

const MAGIC = 0xa5

let seq = 0
let serialReady = false
let lastRxMs = 0
let lastTxMs = 0
let lastHeartbeatRxMs = 0
let latestCmdDrive = null
let latestCmdArmTwist = null
let lastCmdDriveRxMs = 0
let lastCmdArmTwistRxMs = 0

const port = new SerialPort({
  path: config.device,
  baudRate: config.baud,
  autoOpen: true,
})

port.on('open', () => {
  serialReady = true
  log(`Serial open ${config.device} @ ${config.baud}`)
})

port.on('error', (err) => {
  serialReady = false
  log(`Serial error: ${err.message}`)
})

port.on('close', () => {
  serialReady = false
  log('Serial closed')
})

const server = http.createServer((req, res) => {
  if (req.method === 'GET' && req.url && req.url.startsWith('/transitive/token')) {
    handleTransitiveToken(req, res)
    return
  }
  res.writeHead(404)
  res.end()
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
})

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
  console.log(`[base_gateway] ${message}`)
}

function nextSeq() {
  const current = seq
  seq = (seq + 1) & 0xff
  return current
}

function crc16CcittFalse(buffer) {
  let crc = 0xffff
  for (let i = 0; i < buffer.length; i += 1) {
    crc ^= buffer[i] << 8
    for (let bit = 0; bit < 8; bit += 1) {
      if (crc & 0x8000) {
        crc = ((crc << 1) ^ 0x1021) & 0xffff
      } else {
        crc = (crc << 1) & 0xffff
      }
    }
  }
  return crc
}

function encodeFrame(msgId, seqValue, payload) {
  const header = Buffer.alloc(4)
  header.writeUInt8(MAGIC, 0)
  header.writeUInt8(msgId, 1)
  header.writeUInt8(payload.length, 2)
  header.writeUInt8(seqValue, 3)

  const base = Buffer.concat([header, payload])
  const crc = crc16CcittFalse(base)
  const crcBuf = Buffer.alloc(2)
  crcBuf.writeUInt16LE(crc, 0)
  return Buffer.concat([base, crcBuf])
}

function encodeCmdDrive(cmd) {
  const payload = Buffer.alloc(16)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  payload.writeFloatLE(cmd.linear_x_m_s, 4)
  payload.writeFloatLE(cmd.linear_y_m_s, 8)
  payload.writeFloatLE(cmd.angular_z_rad_s, 12)
  return encodeFrame(MsgId.CMD_DRIVE, nextSeq(), payload)
}

function encodeCmdArmTwist(cmd) {
  const payload = Buffer.alloc(28)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  payload.writeFloatLE(cmd.lin_x_m_s, 4)
  payload.writeFloatLE(cmd.lin_y_m_s, 8)
  payload.writeFloatLE(cmd.lin_z_m_s, 12)
  payload.writeFloatLE(cmd.ang_x_rad_s, 16)
  payload.writeFloatLE(cmd.ang_y_rad_s, 20)
  payload.writeFloatLE(cmd.ang_z_rad_s, 24)
  return encodeFrame(MsgId.CMD_ARM_TWIST, nextSeq(), payload)
}

function encodeHeartbeat(cmd) {
  const payload = Buffer.alloc(4)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  return encodeFrame(MsgId.HEARTBEAT, nextSeq(), payload)
}

function decodeTelemBattery(payload) {
  if (payload.length < 16) return null
  return {
    total_capacity_mah: payload.readFloatLE(0),
    available_capacity_mah: payload.readFloatLE(4),
    temperature_c: payload.readFloatLE(8),
    pack_voltage_v: payload.readFloatLE(12),
  }
}

function writeFrame(frame) {
  if (!serialReady) return
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
    const linear = coerceNumber(msg.linear_x_m_s ?? msg.linear_x ?? msg.x)
    const lateral = coerceNumber(msg.linear_y_m_s ?? msg.linear_y ?? msg.y)
    const angular = coerceNumber(msg.angular_z_rad_s ?? msg.angular_z ?? msg.yaw)
    log(`cmd_drive rx x=${linear} y=${lateral} yaw=${angular}`)
    latestCmdDrive = {
      timestamp_ms: Date.now() >>> 0,
      linear_x_m_s: linear,
      linear_y_m_s: lateral,
      angular_z_rad_s: angular,
    }
    lastCmdDriveRxMs = Date.now()
    return
  }

  if (type === 'cmd_arm_twist') {
    latestCmdArmTwist = {
      timestamp_ms: Date.now() >>> 0,
      lin_x_m_s: coerceNumber(msg.lin_x_m_s ?? msg.lin_x ?? msg.linear_x ?? msg.x),
      lin_y_m_s: coerceNumber(msg.lin_y_m_s ?? msg.lin_y ?? msg.linear_y ?? msg.y),
      lin_z_m_s: coerceNumber(msg.lin_z_m_s ?? msg.lin_z ?? msg.linear_z ?? msg.z),
      ang_x_rad_s: coerceNumber(msg.ang_x_rad_s ?? msg.ang_x ?? msg.angular_x),
      ang_y_rad_s: coerceNumber(msg.ang_y_rad_s ?? msg.ang_y ?? msg.angular_y),
      ang_z_rad_s: coerceNumber(msg.ang_z_rad_s ?? msg.ang_z ?? msg.angular_z ?? msg.yaw),
    }
    lastCmdArmTwistRxMs = Date.now()
    return
  }

  if (type === 'heartbeat') {
    const frame = encodeHeartbeat({
      timestamp_ms: Date.now() >>> 0,
    })
    writeFrame(frame)
  }
}

let rxBuffer = Buffer.alloc(0)

port.on('data', (data) => {
  rxBuffer = Buffer.concat([rxBuffer, data])
  parseFrames()
})

function parseFrames() {
  while (rxBuffer.length >= 4) {
    if (rxBuffer[0] !== MAGIC) {
      rxBuffer = rxBuffer.slice(1)
      continue
    }

    const length = rxBuffer[2]
    const frameSize = 4 + length + 2
    if (rxBuffer.length < frameSize) return

    const frame = rxBuffer.slice(0, frameSize)
    const crcExpected = frame.readUInt16LE(frameSize - 2)
    const crcActual = crc16CcittFalse(frame.slice(0, frameSize - 2))
    if (crcExpected !== crcActual) {
      rxBuffer = rxBuffer.slice(1)
      continue
    }

    const msgId = frame[1]
    const payload = frame.slice(4, 4 + length)
    handleFrame(msgId, payload)
    rxBuffer = rxBuffer.slice(frameSize)
  }
}

function handleFrame(msgId, payload) {
  lastRxMs = Date.now()

  if (msgId === MsgId.HEARTBEAT) {
    lastHeartbeatRxMs = Date.now()
  }

  if (msgId === MsgId.TELEM_BATTERY) {
    const telem = decodeTelemBattery(payload)
    if (!telem) return
    broadcast({ type: 'telem_battery', ...telem })
  }
}

function isLinkAlive() {
  if (!serialReady) return false
  if (lastHeartbeatRxMs === 0) return false
  return Date.now() - lastHeartbeatRxMs <= config.linkTimeoutMs
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
    })
    writeFrame(frame)
    broadcast({
      type: 'link_status',
      connected: isLinkAlive(),
      last_rx_ms: lastRxMs,
      last_tx_ms: lastTxMs,
    })
  }, periodMs)
}

if (config.cmdHz > 0) {
  const periodMs = Math.max(1000 / config.cmdHz, 50)
  setInterval(() => {
    const nowMs = Date.now()
    const timeoutMs = config.cmdTimeoutMs > 0 ? config.cmdTimeoutMs : Number.POSITIVE_INFINITY
    if (latestCmdDrive) {
      const isStale = nowMs - lastCmdDriveRxMs > timeoutMs
      const frame = encodeCmdDrive({
        timestamp_ms: nowMs >>> 0,
        linear_x_m_s: isStale ? 0 : latestCmdDrive.linear_x_m_s,
        linear_y_m_s: isStale ? 0 : latestCmdDrive.linear_y_m_s,
        angular_z_rad_s: isStale ? 0 : latestCmdDrive.angular_z_rad_s,
      })
      writeFrame(frame)
    }
    if (latestCmdArmTwist) {
      const isStale = nowMs - lastCmdArmTwistRxMs > timeoutMs
      const frame = encodeCmdArmTwist({
        timestamp_ms: nowMs >>> 0,
        lin_x_m_s: isStale ? 0 : latestCmdArmTwist.lin_x_m_s,
        lin_y_m_s: isStale ? 0 : latestCmdArmTwist.lin_y_m_s,
        lin_z_m_s: isStale ? 0 : latestCmdArmTwist.lin_z_m_s,
        ang_x_rad_s: isStale ? 0 : latestCmdArmTwist.ang_x_rad_s,
        ang_y_rad_s: isStale ? 0 : latestCmdArmTwist.ang_y_rad_s,
        ang_z_rad_s: isStale ? 0 : latestCmdArmTwist.ang_z_rad_s,
      })
      writeFrame(frame)
    }
  }, periodMs)
}
