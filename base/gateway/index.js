'use strict'

const fs = require('fs')
const path = require('path')
const dotenv = require('dotenv')
const { execFile } = require('child_process')
const os = require('os')
const { promisify } = require('util')

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

const DEFAULT_DEVICE = '/dev/ttySIK'
const DEFAULT_BAUD = 57600
const DEFAULT_PORT = 8081
const DEFAULT_HEARTBEAT_HZ = 2
const DEFAULT_CMD_HZ = 10
const DEFAULT_CMD_TIMEOUT_MS = 500
const DEFAULT_LINK_TIMEOUT_MS = 2000
const DEFAULT_ANTENNA_ENABLE = false
const DEFAULT_ANTENNA_DEVICE = ''
const DEFAULT_ANTENNA_BAUD = 115200
const DEFAULT_ANTENNA_CMD_HZ = 2
const DEFAULT_ANTENNA_STALE_MS = 5000
const DEFAULT_ANTENNA_HOME = true
const DEFAULT_ANTENNA_MAX_DEG = 90
const DEFAULT_ANTENNA_SMOOTHING = 0
const DEFAULT_ANTENNA_BOOT_WAIT_MS = 2000
const DEFAULT_ANTENNA_LOG_MS = 5000
const DEFAULT_ANTENNA_STATUS_MS = 1000
const DEFAULT_ANTENNA_ALLOW_PROVISIONAL = true
const DEFAULT_BASE_HEADING_OFFSET_DEG = 0
const DEFAULT_ROCKET_M2_ENABLE = false
const DEFAULT_ROCKET_M2_IP = ''
const DEFAULT_ROCKET_M2_USER = ''
const DEFAULT_ROCKET_M2_PASS = ''
const DEFAULT_ROCKET_M2_POLL_MS = 5000
const DEFAULT_ROCKET_M2_TIMEOUT_MS = 4000

const execFileAsync = promisify(execFile)

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
  antennaEnable: toBool(
    getArg('--antenna-enable') || process.env.BASE_ANTENNA_ENABLE || DEFAULT_ANTENNA_ENABLE
  ),
  antennaDevice:
    getArg('--antenna-device') || process.env.BASE_ANTENNA_DEVICE || DEFAULT_ANTENNA_DEVICE,
  antennaBaud: toInt(
    getArg('--antenna-baud') || process.env.BASE_ANTENNA_BAUD || DEFAULT_ANTENNA_BAUD
  ),
  antennaCmdHz: toFloat(
    getArg('--antenna-cmd-hz') || process.env.BASE_ANTENNA_CMD_HZ || DEFAULT_ANTENNA_CMD_HZ
  ),
  antennaStaleMs: toInt(
    getArg('--antenna-stale-ms') ||
      process.env.BASE_ANTENNA_STALE_MS ||
      DEFAULT_ANTENNA_STALE_MS
  ),
  antennaHome: toBool(
    getArg('--antenna-home') || process.env.BASE_ANTENNA_HOME || DEFAULT_ANTENNA_HOME
  ),
  antennaMaxDeg: toFloat(
    getArg('--antenna-max-deg') || process.env.BASE_ANTENNA_MAX_DEG || DEFAULT_ANTENNA_MAX_DEG
  ),
  antennaSmoothing: toFloat(
    getArg('--antenna-smoothing') ||
      process.env.BASE_ANTENNA_SMOOTHING ||
      DEFAULT_ANTENNA_SMOOTHING
  ),
  antennaBootWaitMs: toInt(
    getArg('--antenna-boot-wait-ms') ||
      process.env.BASE_ANTENNA_BOOT_WAIT_MS ||
      DEFAULT_ANTENNA_BOOT_WAIT_MS
  ),
  antennaLogMs: toInt(
    getArg('--antenna-log-ms') ||
      process.env.BASE_ANTENNA_LOG_MS ||
      DEFAULT_ANTENNA_LOG_MS
  ),
  antennaStatusMs: toInt(
    getArg('--antenna-status-ms') ||
      process.env.BASE_ANTENNA_STATUS_MS ||
      DEFAULT_ANTENNA_STATUS_MS
  ),
  antennaAllowProvisional: toBool(
    getArg('--antenna-allow-provisional') ||
      process.env.BASE_ANTENNA_ALLOW_PROVISIONAL ||
      DEFAULT_ANTENNA_ALLOW_PROVISIONAL
  ),
  baseHeadingOffsetDeg: toFloat(
    getArg('--base-heading-deg') ||
      process.env.BASE_HEADING_OFFSET_DEG ||
      DEFAULT_BASE_HEADING_OFFSET_DEG
  ),
  rocketM2Enable: toBool(
    getArg('--rocket-m2-enable') || process.env.ROCKET_M2_ENABLE || DEFAULT_ROCKET_M2_ENABLE
  ),
  rocketM2Ip: getArg('--rocket-m2-ip') || process.env.ROCKET_M2_IP || DEFAULT_ROCKET_M2_IP,
  rocketM2User:
    getArg('--rocket-m2-user') || process.env.ROCKET_M2_USER || DEFAULT_ROCKET_M2_USER,
  rocketM2Pass:
    getArg('--rocket-m2-pass') || process.env.ROCKET_M2_PASS || DEFAULT_ROCKET_M2_PASS,
  rocketM2PollMs: toInt(
    getArg('--rocket-m2-poll-ms') ||
      process.env.ROCKET_M2_POLL_MS ||
      DEFAULT_ROCKET_M2_POLL_MS
  ),
  rocketM2TimeoutMs: toInt(
    getArg('--rocket-m2-timeout-ms') ||
      process.env.ROCKET_M2_TIMEOUT_MS ||
      DEFAULT_ROCKET_M2_TIMEOUT_MS
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

function toBool(value) {
  if (value === true || value === false) return value
  const text = String(value).toLowerCase()
  return text === '1' || text === 'true' || text === 'yes' || text === 'on'
}

const { AntennaTracker } = require('./antenna_tracker')
const MsgId = {
  CMD_DRIVE: 0x01,
  CMD_ARM_TWIST: 0x02,
  HEARTBEAT: 0x03,
  MISSION_CONTROL: 0x04,
  CAN_ESTOP_REQUEST: 0x05,
  CAN_ESTOP_RESPONSE: 0x06,
  TELEM_BATTERY_1: 0x10,
  TELEM_BATTERY_2: 0x11,
  TELEM_NAV: 0x20,
  BASE_SVIN: 0x30,
  BASE_RTCM: 0x31,
  BASE_RTCM_FRAG: 0x32,
}

const MAGIC = 0xa5

let seq = 0
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
let estopRequestSeq = 0

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

function parseRocketM2Signal(raw) {
  const text = String(raw || '').trim()
  const parsed = JSON.parse(text)
  if (!parsed || typeof parsed !== 'object') {
    throw new Error('Invalid Rocket M2 response')
  }
  return {
    signal: toNumberOrNull(parsed.signal),
    rssi: toNumberOrNull(parsed.rssi),
    noisef: toNumberOrNull(parsed.noisef),
    chwidth: toNumberOrNull(parsed.chwidth),
    rx_chainmask: toNumberOrNull(parsed.rx_chainmask),
    chainrssi: toNumberArray(parsed.chainrssi),
    chainrssimgmt: toNumberArray(parsed.chainrssimgmt),
    chainrssiext: toNumberArray(parsed.chainrssiext),
  }
}

function createRocketM2Status(overrides = {}) {
  return {
    connected: false,
    updated_at_ms: Date.now(),
    last_success_ms: rocketM2LastSuccessMs || null,
    signal: null,
    rssi: null,
    noisef: null,
    chwidth: null,
    rx_chainmask: null,
    chainrssi: [],
    chainrssimgmt: [],
    chainrssiext: [],
    error: null,
    ...overrides,
  }
}

function toNumberOrNull(value) {
  if (value == null) return null
  const num = Number(value)
  return Number.isFinite(num) ? num : null
}

function toNumberArray(value) {
  if (!Array.isArray(value)) return []
  return value
    .map((item) => (item == null ? null : Number(item)))
    .filter((item) => Number.isFinite(item))
}

function formatRocketM2Error(err) {
  if (!err) return 'Unknown error'
  if (typeof err === 'string') return err
  const parts = []
  if (err.code != null) parts.push(`code=${err.code}`)
  if (err.signal) parts.push(`signal=${err.signal}`)
  if (err.killed) parts.push('killed')
  if (err.message) {
    const message = redactRocketM2Secrets(err.message.split('\n')[0])
    if (message) parts.push(message)
  }
  return parts.join(' ') || 'Unknown error'
}

function redactRocketM2Secrets(text) {
  if (!text) return ''
  return text.replace(/(password=)([^&\\s]+)/gi, '$1***')
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

function encodeMissionControl(cmd) {
  const payload = Buffer.alloc(6)
  payload.writeUInt8(cmd.command & 0xff, 0)
  payload.writeUInt8(cmd.clear_costmap ? 1 : 0, 1)
  payload.writeUInt32LE(cmd.mission_id >>> 0, 2)
  return encodeFrame(MsgId.MISSION_CONTROL, nextSeq(), payload)
}

function encodeCanEstopRequest(cmd) {
  const payload = Buffer.alloc(2)
  payload.writeUInt8(cmd.request_id & 0xff, 0)
  payload.writeUInt8(cmd.enable ? 1 : 0, 1)
  return encodeFrame(MsgId.CAN_ESTOP_REQUEST, nextSeq(), payload)
}

function encodeBaseSvin(msg) {
  // Payload: int32 mean_x/y/z cm (12), int8 mean_xhp/mean_yhp/mean_zhp (3),
  // uint8 valid, uint8 active (2), uint32 mean_acc_0p1mm, uint32 obs (8) => 25 bytes
  const payload = Buffer.alloc(25)
  payload.writeInt32LE(msg.mean_x_cm | 0, 0)
  payload.writeInt32LE(msg.mean_y_cm | 0, 4)
  payload.writeInt32LE(msg.mean_z_cm | 0, 8)
  payload.writeInt8(msg.mean_x_hp | 0, 12)
  payload.writeInt8(msg.mean_y_hp | 0, 13)
  payload.writeInt8(msg.mean_z_hp | 0, 14)
  payload.writeUInt8(msg.valid ? 1 : 0, 15)
  payload.writeUInt8(msg.active ? 1 : 0, 16)
  payload.writeUInt32LE(msg.mean_acc_0p1mm >>> 0, 17)
  payload.writeUInt32LE(msg.obs >>> 0, 21)
  return encodeFrame(MsgId.BASE_SVIN, nextSeq(), payload)
}

function encodeBaseRtcm(msg) {
  if (!msg || !msg.message) return []
  const buf = Buffer.from(msg.message)
  const maxSingleLen = 254 // legacy single-frame limit
  if (buf.length <= maxSingleLen) {
    const payload = Buffer.alloc(1 + buf.length)
    payload.writeUInt8(buf.length, 0)
    buf.copy(payload, 1)
    return [encodeFrame(MsgId.BASE_RTCM, nextSeq(), payload)]
  }

  const maxPayload = 255 // fits in uint8 length field
  const fragHeaderSize = 4 // u16 msg_len, u8 frag_count, u8 frag_index
  const maxFragData = maxPayload - fragHeaderSize // 251 bytes
  const fragCount = Math.ceil(buf.length / maxFragData)
  if (fragCount > 255) {
    log(`Dropping RTCM message >${maxFragData * 255} bytes (${buf.length})`)
    return []
  }

  const seqValue = nextSeq()
  const frames = []
  for (let fragIndex = 0; fragIndex < fragCount; fragIndex += 1) {
    const start = fragIndex * maxFragData
    const end = Math.min(start + maxFragData, buf.length)
    const frag = buf.slice(start, end)
    const payload = Buffer.alloc(fragHeaderSize + frag.length)
    payload.writeUInt16LE(buf.length, 0)
    payload.writeUInt8(fragCount, 2)
    payload.writeUInt8(fragIndex, 3)
    frag.copy(payload, fragHeaderSize)
    frames.push(encodeFrame(MsgId.BASE_RTCM_FRAG, seqValue, payload))
  }
  return frames
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

function decodeTelemNav(payload) {
  if (payload.length < 32) return null
  return {
    timestamp_ms: payload.readUInt32LE(0),
    latitude_deg: payload.readFloatLE(4),
    longitude_deg: payload.readFloatLE(8),
    altitude_m: payload.readFloatLE(12),
    heading_deg: payload.readFloatLE(16),
    cov_x_var: payload.readFloatLE(20),
    cov_y_var: payload.readFloatLE(24),
    cov_yaw_var: payload.readFloatLE(28),
  }
}

function decodeCanEstopResponse(payload) {
  if (payload.length !== 3) return null
  const requestId = payload.readUInt8(0)
  const enabled = payload.readUInt8(1) !== 0
  const success = payload.readUInt8(2) !== 0
  return { request_id: requestId, enabled, success }
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
    })
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
    })
    writeFrame(frame)
    return
  }

  if (type === 'heartbeat') {
    const frame = encodeHeartbeat({
      timestamp_ms: Date.now() >>> 0,
    })
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
    })
    writeFrame(frame)
    return
  }

  if (type === 'can_estop') {
    let requestId = 0
    if (msg.request_id != null) {
      const requestIdRaw = coerceNumber(msg.request_id)
      requestId = Math.min(255, Math.max(0, Math.floor(requestIdRaw)))
    } else {
      requestId = estopRequestSeq
      estopRequestSeq = (estopRequestSeq + 1) & 0xff
    }
    const enable = Boolean(msg.enable)
    const frame = encodeCanEstopRequest({ request_id: requestId, enable })
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
      })
      if (frame) writeFrame(frame)
    }
  )

  rosNode.createSubscription('rtcm_msgs/msg/Message', '/base/rtcm', (msg) => {
    if (!msg) return
    const frames = encodeBaseRtcm(msg)
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
    return
  }

  if (msgId === MsgId.CAN_ESTOP_RESPONSE) {
    const resp = decodeCanEstopResponse(payload)
    if (!resp) return
    broadcast({ type: 'can_estop', ...resp })
    return
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
