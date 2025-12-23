'use strict'

const { SerialPort } = require('serialport')
const { WebSocketServer } = require('ws')

const DEFAULT_DEVICE = '/dev/ttyUSB0'
const DEFAULT_BAUD = 57600
const DEFAULT_PORT = 8081
const DEFAULT_HEARTBEAT_HZ = 2

const args = process.argv.slice(2)
const config = {
  device: getArg('--device') || process.env.SIK_DEVICE || DEFAULT_DEVICE,
  baud: toInt(getArg('--baud') || process.env.SIK_BAUD || DEFAULT_BAUD),
  port: toInt(getArg('--port') || process.env.SIK_WS_PORT || DEFAULT_PORT),
  heartbeatHz: toFloat(
    getArg('--heartbeat-hz') || process.env.SIK_HEARTBEAT_HZ || DEFAULT_HEARTBEAT_HZ
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

const wss = new WebSocketServer({ port: config.port })

wss.on('listening', () => {
  log(`WebSocket listening on ${config.port}`)
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
      connected: serialReady,
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
  console.log(`[sik_gateway] ${message}`)
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
  const payload = Buffer.alloc(12)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  payload.writeFloatLE(cmd.linear_x_m_s, 4)
  payload.writeFloatLE(cmd.angular_z_rad_s, 8)
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
  if (payload.length < 12) return null
  return {
    total_capacity_mah: payload.readFloatLE(0),
    available_capacity_mah: payload.readFloatLE(4),
    temperature_c: payload.readFloatLE(8),
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
    const angular = coerceNumber(msg.angular_z_rad_s ?? msg.angular_z ?? msg.yaw)
    const frame = encodeCmdDrive({
      timestamp_ms: Date.now() >>> 0,
      linear_x_m_s: linear,
      angular_z_rad_s: angular,
    })
    writeFrame(frame)
    return
  }

  if (type === 'cmd_arm_twist') {
    const frame = encodeCmdArmTwist({
      timestamp_ms: Date.now() >>> 0,
      lin_x_m_s: coerceNumber(msg.lin_x_m_s ?? msg.lin_x ?? msg.linear_x ?? msg.x),
      lin_y_m_s: coerceNumber(msg.lin_y_m_s ?? msg.lin_y ?? msg.linear_y ?? msg.y),
      lin_z_m_s: coerceNumber(msg.lin_z_m_s ?? msg.lin_z ?? msg.linear_z ?? msg.z),
      ang_x_rad_s: coerceNumber(msg.ang_x_rad_s ?? msg.ang_x ?? msg.angular_x),
      ang_y_rad_s: coerceNumber(msg.ang_y_rad_s ?? msg.ang_y ?? msg.angular_y),
      ang_z_rad_s: coerceNumber(msg.ang_z_rad_s ?? msg.ang_z ?? msg.angular_z ?? msg.yaw),
    })
    writeFrame(frame)
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

  if (msgId === MsgId.TELEM_BATTERY) {
    const telem = decodeTelemBattery(payload)
    if (!telem) return
    broadcast({ type: 'telem_battery', ...telem })
  }
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
      connected: serialReady,
      last_rx_ms: lastRxMs,
      last_tx_ms: lastTxMs,
    })
  }, periodMs)
}
