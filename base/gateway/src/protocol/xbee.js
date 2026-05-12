'use strict'

const MsgId = {
  CMD_DRIVE: 0x01,
  CMD_ARM_TWIST: 0x02,
  HEARTBEAT: 0x03,
  MISSION_CONTROL: 0x04,
  CMD_ARM_GRIPPER: 0x05,
  CMD_ARM_JOINT: 0x06,
  CMD_CAMERA_TURRET: 0x07,
  TELEM_BATTERY_1: 0x10,
  TELEM_BATTERY_2: 0x11,
  TELEM_NAV: 0x20,
  BASE_SVIN: 0x30,
  BASE_RTCM: 0x31,
  BASE_RTCM_FRAG: 0x32,
}

// XBEE payloads use a small custom binary frame: magic, message id, payload
// length, sequence number, payload, then CRC16.
const MAGIC = 0xa5

function createSequencer(start = 0) {
  let seq = start & 0xff
  return function nextSeq() {
    const current = seq
    seq = (seq + 1) & 0xff
    return current
  }
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

function encodeCmdDrive(cmd, nextSeq) {
  const payload = Buffer.alloc(16)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  payload.writeFloatLE(cmd.linear_x_m_s, 4)
  payload.writeFloatLE(cmd.linear_y_m_s, 8)
  payload.writeFloatLE(cmd.angular_z_rad_s, 12)
  return encodeFrame(MsgId.CMD_DRIVE, nextSeq(), payload)
}

function encodeCmdArmTwist(cmd, nextSeq) {
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

function encodeHeartbeat(cmd, nextSeq) {
  const payload = Buffer.alloc(4)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  return encodeFrame(MsgId.HEARTBEAT, nextSeq(), payload)
}

function encodeMissionControl(cmd, nextSeq) {
  const payload = Buffer.alloc(6)
  payload.writeUInt8(cmd.command & 0xff, 0)
  payload.writeUInt8(cmd.clear_costmap ? 1 : 0, 1)
  payload.writeUInt32LE(cmd.mission_id >>> 0, 2)
  return encodeFrame(MsgId.MISSION_CONTROL, nextSeq(), payload)
}

function encodeCmdArmGripper(cmd, nextSeq) {
  const payload = Buffer.alloc(8)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  payload.writeFloatLE(cmd.position_norm, 4)
  return encodeFrame(MsgId.CMD_ARM_GRIPPER, nextSeq(), payload)
}

function encodeCmdArmJoint(cmd, nextSeq) {
  const velocities = Array.isArray(cmd.velocities_rad_s) ? cmd.velocities_rad_s : []
  const payload = Buffer.alloc(28)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  for (let i = 0; i < 6; i += 1) {
    const value = Number(velocities[i])
    payload.writeFloatLE(Number.isFinite(value) ? value : 0, 4 + i * 4)
  }
  return encodeFrame(MsgId.CMD_ARM_JOINT, nextSeq(), payload)
}

function encodeCmdCameraTurret(cmd, nextSeq) {
  const payload = Buffer.alloc(16)
  payload.writeUInt32LE(cmd.timestamp_ms >>> 0, 0)
  payload.writeFloatLE(cmd.x, 4)
  payload.writeFloatLE(cmd.y, 8)
  payload.writeFloatLE(cmd.z, 12)
  return encodeFrame(MsgId.CMD_CAMERA_TURRET, nextSeq(), payload)
}

function encodeBaseSvin(msg, nextSeq) {
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

function encodeBaseRtcm(msg, nextSeq, options = {}) {
  if (!msg || !msg.message) return []

  const log = typeof options.log === 'function' ? options.log : () => {}
  const buf = Buffer.from(msg.message)
  const maxSingleLen = 254
  if (buf.length <= maxSingleLen) {
    const payload = Buffer.alloc(1 + buf.length)
    payload.writeUInt8(buf.length, 0)
    buf.copy(payload, 1)
    return [encodeFrame(MsgId.BASE_RTCM, nextSeq(), payload)]
  }

  const maxPayload = 255
  const fragHeaderSize = 4
  const maxFragData = maxPayload - fragHeaderSize
  const fragCount = Math.ceil(buf.length / maxFragData)
  if (fragCount > 255) {
    log(`Dropping RTCM message >${maxFragData * 255} bytes (${buf.length})`)
    return []
  }

  const seqValue = nextSeq()
  const frames = []
  // Fragmented RTCM frames share one sequence id so the rover can treat them as
  // one logical message stream.
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

function consumeFrames(buffer, onFrame) {
  let rxBuffer = buffer
  while (rxBuffer.length >= 4) {
    if (rxBuffer[0] !== MAGIC) {
      // Skip forward until the next plausible frame boundary.
      rxBuffer = rxBuffer.slice(1)
      continue
    }

    const length = rxBuffer[2]
    const frameSize = 4 + length + 2
    if (rxBuffer.length < frameSize) {
      break
    }

    const frame = rxBuffer.slice(0, frameSize)
    const crcExpected = frame.readUInt16LE(frameSize - 2)
    const crcActual = crc16CcittFalse(frame.slice(0, frameSize - 2))
    if (crcExpected !== crcActual) {
      rxBuffer = rxBuffer.slice(1)
      continue
    }

    onFrame(frame[1], frame.slice(4, 4 + length))
    rxBuffer = rxBuffer.slice(frameSize)
  }
  return rxBuffer
}

module.exports = {
  MAGIC,
  MsgId,
  consumeFrames,
  crc16CcittFalse,
  createSequencer,
  decodeTelemBattery,
  decodeTelemNav,
  encodeBaseRtcm,
  encodeBaseSvin,
  encodeCmdArmGripper,
  encodeCmdArmJoint,
  encodeCmdCameraTurret,
  encodeCmdArmTwist,
  encodeCmdDrive,
  encodeFrame,
  encodeHeartbeat,
  encodeMissionControl,
}
