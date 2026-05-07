'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  MsgId,
  consumeFrames,
  createSequencer,
  decodeTelemNav,
  encodeBaseRtcm,
  encodeCmdArmJoint,
  encodeCmdDrive,
  encodeFrame,
} = require('../src/protocol/xbee')

test('encodeCmdDrive emits a decodable drive frame', () => {
  const nextSeq = createSequencer()
  const frame = encodeCmdDrive(
    {
      timestamp_ms: 123,
      linear_x_m_s: 1.5,
      linear_y_m_s: -0.25,
      angular_z_rad_s: 0.75,
    },
    nextSeq
  )

  const seen = []
  const remaining = consumeFrames(frame, (msgId, payload) => {
    seen.push({ msgId, payload })
  })

  assert.equal(remaining.length, 0)
  assert.equal(seen.length, 1)
  assert.equal(seen[0].msgId, MsgId.CMD_DRIVE)
  assert.equal(seen[0].payload.readUInt32LE(0), 123)
  assert.equal(seen[0].payload.readFloatLE(4), 1.5)
  assert.equal(seen[0].payload.readFloatLE(8), -0.25)
  assert.equal(seen[0].payload.readFloatLE(12), 0.75)
})

test('encodeCmdArmJoint emits six joint velocities', () => {
  const nextSeq = createSequencer()
  const frame = encodeCmdArmJoint(
    {
      timestamp_ms: 789,
      velocities_rad_s: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
    },
    nextSeq
  )

  const seen = []
  const remaining = consumeFrames(frame, (msgId, payload) => {
    seen.push({ msgId, payload })
  })

  assert.equal(remaining.length, 0)
  assert.equal(seen.length, 1)
  assert.equal(seen[0].msgId, MsgId.CMD_ARM_JOINT)
  assert.equal(seen[0].payload.readUInt32LE(0), 789)
  assert.ok(Math.abs(seen[0].payload.readFloatLE(4) - 0.1) < 1e-6)
  assert.ok(Math.abs(seen[0].payload.readFloatLE(8) + 0.2) < 1e-6)
  assert.ok(Math.abs(seen[0].payload.readFloatLE(12) - 0.3) < 1e-6)
  assert.ok(Math.abs(seen[0].payload.readFloatLE(16) + 0.4) < 1e-6)
  assert.ok(Math.abs(seen[0].payload.readFloatLE(20) - 0.5) < 1e-6)
  assert.ok(Math.abs(seen[0].payload.readFloatLE(24) + 0.6) < 1e-6)
})

test('encodeBaseRtcm fragments oversized RTCM payloads under one sequence id', () => {
  const nextSeq = createSequencer()
  const message = Buffer.alloc(600, 0x42)
  const frames = encodeBaseRtcm({ message }, nextSeq)

  assert.equal(frames.length, 3)
  assert.equal(frames[0][1], MsgId.BASE_RTCM_FRAG)
  assert.equal(frames[1][1], MsgId.BASE_RTCM_FRAG)
  assert.equal(frames[2][1], MsgId.BASE_RTCM_FRAG)
  assert.equal(frames[0][3], frames[1][3])
  assert.equal(frames[1][3], frames[2][3])
})

test('consumeFrames preserves incomplete trailing data', () => {
  const frame = encodeFrame(MsgId.HEARTBEAT, 7, Buffer.from([1, 2, 3, 4]))
  const partial = frame.slice(0, frame.length - 2)
  const seen = []
  const remaining = consumeFrames(partial, (msgId) => {
    seen.push(msgId)
  })

  assert.deepEqual(seen, [])
  assert.equal(Buffer.compare(remaining, partial), 0)
})

test('decodeTelemNav maps the rover telemetry payload layout', () => {
  const payload = Buffer.alloc(32)
  payload.writeUInt32LE(456, 0)
  payload.writeFloatLE(38.4, 4)
  payload.writeFloatLE(-110.79, 8)
  payload.writeFloatLE(1234.5, 12)
  payload.writeFloatLE(180, 16)
  payload.writeFloatLE(0.1, 20)
  payload.writeFloatLE(0.2, 24)
  payload.writeFloatLE(0.3, 28)

  const decoded = decodeTelemNav(payload)
  assert.equal(decoded.timestamp_ms, 456)
  assert.ok(Math.abs(decoded.latitude_deg - 38.4) < 1e-5)
  assert.ok(Math.abs(decoded.longitude_deg + 110.79) < 1e-5)
  assert.equal(decoded.altitude_m, 1234.5)
  assert.equal(decoded.heading_deg, 180)
  assert.ok(Math.abs(decoded.cov_x_var - 0.1) < 1e-6)
  assert.ok(Math.abs(decoded.cov_y_var - 0.2) < 1e-6)
  assert.ok(Math.abs(decoded.cov_yaw_var - 0.3) < 1e-6)
})
