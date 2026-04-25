'use strict'

const { EventEmitter } = require('events')
const { SerialPort } = require('serialport')

const SOF0 = 0xaa
const SOF1 = 0x55

const CMD_HOMING_START = 0x01
const CMD_MOVE_TO_RAD = 0x02

const CMD_ACK = 0x80
const CMD_DONE = 0x81
const CMD_ERROR = 0x82

// The antenna controller uses a compact framed protocol unrelated to the XBEE
// radio framing used elsewhere in the gateway.
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

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max)
}

function q16_16(rad) {
  return Math.round(rad * 65536)
}

// Thin serial client for the base antenna controller. It owns framing,
// sequence numbers, and acknowledgement event decoding.
class BaseStationAntenna extends EventEmitter {
  constructor(options = {}) {
    super()
    this.device = options.device || null
    this.baud = options.baud || 115200
    this.port = options.port || null
    this.disableControlLines = options.disableControlLines !== false
    this.seq = 0
    this.rxBuffer = Buffer.alloc(0)

    if (!this.port && this.device) {
      this.port = new SerialPort({
        path: this.device,
        baudRate: this.baud,
        autoOpen: options.autoOpen !== false,
      })
    }

    if (this.port) {
      this._attachPort(this.port)
    }
  }

  _attachPort(port) {
    port.on('open', () => {
      // Some Arduino-compatible boards reset on DTR/RTS changes. Keep them low
      // so opening the port does not reboot the controller unexpectedly.
      if (this.disableControlLines && typeof port.set === 'function') {
        port.set({ dtr: false, rts: false }, (err) => {
          if (err) {
            this.emit('serial_error', err)
          }
          this.emit('open')
        })
        return
      }
      this.emit('open')
    })
    port.on('close', () => this.emit('close'))
    port.on('error', (err) => this.emit('serial_error', err))
    port.on('data', (data) => {
      this.rxBuffer = Buffer.concat([this.rxBuffer, data])
      this._parseFrames()
    })
  }

  _nextSeq() {
    const current = this.seq
    this.seq = (this.seq + 1) & 0xff
    return current
  }

  _buildFrame(cmd, seqValue, payload) {
    const lenField = 2 + payload.length
    const body = Buffer.concat([
      Buffer.from([lenField, seqValue, cmd]),
      payload,
    ])
    const crc = crc16CcittFalse(body)
    const crcBuf = Buffer.alloc(2)
    crcBuf.writeUInt16LE(crc, 0)
    return Buffer.concat([Buffer.from([SOF0, SOF1]), body, crcBuf])
  }

  _parseFrames() {
    while (this.rxBuffer.length >= 4) {
      if (this.rxBuffer[0] !== SOF0 || this.rxBuffer[1] !== SOF1) {
        this.rxBuffer = this.rxBuffer.slice(1)
        continue
      }

      const lenField = this.rxBuffer[2]
      const frameSize = 2 + 1 + lenField + 2
      if (this.rxBuffer.length < frameSize) return

      const frame = this.rxBuffer.slice(0, frameSize)
      const crcExpected = frame.readUInt16LE(frameSize - 2)
      const crcActual = crc16CcittFalse(frame.slice(2, frameSize - 2))
      if (crcExpected !== crcActual) {
        this.rxBuffer = this.rxBuffer.slice(1)
        continue
      }

      const seqValue = frame[3]
      const cmd = frame[4]
      const payload = frame.slice(5, frameSize - 2)
      this._handleFrame(seqValue, cmd, payload)
      this.rxBuffer = this.rxBuffer.slice(frameSize)
    }
  }

  _handleFrame(seqValue, cmd, payload) {
    this.emit('frame', { seq: seqValue, cmd, payload })

    if (cmd === CMD_ACK && payload.length >= 2) {
      this.emit('ack', { seq: payload[0], cmd: payload[1] })
      return
    }
    if (cmd === CMD_DONE && payload.length >= 2) {
      this.emit('done', { seq: payload[0], cmd: payload[1] })
      return
    }
    if (cmd === CMD_ERROR && payload.length >= 4) {
      const detail = payload[2] | (payload[3] << 8)
      this.emit('error', { seq: payload[0], code: payload[1], detail })
    }
  }

  _writeFrame(frame) {
    if (!this.port) return false
    this.port.write(frame)
    return true
  }

  sendHoming(seqValue = this._nextSeq()) {
    const frame = this._buildFrame(CMD_HOMING_START, seqValue, Buffer.alloc(0))
    return this._writeFrame(frame)
  }

  sendMoveRad(rad, seqValue = this._nextSeq()) {
    // The controller only accepts a mechanical range of +/- 90 degrees.
    const clamped = clamp(rad, -Math.PI / 2, Math.PI / 2)
    // Hardware orientation is mirrored relative to logical heading commands.
    const hardwareRad = -clamped
    const q = q16_16(hardwareRad)
    const payload = Buffer.alloc(4)
    payload.writeInt32LE(q, 0)
    const frame = this._buildFrame(CMD_MOVE_TO_RAD, seqValue, payload)
    return this._writeFrame(frame)
  }
}

module.exports = {
  BaseStationAntenna,
  CMD_HOMING_START,
  CMD_MOVE_TO_RAD,
  CMD_ACK,
  CMD_DONE,
  CMD_ERROR,
}
