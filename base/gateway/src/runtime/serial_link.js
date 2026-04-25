'use strict'

const { EventEmitter } = require('events')

const { consumeFrames } = require('../protocol/xbee')

// Reconnecting serial wrapper around the XBEE link. It emits decoded protocol
// frames upward so the app layer does not deal with byte buffering.
class XbeeSerialLink extends EventEmitter {
  constructor(options = {}) {
    super()
    this.device = options.device
    this.baud = options.baud
    this.log = typeof options.log === 'function' ? options.log : () => {}
    this.SerialPortImpl = options.SerialPortImpl || null

    this.port = null
    this.portReconnectTimer = null
    this.rxBuffer = Buffer.alloc(0)
    this.serialReady = false
    this.lastTxMs = 0
    this.stopped = true
  }

  start() {
    this.stopped = false
    this._setupPort()
  }

  stop() {
    this.stopped = true
    if (this.portReconnectTimer) {
      clearTimeout(this.portReconnectTimer)
      this.portReconnectTimer = null
    }
    this.serialReady = false
    if (!this.port) return

    try {
      this.port.removeAllListeners()
      this.port.destroy()
    } catch (_) {
      /* ignore */
    }
    this.port = null
  }

  isReady() {
    return this.serialReady
  }

  getLastTxMs() {
    return this.lastTxMs
  }

  write(frame) {
    if (!this.serialReady || !this.port) return false
    this.port.write(frame)
    this.lastTxMs = Date.now()
    return true
  }

  _scheduleReconnect() {
    if (this.stopped || this.portReconnectTimer) return
    this.portReconnectTimer = setTimeout(() => {
      this.portReconnectTimer = null
      this._setupPort()
    }, 1000)
  }

  _setupPort() {
    if (this.stopped) return
    if (this.port) {
      try {
        this.port.removeAllListeners()
        this.port.destroy()
      } catch (_) {
        /* ignore */
      }
      this.port = null
    }

    const SerialPortImpl = this._getSerialPortImpl()
    this.port = new SerialPortImpl({
      path: this.device,
      baudRate: this.baud,
      autoOpen: true,
    })

    this.port.on('open', () => {
      this.serialReady = true
      this.rxBuffer = Buffer.alloc(0)
      this.log(`Serial open ${this.device} @ ${this.baud}`)
      this.emit('open')
    })

    this.port.on('error', (err) => {
      this.serialReady = false
      this.log(`Serial error: ${err.message}`)
      this.emit('serial_error', err)
      this._scheduleReconnect()
    })

    this.port.on('close', () => {
      this.serialReady = false
      this.log('Serial closed')
      this.emit('close')
      this._scheduleReconnect()
    })

    this.port.on('data', (data) => {
      this.rxBuffer = Buffer.concat([this.rxBuffer, data])
      // Preserve incomplete trailing bytes so fragmented serial reads still
      // reconstruct valid XBEE frames.
      this.rxBuffer = consumeFrames(this.rxBuffer, (msgId, payload) => {
        this.emit('frame', msgId, payload)
      })
    })
  }

  _getSerialPortImpl() {
    if (this.SerialPortImpl) return this.SerialPortImpl
    // Delay loading the serialport package so tests can inject a fake implementation.
    const { SerialPort } = require('serialport')
    this.SerialPortImpl = SerialPort
    return this.SerialPortImpl
  }
}

module.exports = {
  XbeeSerialLink,
}
