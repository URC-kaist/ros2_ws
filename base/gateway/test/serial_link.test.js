'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const { EventEmitter } = require('events')

const { encodeFrame, MsgId } = require('../src/protocol/sik')
const { SikSerialLink } = require('../src/runtime/serial_link')

class FakeSerialPort extends EventEmitter {
  constructor(options) {
    super()
    this.options = options
    this.writes = []
    this.destroyed = false
  }

  write(buffer) {
    this.writes.push(buffer)
  }

  destroy() {
    this.destroyed = true
  }
}

test('SikSerialLink emits decoded frames and tracks writes', () => {
  const link = new SikSerialLink({
    device: '/tmp/fake',
    baud: 57600,
    SerialPortImpl: FakeSerialPort,
  })

  const seen = []
  link.on('frame', (msgId, payload) => {
    seen.push({ msgId, payload })
  })

  link.start()
  link.port.emit('open')

  const frame = encodeFrame(MsgId.HEARTBEAT, 5, Buffer.from([1, 2, 3, 4]))
  link.port.emit('data', frame)

  assert.equal(seen.length, 1)
  assert.equal(seen[0].msgId, MsgId.HEARTBEAT)
  assert.equal(Buffer.compare(seen[0].payload, Buffer.from([1, 2, 3, 4])), 0)

  const written = link.write(Buffer.from([9, 8, 7]))
  assert.equal(written, true)
  assert.equal(link.port.writes.length, 1)

  link.stop()
  assert.equal(link.port, null)
})

test('SikSerialLink refuses writes before open', () => {
  const link = new SikSerialLink({
    device: '/tmp/fake',
    baud: 57600,
    SerialPortImpl: FakeSerialPort,
  })

  link.start()
  assert.equal(link.write(Buffer.from([1])), false)
})
