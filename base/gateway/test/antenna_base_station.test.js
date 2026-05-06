'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const { EventEmitter } = require('events')

const { BaseStationAntenna, CMD_MOVE_TO_RAD } = require('../src/antenna/base_station')

class FakePort extends EventEmitter {
  constructor() {
    super()
    this.writes = []
  }

  write(buffer) {
    this.writes.push(buffer)
  }
}

function moveFramePayloadQ16(frame) {
  assert.equal(frame[0], 0xaa)
  assert.equal(frame[1], 0x55)
  assert.equal(frame[4], CMD_MOVE_TO_RAD)
  return frame.readInt32LE(5)
}

test('BaseStationAntenna clamps move commands to +/- 180 degrees', () => {
  const port = new FakePort()
  const antenna = new BaseStationAntenna({ port })

  assert.equal(antenna.sendMoveRad(Math.PI * 2, 7), true)
  assert.equal(port.writes.length, 1)
  assert.equal(moveFramePayloadQ16(port.writes[0]), Math.round(-Math.PI * 65536))

  assert.equal(antenna.sendMoveRad(-Math.PI * 2, 8), true)
  assert.equal(port.writes.length, 2)
  assert.equal(moveFramePayloadQ16(port.writes[1]), Math.round(Math.PI * 65536))
})
