'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const { EventEmitter } = require('events')

const { createMavproxyArgs, startMavproxy } = require('../src/runtime/mavproxy')

function createConfig(overrides = {}) {
  return {
    mavproxyEnable: true,
    mavproxyBinary: 'mavproxy.py',
    mavproxyMasterDevice: '/dev/ttySIK',
    mavproxyMasterBaud: 57600,
    mavproxyOut: 'udp:192.168.1.108:14550',
    mavproxyDefaultModules: '',
    ...overrides,
  }
}

test('createMavproxyArgs builds the startup command arguments', () => {
  assert.deepEqual(createMavproxyArgs(createConfig()), [
    '--master=/dev/ttySIK,57600',
    '--out=udp:192.168.1.108:14550',
    '--default-modules=',
    '--non-interactive',
  ])
})

test('startMavproxy spawns MAVProxy and stops it with SIGTERM', () => {
  const calls = []
  const child = new EventEmitter()
  child.stdout = new EventEmitter()
  child.stderr = new EventEmitter()
  child.killed = false
  child.kill = (signal) => {
    child.killed = true
    child.signal = signal
  }
  const logs = []

  const runtime = startMavproxy({
    config: createConfig(),
    log: (message) => logs.push(message),
    spawn: (binary, args, options) => {
      calls.push({ binary, args, options })
      return child
    },
  })

  assert.equal(calls[0].binary, 'mavproxy.py')
  assert.deepEqual(calls[0].args, [
    '--master=/dev/ttySIK,57600',
    '--out=udp:192.168.1.108:14550',
    '--default-modules=',
    '--non-interactive',
  ])
  assert.equal(calls[0].options.stdio[0], 'ignore')
  assert.match(logs[0], /MAVProxy starting/)

  runtime.stop()

  assert.equal(child.killed, true)
  assert.equal(child.signal, 'SIGTERM')
})

test('startMavproxy is a no-op when disabled', () => {
  let spawned = false
  const runtime = startMavproxy({
    config: createConfig({ mavproxyEnable: false }),
    spawn: () => {
      spawned = true
    },
  })

  runtime.stop()

  assert.equal(spawned, false)
})
