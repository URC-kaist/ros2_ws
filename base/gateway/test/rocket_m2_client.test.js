'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { RocketM2Client } = require('../src/runtime/rocket_m2_client')

function createConfig(overrides = {}) {
  return {
    rocketM2Enable: true,
    rocketM2Ip: '192.168.1.20',
    rocketM2User: 'ubnt',
    rocketM2Pass: 'ubnt',
    rocketM2PollMs: 5000,
    rocketM2TimeoutMs: 4000,
    ...overrides,
  }
}

test('RocketM2Client getState reflects configuration', () => {
  const client = new RocketM2Client({
    config: createConfig({ rocketM2Enable: false, rocketM2Ip: '', rocketM2User: '', rocketM2Pass: '' }),
    execFileAsync: async () => ({ stdout: '' }),
  })

  assert.deepEqual(client.getState(), {
    target: 'base',
    label: 'Base',
    enabled: false,
    configured: false,
    status: null,
  })
})

test('RocketM2Client does not auto-enable configured radios when profile disables it', () => {
  const client = new RocketM2Client({
    config: createConfig({
      rocketM2Enable: false,
      rocketM2AutoEnable: false,
    }),
    execFileAsync: async () => ({ stdout: '' }),
  })

  assert.deepEqual(client.getState(), {
    target: 'base',
    label: 'Base',
    enabled: false,
    configured: true,
    status: null,
  })
})

test('RocketM2Client poll publishes successful status', async () => {
  const statuses = []
  const execCalls = []
  const client = new RocketM2Client({
    config: createConfig(),
    execFileAsync: async (_cmd, args) => {
      execCalls.push(args)
      if (args.includes('/dev/null')) {
        return { stdout: '' }
      }
      return {
        stdout: JSON.stringify({
          signal: '-62',
          rssi: '-61',
          noisef: '-95',
          chwidth: '20',
          rx_chainmask: '3',
          chainrssi: ['-61', '-63'],
          chainrssimgmt: [],
          chainrssiext: [],
        }),
      }
    },
    onStatus: (status) => statuses.push(status),
  })

  client.cookiePath = '/tmp/test.cookies'
  await client.poll()

  assert.equal(execCalls.length, 2)
  assert.equal(statuses.length, 1)
  assert.equal(statuses[0].target, 'base')
  assert.equal(statuses[0].label, 'Base')
  assert.equal(statuses[0].connected, true)
  assert.equal(statuses[0].signal, -62)
})

test('RocketM2Client poll preserves previous radio metrics on failure', async () => {
  const statuses = []
  const client = new RocketM2Client({
    config: createConfig(),
    execFileAsync: async () => {
      throw new Error('curl failed password=secret123')
    },
    onStatus: (status) => statuses.push(status),
  })

  client.cookiePath = '/tmp/test.cookies'
  client.status = {
    connected: true,
    updated_at_ms: 10,
    last_success_ms: 10,
    signal: -60,
    rssi: -59,
    noisef: -95,
    chwidth: 20,
    rx_chainmask: 3,
    chainrssi: [-60, -61],
    chainrssimgmt: [],
    chainrssiext: [],
    error: null,
  }
  client.lastSuccessMs = 10

  await client.poll()

  assert.equal(statuses.length, 1)
  assert.equal(statuses[0].connected, false)
  assert.equal(statuses[0].signal, -60)
  assert.doesNotMatch(statuses[0].error, /secret123/)
})
