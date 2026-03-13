'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const { startRosBridge } = require('../src/runtime/ros_bridge')

function createRclnodejs({ initialized = false } = {}) {
  let isInitialized = initialized
  const rosNode = {
    subscriptions: [],
    createSubscription(type, topic, callback) {
      this.subscriptions.push({ type, topic, callback })
    },
    destroyCalled: false,
    destroy() {
      this.destroyCalled = true
    },
  }

  function Node() {
    return rosNode
  }

  return {
    rosNode,
    api: {
      initCalls: 0,
      shutdownCalls: 0,
      Node,
      spin() {},
      isInitialized() {
        return isInitialized
      },
      async init() {
        this.initCalls += 1
        isInitialized = true
      },
      async shutdown() {
        this.shutdownCalls += 1
        isInitialized = false
      },
    },
  }
}

test('startRosBridge shuts down ROS when it initialized the context', async () => {
  const { api, rosNode } = createRclnodejs({ initialized: false })
  const bridge = await startRosBridge({
    rclnodejs: api,
    nextSeq: () => 1,
    writeFrame() {},
  })

  assert.equal(api.initCalls, 1)
  await bridge.stop()

  assert.equal(rosNode.destroyCalled, true)
  assert.equal(api.shutdownCalls, 1)
})

test('startRosBridge leaves shared ROS contexts running', async () => {
  const { api, rosNode } = createRclnodejs({ initialized: true })
  const bridge = await startRosBridge({
    rclnodejs: api,
    nextSeq: () => 1,
    writeFrame() {},
  })

  assert.equal(api.initCalls, 0)
  await bridge.stop()

  assert.equal(rosNode.destroyCalled, true)
  assert.equal(api.shutdownCalls, 0)
})
