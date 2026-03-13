'use strict'

const { encodeBaseRtcm, encodeBaseSvin } = require('../protocol/sik')

async function startRosTopicRelay(options = {}) {
  const nextSeq = options.nextSeq
  const writeFrame = options.writeFrame
  const log = typeof options.log === 'function' ? options.log : () => {}
  const onBaseSurveyIn =
    typeof options.onBaseSurveyIn === 'function' ? options.onBaseSurveyIn : null

  let rclnodejs = options.rclnodejs
  if (!rclnodejs) {
    try {
      // eslint-disable-next-line global-require
      rclnodejs = require('rclnodejs')
    } catch (err) {
      log(`ROS topic relay disabled: rclnodejs not available (${err.message})`)
      return { stop() {} }
    }
  }

  let ownsRosContext = false
  try {
    if (!rclnodejs.isInitialized || !rclnodejs.isInitialized()) {
      await rclnodejs.init()
      ownsRosContext = true
    }
  } catch (err) {
    if (!/already been initialized/i.test(err.message || '')) {
      log(`ROS topic relay init failed: ${err.message}`)
      return { stop() {} }
    }
  }

  let lastSvinTxMs = 0
  const nodeName = `gateway_ros_topics_${process.pid || Math.floor(Math.random() * 1e5)}`
  const rosNode = new rclnodejs.Node(nodeName)

  rosNode.createSubscription(
    'ublox_ubx_msgs/msg/UBXNavSvin',
    '/base/ubx_nav_svin',
    (msg) => {
      if (!msg) return
      if (onBaseSurveyIn) {
        onBaseSurveyIn(msg)
      }
      const nowMs = Date.now()
      if (nowMs - lastSvinTxMs < 500) return
      lastSvinTxMs = nowMs
      const frame = encodeBaseSvin(
        {
          mean_x_cm: msg.mean_x,
          mean_y_cm: msg.mean_y,
          mean_z_cm: msg.mean_z,
          mean_x_hp: msg.mean_x_hp,
          mean_y_hp: msg.mean_y_hp,
          mean_z_hp: msg.mean_z_hp,
          valid: !!msg.valid,
          active: !!msg.active,
          mean_acc_0p1mm: msg.mean_acc >>> 0,
          obs: msg.obs >>> 0,
        },
        nextSeq
      )
      if (frame) writeFrame(frame)
    }
  )

  rosNode.createSubscription('rtcm_msgs/msg/Message', '/base/rtcm', (msg) => {
    if (!msg) return
    const frames = encodeBaseRtcm(msg, nextSeq, { log })
    for (const frame of frames) {
      writeFrame(frame)
    }
  })

  rclnodejs.spin(rosNode)
  log('ROS topic relay started (SVIN + RTCM over SiK)')

  return {
    async stop() {
      try {
        if (typeof rosNode.destroy === 'function') {
          rosNode.destroy()
        }
      } catch (_) {
        /* ignore */
      }
      if (!ownsRosContext || typeof rclnodejs.shutdown !== 'function') {
        return
      }
      try {
        if (!rclnodejs.isInitialized || rclnodejs.isInitialized()) {
          await rclnodejs.shutdown()
        }
      } catch (_) {
        /* ignore */
      }
    },
  }
}

module.exports = {
  startRosTopicRelay,
}
