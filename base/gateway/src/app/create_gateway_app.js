'use strict'

const http = require('http')
const { execFile } = require('child_process')
const { promisify } = require('util')

const { AntennaTracker } = require('../../antenna_tracker')
const {
  MsgId,
  createSequencer,
  decodeTelemBattery,
  decodeTelemNav,
  encodeCmdArmGripper,
  encodeCmdArmTwist,
  encodeCmdDrive,
  encodeHeartbeat,
  encodeMissionControl,
} = require('../protocol/sik')
const { createGatewayHttpHandler } = require('../runtime/http_handlers')
const { RocketM2Client } = require('../runtime/rocket_m2_client')
const { startRosBridge } = require('../runtime/ros_bridge')
const { SikSerialLink } = require('../runtime/serial_link')
const { createWsHub } = require('../runtime/ws_hub')

function createGatewayApp(options = {}) {
  const config = options.config
  const env = options.env || process.env
  const execFileAsync = options.execFileAsync || promisify(execFile)

  const nextSeq = createSequencer()
  const serialLink = new SikSerialLink({
    device: config.device,
    baud: config.baud,
    log,
  })

  let lastRxMs = 0
  let lastHeartbeatRxMs = 0
  let heartbeatTimer = null
  let antennaStatusTimer = null
  let antennaTracker = null
  let rosBridge = { stop() {} }
  let serverListening = false

  const rocketM2Client = new RocketM2Client({
    config,
    execFileAsync,
    log,
    onStatus: (status) => {
      wsHub.broadcast({ type: 'rocket_m2_status', ...status })
    },
  })

  const server = http.createServer(
    createGatewayHttpHandler({
      env,
      log,
      getRocketM2State: () => rocketM2Client.getState(),
    })
  )

  const wsHub = createWsHub({
    server,
    onMessage: handleDashboardMessage,
    getInitialMessages: () => {
      const messages = [getLinkStatus()]
      const rocketM2Status = rocketM2Client.getStatus()
      if (rocketM2Status) {
        messages.push({ type: 'rocket_m2_status', ...rocketM2Status })
      }
      return messages
    },
  })

  serialLink.on('frame', (msgId, payload) => {
    handleFrame(msgId, payload)
  })

  function log(message) {
    // eslint-disable-next-line no-console
    console.log(`[gateway] ${message}`)
  }

  function writeFrame(frame) {
    serialLink.write(frame)
  }

  function getLinkStatus() {
    return {
      type: 'link_status',
      connected: isLinkAlive(),
      last_rx_ms: lastRxMs,
      last_tx_ms: serialLink.getLastTxMs(),
    }
  }

  function isLinkAlive() {
    if (!serialLink.isReady()) return false
    if (lastHeartbeatRxMs === 0) return false
    return Date.now() - lastHeartbeatRxMs <= config.linkTimeoutMs
  }

  function coerceNumber(value) {
    const num = Number(value)
    return Number.isFinite(num) ? num : 0
  }

  function handleDashboardMessage(msg) {
    if (!msg || typeof msg !== 'object') return

    const type = msg.type || msg.event
    if (!type) return

    if (type === 'cmd_drive') {
      const linear = coerceNumber(msg.linear_x_m_s)
      const lateral = coerceNumber(msg.linear_y_m_s)
      const angular = coerceNumber(msg.angular_z_rad_s)
      log(`cmd_drive rx x=${linear} y=${lateral} yaw=${angular}`)
      writeFrame(
        encodeCmdDrive(
          {
            timestamp_ms: Date.now() >>> 0,
            linear_x_m_s: linear,
            linear_y_m_s: lateral,
            angular_z_rad_s: angular,
          },
          nextSeq
        )
      )
      return
    }

    if (type === 'cmd_arm_twist') {
      log(
        `cmd_arm_twist rx lin=(${coerceNumber(msg.lin_x_m_s)}, ${coerceNumber(
          msg.lin_y_m_s
        )}, ${coerceNumber(msg.lin_z_m_s)}) ang=(${coerceNumber(
          msg.ang_x_rad_s
        )}, ${coerceNumber(msg.ang_y_rad_s)}, ${coerceNumber(msg.ang_z_rad_s)})`
      )
      writeFrame(
        encodeCmdArmTwist(
          {
            timestamp_ms: Date.now() >>> 0,
            lin_x_m_s: coerceNumber(msg.lin_x_m_s),
            lin_y_m_s: coerceNumber(msg.lin_y_m_s),
            lin_z_m_s: coerceNumber(msg.lin_z_m_s),
            ang_x_rad_s: coerceNumber(msg.ang_x_rad_s),
            ang_y_rad_s: coerceNumber(msg.ang_y_rad_s),
            ang_z_rad_s: coerceNumber(msg.ang_z_rad_s),
          },
          nextSeq
        )
      )
      return
    }

    if (type === 'heartbeat') {
      writeFrame(
        encodeHeartbeat(
          {
            timestamp_ms: Date.now() >>> 0,
          },
          nextSeq
        )
      )
      return
    }

    if (type === 'mission_control') {
      const command = Math.min(255, Math.max(0, Math.floor(coerceNumber(msg.command))))
      const missionId = Math.min(
        0xffffffff,
        Math.max(0, Math.floor(coerceNumber(msg.mission_id)))
      )
      const clearCostmap = Boolean(msg.clear_costmap)
      log(
        `mission_control rx cmd=${command} clear=${clearCostmap ? 'true' : 'false'} mission_id=${missionId}`
      )
      writeFrame(
        encodeMissionControl(
          {
            command,
            clear_costmap: clearCostmap,
            mission_id: missionId,
          },
          nextSeq
        )
      )
      return
    }

    if (type === 'cmd_arm_gripper') {
      const positionNorm = coerceNumber(msg.position_norm)
      log(`cmd_arm_gripper rx pos_norm=${positionNorm}`)
      writeFrame(
        encodeCmdArmGripper(
          {
            timestamp_ms: Date.now() >>> 0,
            position_norm: positionNorm,
          },
          nextSeq
        )
      )
      return
    }

    if (type === 'base_heading' && antennaTracker) {
      antennaTracker.setBaseHeadingOffsetDeg(coerceNumber(msg.heading_deg))
    }
  }

  function handleFrame(msgId, payload) {
    lastRxMs = Date.now()

    if (msgId === MsgId.HEARTBEAT) {
      lastHeartbeatRxMs = Date.now()
    }

    if (msgId === MsgId.TELEM_BATTERY_1 || msgId === MsgId.TELEM_BATTERY_2) {
      const telem = decodeTelemBattery(payload)
      if (!telem) return
      const batteryId = msgId === MsgId.TELEM_BATTERY_2 ? 2 : 1
      wsHub.broadcast({ type: 'telem_battery', battery_id: batteryId, ...telem })
      return
    }

    if (msgId === MsgId.TELEM_NAV) {
      const nav = decodeTelemNav(payload)
      if (!nav) return
      if (antennaTracker) {
        antennaTracker.updateRoverNav(nav)
      }
      wsHub.broadcast({ type: 'telem_nav', ...nav })
    }
  }

  async function start() {
    serialLink.start()

    if (config.antennaEnable) {
      antennaTracker = new AntennaTracker({
        enabled: true,
        device: config.antennaDevice,
        baud: config.antennaBaud,
        cmdHz: config.antennaCmdHz,
        staleMs: config.antennaStaleMs,
        autoHome: config.antennaHome,
        maxRad: (Math.max(config.antennaMaxDeg, 0) * Math.PI) / 180,
        smoothing: config.antennaSmoothing,
        bootWaitMs: config.antennaBootWaitMs,
        logHeadingMs: config.antennaLogMs,
        allowProvisional: config.antennaAllowProvisional,
        headingOffsetDeg: config.baseHeadingOffsetDeg,
        log,
      })
      await antennaTracker.start()

      if (config.antennaStatusMs > 0) {
        antennaStatusTimer = setInterval(() => {
          if (!antennaTracker) return
          wsHub.broadcast({ type: 'base_status', ...antennaTracker.getStatus() })
        }, Math.max(config.antennaStatusMs, 200))
      }
    }

    rosBridge = await startRosBridge({
      nextSeq,
      writeFrame,
      log,
      onBaseSurveyIn: (msg) => {
        if (antennaTracker) {
          antennaTracker.updateBaseSurveyIn(msg)
        }
      },
    })

    await new Promise((resolve) => {
      server.listen(config.port, () => {
        serverListening = true
        log(`HTTP/WebSocket listening on ${config.port}`)
        resolve()
      })
    })

    rocketM2Client.start()

    if (config.heartbeatHz > 0) {
      const periodMs = Math.max(1000 / config.heartbeatHz, 100)
      heartbeatTimer = setInterval(() => {
        writeFrame(
          encodeHeartbeat(
            {
              timestamp_ms: Date.now() >>> 0,
            },
            nextSeq
          )
        )
        wsHub.broadcast(getLinkStatus())
      }, periodMs)
    }
  }

  async function stop() {
    if (heartbeatTimer) {
      clearInterval(heartbeatTimer)
      heartbeatTimer = null
    }
    if (antennaStatusTimer) {
      clearInterval(antennaStatusTimer)
      antennaStatusTimer = null
    }

    rocketM2Client.stop()

    if (antennaTracker) {
      antennaTracker.stop()
      antennaTracker = null
    }

    if (rosBridge) {
      rosBridge.stop()
      rosBridge = null
    }

    serialLink.stop()
    wsHub.close()

    if (serverListening) {
      await new Promise((resolve) => {
        server.close(() => {
          serverListening = false
          resolve()
        })
      })
    }
  }

  return {
    start,
    stop,
  }
}

module.exports = {
  createGatewayApp,
}
