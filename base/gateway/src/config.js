'use strict'

const fs = require('fs')
const path = require('path')

const DEFAULTS = {
  device: '/dev/ttyXBEE',
  host: '0.0.0.0',
  port: 8081,
  videoConfigPath: path.resolve(
    __dirname,
    '../../../rover/ros2_ws/src/mr2_launch/config/video_streams.json'
  ),
  videoGstBinary: 'gst-launch-1.0',
  videoJitterLatencyMs: 40,
  videoReceiverRestartMs: 1000,
  videoAvailabilityStaleMs: 1500,
  videoClientMaxBufferedBytes: 1048576,
  heartbeatHz: 2,
  linkTimeoutMs: 2000,
  antennaEnable: false,
  antennaDevice: '/dev/ttyARDUINO',
  antennaBaud: 115200,
  antennaCmdHz: 2,
  antennaStaleMs: 5000,
  antennaHome: true,
  antennaMaxDeg: 180,
  antennaSmoothing: 0,
  antennaBootWaitMs: 2000,
  antennaLogMs: 5000,
  antennaStatusMs: 1000,
  antennaAllowProvisional: true,
  baseHeadingOffsetDeg: 0,
  rocketM2Enable: false,
  rocketM2Ip: '',
  rocketM2Targets: [],
  rocketM2User: '',
  rocketM2Pass: '',
  rocketM2PollMs: 5000,
  rocketM2TimeoutMs: 4000,
}

function loadGatewayEnv(baseDir) {
  // Delay loading dotenv so pure config parsing can be tested without installed deps.
  const dotenv = require('dotenv')
  const repoRoot = path.resolve(baseDir, '../..')
  const repoEnvPath = path.join(repoRoot, '.env')
  const repoEnvLocalPath = path.join(repoRoot, '.env.local')
  const envLocalPath = path.join(baseDir, '.env.local')
  const envPath = path.join(baseDir, '.env')
  const loaded = []

  if (fs.existsSync(repoEnvPath)) {
    dotenv.config({ path: repoEnvPath })
    loaded.push(repoEnvPath)
  }
  if (fs.existsSync(repoEnvLocalPath)) {
    dotenv.config({ path: repoEnvLocalPath, override: true })
    loaded.push(repoEnvLocalPath)
  }
  if (fs.existsSync(envPath)) {
    dotenv.config({ path: envPath, override: true })
    loaded.push(envPath)
  }
  if (fs.existsSync(envLocalPath)) {
    dotenv.config({ path: envLocalPath, override: true })
    loaded.push(envLocalPath)
  }
  return loaded.length > 0 ? loaded : null
}

function getArg(args, name) {
  const index = args.indexOf(name)
  if (index === -1) return null
  const value = args[index + 1]
  if (!value || value.startsWith('--')) return ''
  return value
}

function toInt(value) {
  const parsed = Number.parseInt(String(value), 10)
  return Number.isFinite(parsed) ? parsed : 0
}

function toFloat(value) {
  const parsed = Number.parseFloat(String(value))
  return Number.isFinite(parsed) ? parsed : 0
}

function toBool(value) {
  if (value === true || value === false) return value
  const text = String(value).toLowerCase()
  return text === '1' || text === 'true' || text === 'yes' || text === 'on'
}

// Resolve runtime config with the standard precedence for this package:
// CLI flags override environment variables, which override hard-coded defaults.
function parseGatewayConfig(args = process.argv.slice(2), env = process.env) {
  const rocketM2Targets = [
    {
      target: 'base',
      label: 'Base',
      ip: getArg(args, '--base-rocket-m2-ip') || env.MR2_BASE_ROCKET_IP || '',
    },
    {
      target: 'drone',
      label: 'Drone',
      ip: getArg(args, '--drone-rocket-m2-ip') || env.MR2_DRONE_ROCKET_IP || '',
    },
    {
      target: 'rover',
      label: 'Rover',
      ip: getArg(args, '--rover-rocket-m2-ip') || env.MR2_ROVER_ROCKET_IP || '',
    },
  ]

  return {
    device:
      getArg(args, '--base-xbee-device') || env.BASE_XBEE_DEVICE || DEFAULTS.device,
    host:
      getArg(args, '--gateway-host') || env.MR2_GATEWAY_HOST || DEFAULTS.host,
    port: toInt(
      getArg(args, '--gateway-port') || env.MR2_GATEWAY_PORT || DEFAULTS.port
    ),
    videoConfigPath:
      getArg(args, '--video-config') || env.VIDEO_CONFIG_PATH || DEFAULTS.videoConfigPath,
    videoGstBinary:
      getArg(args, '--video-gst-binary') ||
      env.VIDEO_GST_BINARY ||
      DEFAULTS.videoGstBinary,
    videoJitterLatencyMs: toInt(
      getArg(args, '--video-jitter-ms') ||
        env.VIDEO_JITTER_LATENCY_MS ||
        DEFAULTS.videoJitterLatencyMs
    ),
    videoReceiverRestartMs: toInt(
      getArg(args, '--video-restart-ms') ||
        env.VIDEO_RECEIVER_RESTART_MS ||
        DEFAULTS.videoReceiverRestartMs
    ),
    videoAvailabilityStaleMs: toInt(
      getArg(args, '--video-availability-stale-ms') ||
        env.VIDEO_AVAILABILITY_STALE_MS ||
        DEFAULTS.videoAvailabilityStaleMs
    ),
    videoClientMaxBufferedBytes: toInt(
      getArg(args, '--video-client-max-buffered-bytes') ||
        env.VIDEO_CLIENT_MAX_BUFFERED_BYTES ||
        DEFAULTS.videoClientMaxBufferedBytes
    ),
    heartbeatHz: toFloat(
      getArg(args, '--base-xbee-heartbeat-hz') ||
        env.BASE_XBEE_HEARTBEAT_HZ ||
        DEFAULTS.heartbeatHz
    ),
    linkTimeoutMs: toInt(
      getArg(args, '--base-xbee-link-timeout-ms') ||
        env.BASE_XBEE_LINK_TIMEOUT_MS ||
        DEFAULTS.linkTimeoutMs
    ),
    antennaEnable: toBool(
      getArg(args, '--antenna-enable') ||
        env.BASE_ANTENNA_ENABLE ||
        DEFAULTS.antennaEnable
    ),
    antennaDevice:
      getArg(args, '--antenna-device') || env.BASE_ANTENNA_DEVICE || DEFAULTS.antennaDevice,
    antennaBaud: toInt(
      getArg(args, '--antenna-baud') || env.BASE_ANTENNA_BAUD || DEFAULTS.antennaBaud
    ),
    antennaCmdHz: toFloat(
      getArg(args, '--antenna-cmd-hz') || env.BASE_ANTENNA_CMD_HZ || DEFAULTS.antennaCmdHz
    ),
    antennaStaleMs: toInt(
      getArg(args, '--antenna-stale-ms') ||
        env.BASE_ANTENNA_STALE_MS ||
        DEFAULTS.antennaStaleMs
    ),
    antennaHome: toBool(
      getArg(args, '--antenna-home') || env.BASE_ANTENNA_HOME || DEFAULTS.antennaHome
    ),
    antennaMaxDeg: toFloat(
      getArg(args, '--antenna-max-deg') || env.BASE_ANTENNA_MAX_DEG || DEFAULTS.antennaMaxDeg
    ),
    antennaSmoothing: toFloat(
      getArg(args, '--antenna-smoothing') ||
        env.BASE_ANTENNA_SMOOTHING ||
        DEFAULTS.antennaSmoothing
    ),
    antennaBootWaitMs: toInt(
      getArg(args, '--antenna-boot-wait-ms') ||
        env.BASE_ANTENNA_BOOT_WAIT_MS ||
        DEFAULTS.antennaBootWaitMs
    ),
    antennaLogMs: toInt(
      getArg(args, '--antenna-log-ms') || env.BASE_ANTENNA_LOG_MS || DEFAULTS.antennaLogMs
    ),
    antennaStatusMs: toInt(
      getArg(args, '--antenna-status-ms') ||
        env.BASE_ANTENNA_STATUS_MS ||
        DEFAULTS.antennaStatusMs
    ),
    antennaAllowProvisional: toBool(
      getArg(args, '--antenna-allow-provisional') ||
        env.BASE_ANTENNA_ALLOW_PROVISIONAL ||
        DEFAULTS.antennaAllowProvisional
    ),
    baseHeadingOffsetDeg: toFloat(
      getArg(args, '--base-heading-deg') ||
        env.BASE_HEADING_OFFSET_DEG ||
        DEFAULTS.baseHeadingOffsetDeg
    ),
    rocketM2Enable: toBool(
      getArg(args, '--rocket-m2-enable') || env.ROCKET_M2_ENABLE || DEFAULTS.rocketM2Enable
    ),
    rocketM2Targets,
    rocketM2Ip:
      rocketM2Targets.find((target) => target.target === 'base')?.ip ||
      DEFAULTS.rocketM2Ip,
    rocketM2User:
      getArg(args, '--rocket-m2-user') || env.ROCKET_M2_USER || DEFAULTS.rocketM2User,
    rocketM2Pass:
      getArg(args, '--rocket-m2-pass') || env.ROCKET_M2_PASS || DEFAULTS.rocketM2Pass,
    rocketM2PollMs: toInt(
      getArg(args, '--rocket-m2-poll-ms') ||
        env.ROCKET_M2_POLL_MS ||
        DEFAULTS.rocketM2PollMs
    ),
    rocketM2TimeoutMs: toInt(
      getArg(args, '--rocket-m2-timeout-ms') ||
        env.ROCKET_M2_TIMEOUT_MS ||
        DEFAULTS.rocketM2TimeoutMs
    ),
  }
}

module.exports = {
  DEFAULTS,
  getArg,
  loadGatewayEnv,
  parseGatewayConfig,
  toBool,
  toFloat,
  toInt,
}
