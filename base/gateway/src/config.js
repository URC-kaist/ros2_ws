'use strict'

const fs = require('fs')
const path = require('path')

const DEFAULTS = {
  device: '/dev/ttySIK',
  baud: 57600,
  port: 8081,
  heartbeatHz: 2,
  linkTimeoutMs: 2000,
  antennaEnable: false,
  antennaDevice: '/dev/ttyARDUINO',
  antennaBaud: 115200,
  antennaCmdHz: 2,
  antennaStaleMs: 5000,
  antennaHome: true,
  antennaMaxDeg: 90,
  antennaSmoothing: 0,
  antennaBootWaitMs: 2000,
  antennaLogMs: 5000,
  antennaStatusMs: 1000,
  antennaAllowProvisional: true,
  baseHeadingOffsetDeg: 0,
  rocketM2Enable: false,
  rocketM2Ip: '',
  rocketM2User: '',
  rocketM2Pass: '',
  rocketM2PollMs: 5000,
  rocketM2TimeoutMs: 4000,
}

function loadGatewayEnv(baseDir) {
  // Delay loading dotenv so pure config parsing can be tested without installed deps.
  const dotenv = require('dotenv')
  const envLocalPath = path.join(baseDir, '.env.local')
  const envPath = path.join(baseDir, '.env')
  if (fs.existsSync(envLocalPath)) {
    dotenv.config({ path: envLocalPath })
    return envLocalPath
  }
  if (fs.existsSync(envPath)) {
    dotenv.config({ path: envPath })
    return envPath
  }
  return null
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
  return {
    device: getArg(args, '--device') || env.SIK_DEVICE || DEFAULTS.device,
    baud: toInt(getArg(args, '--baud') || env.SIK_BAUD || DEFAULTS.baud),
    port: toInt(getArg(args, '--port') || env.SIK_WS_PORT || DEFAULTS.port),
    heartbeatHz: toFloat(
      getArg(args, '--heartbeat-hz') || env.SIK_HEARTBEAT_HZ || DEFAULTS.heartbeatHz
    ),
    linkTimeoutMs: toInt(
      getArg(args, '--link-timeout-ms') ||
        env.SIK_LINK_TIMEOUT_MS ||
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
    rocketM2Ip: getArg(args, '--rocket-m2-ip') || env.ROCKET_M2_IP || DEFAULTS.rocketM2Ip,
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
