'use strict'

const { BaseStationAntenna } = require('./base_station')

const WGS84_A = 6378137.0
const WGS84_F = 1 / 298.257223563
const WGS84_B = WGS84_A * (1 - WGS84_F)
const WGS84_E2 = 1 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A)

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max)
}

function wrapToPi(rad) {
  const twoPi = Math.PI * 2
  let out = ((rad + Math.PI) % twoPi + twoPi) % twoPi - Math.PI
  return out
}

function normalizeHeadingDeg(deg) {
  const mod = ((deg % 360) + 360) % 360
  return mod
}

function degToRad(deg) {
  return (deg * Math.PI) / 180
}

function ecefToLlh(x, y, z) {
  const p = Math.hypot(x, y)
  let lat = Math.atan2(z, p * (1 - WGS84_E2))
  for (let i = 0; i < 10; i += 1) {
    const sinLat = Math.sin(lat)
    const n = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat)
    const h = p / Math.cos(lat) - n
    const nextLat = Math.atan2(z, p * (1 - (WGS84_E2 * n) / (n + h)))
    if (Math.abs(nextLat - lat) < 1e-12) {
      lat = nextLat
      break
    }
    lat = nextLat
  }
  const sinLat = Math.sin(lat)
  const n = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat)
  const h = p / Math.cos(lat) - n
  const lon = Math.atan2(y, x)
  return {
    latDeg: (lat * 180) / Math.PI,
    lonDeg: (lon * 180) / Math.PI,
    altM: h,
  }
}

function bearingRad(lat1, lon1, lat2, lon2) {
  const dLon = lon2 - lon1
  const y = Math.sin(dLon) * Math.cos(lat2)
  const x =
    Math.cos(lat1) * Math.sin(lat2) -
    Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon)
  let az = Math.atan2(y, x)
  if (az < 0) az += Math.PI * 2
  return az
}

// Convert base survey-in and rover navigation into bounded antenna heading
// commands for the simple single-axis base antenna controller.
class AntennaTracker {
  constructor(options = {}) {
    this.enabled = options.enabled === true
    this.device = options.device || null
    this.baud = options.baud || 115200
    this.cmdHz = Number.isFinite(options.cmdHz) ? options.cmdHz : 2
    this.staleMs = Number.isFinite(options.staleMs) ? options.staleMs : 5000
    this.headingOffsetDeg = Number.isFinite(options.headingOffsetDeg)
      ? normalizeHeadingDeg(options.headingOffsetDeg)
      : 0
    this.maxRad = Number.isFinite(options.maxRad) ? options.maxRad : Math.PI / 2
    this.smoothing = Number.isFinite(options.smoothing) ? options.smoothing : 0
    this.autoHome = options.autoHome === true
    this.bootWaitMs = Number.isFinite(options.bootWaitMs) ? options.bootWaitMs : 2000
    this.logHeadingMs = Number.isFinite(options.logHeadingMs) ? options.logHeadingMs : 5000
    this.allowProvisional = options.allowProvisional !== false
    this.log = typeof options.log === 'function' ? options.log : () => {}

    this.antenna = null
    this.timer = null
    this.lastCmdRad = null
    this.rover = null
    this.base = null
    this.antennaReady = false
    this.lastHeadingLogMs = 0
    this.lastCmdMs = 0
    this.idleReason = 'not started'
    this.baseSvinValid = false
    this.baseSvinActive = false
  }

  async start() {
    if (!this.enabled) return
    if (!this.device) {
      this.log('Antenna tracker disabled: no antenna device configured')
      return
    }

    this.antenna = new BaseStationAntenna({
      device: this.device,
      baud: this.baud,
    })
    this.antenna.on('error', (msg) => {
      const seq = msg?.seq ?? 'unknown'
      const code = msg?.code ?? 'unknown'
      const detail = msg?.detail ?? 'unknown'
      this.log(`Antenna error seq=${seq} code=${code} detail=${detail}`)
    })
    this.antenna.on('serial_error', (err) => {
      this.log(`Antenna serial error: ${err?.message || err}`)
    })
    this.antenna.on('open', () => {
      this.antennaReady = true
      this.idleReason = 'serial ready'
      if (this.autoHome) {
        setTimeout(() => {
          if (this.antenna) {
            this.antenna.sendHoming()
          }
        }, Math.max(this.bootWaitMs, 0))
      }
    })

    const periodMs = Math.max(1000 / Math.max(this.cmdHz, 0.1), 100)
    this.timer = setInterval(() => this.tick_(), periodMs)
  }

  stop() {
    if (this.timer) {
      clearInterval(this.timer)
      this.timer = null
    }
  }

  updateRoverNav(nav) {
    if (!nav) return
    const lat = Number(nav.latitude_deg)
    const lon = Number(nav.longitude_deg)
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return
    this.rover = {
      latDeg: lat,
      lonDeg: lon,
      altM: Number(nav.altitude_m) || 0,
      stampMs: Date.now(),
    }
  }

  setBaseHeadingOffsetDeg(deg) {
    if (!Number.isFinite(deg)) return
    this.headingOffsetDeg = normalizeHeadingDeg(deg)
  }

  updateBaseSurveyIn(msg) {
    if (!msg) return
    const svinValid = !!msg.valid
    const svinActive = !!msg.active
    this.baseSvinValid = svinValid
    this.baseSvinActive = svinActive
    if (!svinValid && !this.allowProvisional) return
    const meanX = Number(msg.mean_x)
    const meanY = Number(msg.mean_y)
    const meanZ = Number(msg.mean_z)
    const meanXHp = Number(msg.mean_x_hp) || 0
    const meanYHp = Number(msg.mean_y_hp) || 0
    const meanZHp = Number(msg.mean_z_hp) || 0
    if (!Number.isFinite(meanX) || !Number.isFinite(meanY) || !Number.isFinite(meanZ)) {
      return
    }

    // UBX survey-in reports ECEF in centimeters plus high-precision 0.01 cm
    // components. Convert to meters before projecting into latitude/longitude.
    const xM = (meanX + 0.01 * meanXHp) / 100.0
    const yM = (meanY + 0.01 * meanYHp) / 100.0
    const zM = (meanZ + 0.01 * meanZHp) / 100.0
    const llh = ecefToLlh(xM, yM, zM)
    this.base = {
      latDeg: llh.latDeg,
      lonDeg: llh.lonDeg,
      altM: llh.altM,
      stampMs: Date.now(),
    }
  }

  tick_() {
    const now = Date.now()
    let logReason = null
    if (!this.antenna || !this.antennaReady) {
      logReason = 'serial not ready'
    } else if (!this.base) {
      logReason = 'no base survey-in fix'
    } else if (!this.rover) {
      logReason = 'no rover nav'
    } else if (now - this.rover.stampMs > this.staleMs) {
      logReason = 'rover nav stale'
    } else if (now - this.base.stampMs > this.staleMs) {
      logReason = 'base fix stale'
    }

    if (logReason) {
      this.idleReason = logReason
      if (this.logHeadingMs > 0 && now - this.lastHeadingLogMs >= this.logHeadingMs) {
        this.lastHeadingLogMs = now
        this.log(`Antenna idle: ${logReason}`)
      }
      return
    }

    const lat1 = degToRad(this.base.latDeg)
    const lon1 = degToRad(this.base.lonDeg)
    const lat2 = degToRad(this.rover.latDeg)
    const lon2 = degToRad(this.rover.lonDeg)
    const az = bearingRad(lat1, lon1, lat2, lon2)
    const offsetRad = degToRad(this.headingOffsetDeg)
    let desired = wrapToPi(az - offsetRad)
    desired = clamp(desired, -this.maxRad, this.maxRad)

    // Optional smoothing trades responsiveness for less servo chatter.
    if (this.smoothing > 0 && this.smoothing < 1 && this.lastCmdRad != null) {
      desired = this.lastCmdRad + (desired - this.lastCmdRad) * this.smoothing
    }

    this.lastCmdRad = desired
    this.lastCmdMs = now
    this.idleReason = 'tracking'
    this.antenna.sendMoveRad(desired)

    if (this.logHeadingMs > 0 && now - this.lastHeadingLogMs >= this.logHeadingMs) {
      this.lastHeadingLogMs = now
      const headingDeg = (desired * 180) / Math.PI
      this.log(
        `Antenna heading cmd=${headingDeg.toFixed(1)} deg (offset=${this.headingOffsetDeg.toFixed(1)} deg)`
      )
    }
  }

  getStatus() {
    const now = Date.now()
    const baseAgeMs = this.base ? now - this.base.stampMs : null
    const roverAgeMs = this.rover ? now - this.rover.stampMs : null
    const lastCmdAgeMs = this.lastCmdMs ? now - this.lastCmdMs : null
    const antennaHeadingDeg =
      this.lastCmdRad != null
        ? normalizeHeadingDeg(this.headingOffsetDeg + (this.lastCmdRad * 180) / Math.PI)
        : null
    const baseFresh = baseAgeMs != null ? baseAgeMs <= this.staleMs : false
    const roverFresh = roverAgeMs != null ? roverAgeMs <= this.staleMs : false
    return {
      enabled: this.enabled,
      antenna_ready: this.antennaReady,
      auto_home: this.autoHome,
      heading_offset_deg: this.headingOffsetDeg,
      base_lat_deg: this.base ? this.base.latDeg : null,
      base_lon_deg: this.base ? this.base.lonDeg : null,
      base_alt_m: this.base ? this.base.altM : null,
      antenna_heading_deg: antennaHeadingDeg,
      last_cmd_heading_deg:
        this.lastCmdRad != null ? (this.lastCmdRad * 180) / Math.PI : null,
      last_cmd_age_ms: lastCmdAgeMs,
      base_fix_age_ms: baseAgeMs,
      rover_nav_age_ms: roverAgeMs,
      base_fix_valid: baseFresh && this.baseSvinValid,
      rover_nav_valid: roverFresh,
      idle_reason: this.idleReason,
    }
  }
}

module.exports = {
  AntennaTracker,
}
