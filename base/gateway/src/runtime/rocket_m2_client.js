'use strict'

const os = require('os')
const path = require('path')

const {
  createRocketM2Status,
  formatRocketM2Error,
  parseRocketM2Signal,
} = require('../rocket_m2')

// Poll the Rocket M2 management interface through curl so we can tolerate its
// older TLS stack and preserve the latest known status for the dashboard.
class RocketM2Client {
  constructor(options = {}) {
    this.config = options.config
    this.execFileAsync = options.execFileAsync
    this.log = typeof options.log === 'function' ? options.log : () => {}
    this.onStatus = typeof options.onStatus === 'function' ? options.onStatus : () => {}
    this.target = options.target || this.config.rocketM2Target || 'base'
    this.label = options.label || this.config.rocketM2Label || 'Base'

    this.status = null
    this.lastSuccessMs = 0
    this.pollTimer = null
    this.pollInFlight = false
    this.cookiePath = null
    this.lastError = null
  }

  getState() {
    const configured =
      this.config.rocketM2Ip && this.config.rocketM2User && this.config.rocketM2Pass
    const autoEnable = this.config.rocketM2AutoEnable !== false
    return {
      target: this.target,
      label: this.label,
      enabled: Boolean(this.config.rocketM2Enable || (autoEnable && configured)),
      configured: Boolean(configured),
      status: this.status,
    }
  }

  getStatus() {
    return this.status
  }

  start() {
    const state = this.getState()
    if (!state.enabled) return
    if (!state.configured) {
      this.log(
        `Rocket M2 ${this.label} enabled but missing target IP/ROCKET_M2_USER/ROCKET_M2_PASS`
      )
      return
    }

    const pollMs = Math.max(this.config.rocketM2PollMs, 0)
    if (pollMs <= 0) {
      this.log(`Rocket M2 ${this.label} polling disabled (interval <= 0)`)
      return
    }

    if (!this.cookiePath) {
      this.cookiePath = path.join(os.tmpdir(), `rocket_m2_${this.target}_${process.pid}.cookies`)
    }

    this.poll()
    this.pollTimer = setInterval(() => this.poll(), Math.max(pollMs, 500))
    this.log(`Rocket M2 ${this.label} polling every ${Math.max(pollMs, 500)} ms`)
  }

  stop() {
    if (this.pollTimer) {
      clearInterval(this.pollTimer)
      this.pollTimer = null
    }
  }

  async poll() {
    if (this.pollInFlight) return
    this.pollInFlight = true
    const nowMs = Date.now()

    try {
      const data = await this.fetchSignal()
      this.lastSuccessMs = nowMs
      this.status = createRocketM2Status(
        {
          connected: true,
          target: this.target,
          label: this.label,
          updated_at_ms: nowMs,
          last_success_ms: nowMs,
          error: null,
          ...data,
        },
        {
          nowMs,
          lastSuccessMs: nowMs,
        }
      )
      if (this.lastError) {
        this.log(`Rocket M2 ${this.label} polling recovered`)
        this.lastError = null
      }
    } catch (err) {
      const error = formatRocketM2Error(err)
      const previous = this.status
      // Keep the last successful radio metrics visible even while polling fails.
      this.status = createRocketM2Status(
        {
          connected: false,
          target: this.target,
          label: this.label,
          updated_at_ms: nowMs,
          last_success_ms: this.lastSuccessMs || null,
          signal: previous?.signal ?? null,
          rssi: previous?.rssi ?? null,
          noisef: previous?.noisef ?? null,
          chwidth: previous?.chwidth ?? null,
          rx_chainmask: previous?.rx_chainmask ?? null,
          chainrssi: previous?.chainrssi ?? [],
          chainrssimgmt: previous?.chainrssimgmt ?? [],
          chainrssiext: previous?.chainrssiext ?? [],
          error,
        },
        {
          nowMs,
          lastSuccessMs: this.lastSuccessMs || null,
        }
      )
      if (error && error !== this.lastError) {
        this.log(`Rocket M2 ${this.label} poll failed: ${error}`)
        this.lastError = error
      }
    } finally {
      this.pollInFlight = false
    }

    if (this.status) {
      this.onStatus(this.status)
    }
  }

  async fetchSignal() {
    if (!this.cookiePath) {
      throw new Error('Rocket M2 cookie path not initialized')
    }

    // The device expects an authenticated cookie-backed session before
    // requesting signal.cgi.
    const loginPayload = new URLSearchParams({
      username: this.config.rocketM2User,
      password: this.config.rocketM2Pass,
      uri: '/index.cgi',
    }).toString()

    await this.runCurl(
      [
        '-k',
        '--ciphers',
        'DEFAULT:@SECLEVEL=0',
        '-sS',
        '-L',
        '-c',
        this.cookiePath,
        '-b',
        this.cookiePath,
        '-X',
        'POST',
        `https://${this.config.rocketM2Ip}/login.cgi`,
        '-H',
        'Content-Type: application/x-www-form-urlencoded',
        '--data',
        loginPayload,
        '-o',
        '/dev/null',
      ],
      this.config.rocketM2TimeoutMs
    )

    const signalRaw = await this.runCurl(
      [
        '-k',
        '--ciphers',
        'DEFAULT:@SECLEVEL=0',
        '-sS',
        '-L',
        '-b',
        this.cookiePath,
        '-c',
        this.cookiePath,
        '-H',
        'Accept: application/json',
        `https://${this.config.rocketM2Ip}/signal.cgi?_=${Date.now()}`,
      ],
      this.config.rocketM2TimeoutMs
    )

    return parseRocketM2Signal(signalRaw)
  }

  runCurl(args, timeoutMs) {
    return this.execFileAsync('curl', args, {
      timeout: Math.max(timeoutMs || 0, 1000),
      maxBuffer: 1024 * 1024,
      encoding: 'utf8',
    }).then(({ stdout }) => stdout)
  }
}

module.exports = {
  RocketM2Client,
}
