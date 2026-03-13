'use strict'

// Rocket M2 responses vary in shape and sometimes carry numeric values as
// strings. Keep parsing and status normalization isolated from the polling code.
function parseRocketM2Signal(raw) {
  const text = String(raw || '').trim()
  const parsed = JSON.parse(text)
  if (!parsed || typeof parsed !== 'object') {
    throw new Error('Invalid Rocket M2 response')
  }
  return {
    signal: toNumberOrNull(parsed.signal),
    rssi: toNumberOrNull(parsed.rssi),
    noisef: toNumberOrNull(parsed.noisef),
    chwidth: toNumberOrNull(parsed.chwidth),
    rx_chainmask: toNumberOrNull(parsed.rx_chainmask),
    chainrssi: toNumberArray(parsed.chainrssi),
    chainrssimgmt: toNumberArray(parsed.chainrssimgmt),
    chainrssiext: toNumberArray(parsed.chainrssiext),
  }
}

function createRocketM2Status(overrides = {}, options = {}) {
  const nowMs = Number.isFinite(options.nowMs) ? options.nowMs : Date.now()
  const lastSuccessMs = Number.isFinite(options.lastSuccessMs) ? options.lastSuccessMs : null
  return {
    connected: false,
    updated_at_ms: nowMs,
    last_success_ms: lastSuccessMs,
    signal: null,
    rssi: null,
    noisef: null,
    chwidth: null,
    rx_chainmask: null,
    chainrssi: [],
    chainrssimgmt: [],
    chainrssiext: [],
    error: null,
    ...overrides,
  }
}

function toNumberOrNull(value) {
  if (value == null) return null
  const num = Number(value)
  return Number.isFinite(num) ? num : null
}

function toNumberArray(value) {
  if (!Array.isArray(value)) return []
  return value
    .map((item) => (item == null ? null : Number(item)))
    .filter((item) => Number.isFinite(item))
}

function redactRocketM2Secrets(text) {
  if (!text) return ''
  return text.replace(/(password=)([^&\s]+)/gi, '$1***')
}

function formatRocketM2Error(err) {
  if (!err) return 'Unknown error'
  if (typeof err === 'string') return err
  const parts = []
  if (err.code != null) parts.push(`code=${err.code}`)
  if (err.signal) parts.push(`signal=${err.signal}`)
  if (err.killed) parts.push('killed')
  if (err.message) {
    const message = redactRocketM2Secrets(err.message.split('\n')[0])
    if (message) parts.push(message)
  }
  return parts.join(' ') || 'Unknown error'
}

module.exports = {
  createRocketM2Status,
  formatRocketM2Error,
  parseRocketM2Signal,
  redactRocketM2Secrets,
  toNumberArray,
  toNumberOrNull,
}
