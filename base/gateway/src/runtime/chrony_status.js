'use strict'

const { execFile } = require('child_process')
const { promisify } = require('util')

const DEFAULT_TIMEOUT_MS = 1500
const MAX_OUTPUT_BYTES = 16 * 1024

function parseNumberField(fields, name) {
  const raw = fields.get(name)
  if (!raw) return null
  const value = Number.parseFloat(raw)
  return Number.isFinite(value) ? value : null
}

function parseChronycTracking(output, sampledAtEpochMs = Date.now()) {
  const fields = new Map()
  for (const line of String(output).split(/\r?\n/)) {
    const separator = line.indexOf(':')
    if (separator < 0) continue
    fields.set(line.slice(0, separator).trim(), line.slice(separator + 1).trim())
  }

  const leapStatus = fields.get('Leap status') || ''
  const stratum = parseNumberField(fields, 'Stratum')
  const referenceRaw = fields.get('Reference ID') || ''
  const referenceMatch = /^([^\s]+)(?:\s+\((.+)\))?$/.exec(referenceRaw)
  const systemTimeRaw = fields.get('System time') || ''
  const systemTimeMatch =
    /^([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s+seconds\s+(fast|slow)\s+of\s+NTP\s+time$/i.exec(
      systemTimeRaw
    )
  if (!leapStatus || stratum == null || !referenceMatch || !systemTimeMatch) {
    throw new Error('chronyc tracking output is missing required fields')
  }

  const magnitude = Math.abs(Number.parseFloat(systemTimeMatch[1]))
  const systemTimeOffsetS = systemTimeMatch[2].toLowerCase() === 'fast' ? magnitude : -magnitude
  const synchronized = leapStatus.toLowerCase() !== 'not synchronised' && stratum > 0

  return {
    schema_version: 1,
    role: 'base',
    available: true,
    synchronized,
    reference_id: referenceMatch[1],
    reference_name: referenceMatch[2] || null,
    stratum,
    system_time_offset_s: systemTimeOffsetS,
    last_offset_s: parseNumberField(fields, 'Last offset'),
    rms_offset_s: parseNumberField(fields, 'RMS offset'),
    root_delay_s: parseNumberField(fields, 'Root delay'),
    root_dispersion_s: parseNumberField(fields, 'Root dispersion'),
    update_interval_s: parseNumberField(fields, 'Update interval'),
    leap_status: leapStatus,
    sampled_at_epoch_ms: sampledAtEpochMs,
    error: null,
  }
}

function unavailableStatus(error, sampledAtEpochMs = Date.now()) {
  return {
    schema_version: 1,
    role: 'base',
    available: false,
    synchronized: false,
    reference_id: null,
    reference_name: null,
    stratum: null,
    system_time_offset_s: null,
    last_offset_s: null,
    rms_offset_s: null,
    root_delay_s: null,
    root_dispersion_s: null,
    update_interval_s: null,
    leap_status: null,
    sampled_at_epoch_ms: sampledAtEpochMs,
    error,
  }
}

function createChronyStatusProvider(options = {}) {
  const execFileAsync = options.execFileAsync || promisify(execFile)
  const timeoutMs = options.timeoutMs || DEFAULT_TIMEOUT_MS

  return async function getChronyStatus() {
    const sampledAtEpochMs = Date.now()
    try {
      const { stdout } = await execFileAsync('chronyc', ['-n', 'tracking'], {
        timeout: timeoutMs,
        maxBuffer: MAX_OUTPUT_BYTES,
        windowsHide: true,
      })
      return parseChronycTracking(stdout, sampledAtEpochMs)
    } catch (error) {
      const code = error && typeof error === 'object' ? error.code : null
      const message = code === 'ENOENT' ? 'chronyc is not installed' : 'chronyc tracking failed'
      return unavailableStatus(message, sampledAtEpochMs)
    }
  }
}

module.exports = {
  createChronyStatusProvider,
  parseChronycTracking,
  unavailableStatus,
}
