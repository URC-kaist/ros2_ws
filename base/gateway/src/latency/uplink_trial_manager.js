'use strict'

const crypto = require('crypto')
const fs = require('fs')
const path = require('path')
const { spawn } = require('child_process')

const { ArtifactStore } = require('./artifact_store')
const { RtpCaptureProcess } = require('./capture_process')

const ACTIVE_PHASES = new Set([
  'created',
  'base_capturing',
  'waiting_for_rover_artifacts',
  'stopping_base_capture',
  'analyzing',
])

function validateDuration(value, maximum = 60) {
  const duration = Number(value)
  if (!Number.isFinite(duration) || duration < 2 || duration > maximum) {
    throw new Error(`duration_s must be between 2 and ${maximum}`)
  }
  return duration
}

function runCommand(binary, args, spawnProcess = spawn, onChild = null) {
  return new Promise((resolve, reject) => {
    const child = spawnProcess(binary, args, { stdio: ['ignore', 'pipe', 'pipe'] })
    if (onChild) onChild(child)
    let stdout = ''
    let stderr = ''
    child.stdout?.on('data', (chunk) => { stdout += chunk.toString('utf8') })
    child.stderr?.on('data', (chunk) => { stderr += chunk.toString('utf8') })
    child.once('error', reject)
    child.once('close', (code) => {
      if (code === 0) resolve({ stdout, stderr })
      else reject(new Error(stderr.trim() || `${binary} exited with code ${code}`))
    })
  })
}

class UplinkTrialManager {
  constructor(options) {
    this.enabled = options.enabled === true
    this.interfaceName = options.interfaceName || ''
    this.publicBaseUrl = String(options.publicBaseUrl || '').replace(/\/$/, '')
    this.videoConfigPath = path.resolve(options.videoConfigPath)
    this.streams = options.streams || []
    this.captureDurationS = options.captureDurationS || 15
    this.captureEdgeMarginS = options.captureEdgeMarginS || 5
    this.maxUploadBytes = options.maxUploadBytes || 256 * 1024 * 1024
    this.maxJsonBytes = options.maxJsonBytes || 2 * 1024 * 1024
    this.pythonBinary = options.pythonBinary || 'python3'
    this.analyzerPath = path.resolve(options.analyzerPath)
    this.tcpdumpBinary = options.tcpdumpBinary || 'tcpdump'
    this.spawn = options.spawn || spawn
    this.captureFactory = options.captureFactory || ((captureOptions) => new RtpCaptureProcess(captureOptions))
    this.runAnalyzer = options.runAnalyzer || ((binary, args, trial) =>
      runCommand(binary, args, this.spawn, (child) => { trial.analyzerChild = child }))
    this.store = options.store || new ArtifactStore(options.artifactRoot)
    this.trials = new Map()
    this.activeTrialId = null
  }

  configurationStatus() {
    const missing = []
    if (!this.enabled) missing.push('MR2_LATENCY_DIAGNOSTICS_ENABLE')
    if (!this.interfaceName) missing.push('MR2_BASE_ROCKET_INTERFACE')
    if (!this.publicBaseUrl) missing.push('MR2_LATENCY_PUBLIC_BASE_URL')
    return { configured: missing.length === 0, missing }
  }

  createTrial(body) {
    const configuration = this.configurationStatus()
    if (!configuration.configured) {
      const error = new Error(`uplink diagnostics not configured: ${configuration.missing.join(', ')}`)
      error.code = 'not_configured'
      throw error
    }
    if (this.activeTrialId) throw new Error('another uplink trial is active')
    const durationS = validateDuration(body.duration_s ?? this.captureDurationS)
    const requestedIds = Array.isArray(body.stream_ids) ? [...new Set(body.stream_ids)] : []
    const feedCount = Number(body.feed_count)
    if (!Number.isInteger(feedCount) || feedCount < 1 || feedCount !== requestedIds.length) {
      throw new Error('feed_count must match the number of unique stream_ids')
    }
    const byId = new Map(this.streams.map((stream) => [stream.stream_id, stream]))
    const selected = requestedIds.map((streamId) => {
      const stream = byId.get(streamId)
      if (!stream) throw new Error(`unknown stream_id ${streamId}`)
      return stream
    })
    const id = crypto.randomUUID()
    const directory = this.store.createTrialDirectory()
    const trial = {
      id,
      phase: 'created',
      progress: 0.05,
      feedCount,
      streamIds: requestedIds,
      streams: selected,
      durationS,
      directory,
      token: null,
      baseCapture: null,
      roverMetadataReceived: false,
      roverCaptureReceived: false,
      browserSamplesReceived: false,
      report: null,
      error: null,
      clock: null,
      createdAtEpochMs: Date.now(),
      startedAtEpochMs: null,
      completedAtEpochMs: null,
      analysisStarted: false,
      edgeTimer: null,
      analyzerChild: null,
    }
    this.trials.set(id, trial)
    this.activeTrialId = id
    return this.serialize(trial)
  }

  startTrial(id, body) {
    const trial = this.requireTrial(id)
    if (trial.phase !== 'created') throw new Error(`trial cannot start from phase ${trial.phase}`)
    const browserClock = body.browser_clock
    const chrony = body.chrony_snapshot
    if (!browserClock || !Number.isFinite(Number(browserClock.offset_us)) ||
        !Number.isFinite(Number(browserClock.rtt_us)) ||
        !Number.isFinite(Number(browserClock.sampled_at_epoch_us))) {
      throw new Error('browser_clock snapshot is invalid')
    }
    if (!chrony || chrony.ready !== true) throw new Error('chrony snapshot is not ready')
    const sampleAgeUs = Math.abs(Date.now() * 1000 - Number(browserClock.sampled_at_epoch_us))
    if (sampleAgeUs > 5_000_000) throw new Error('browser_clock snapshot is stale')

    const basePath = path.join(trial.directory, 'base.pcap')
    trial.baseCapture = this.captureFactory({
      role: 'base',
      interfaceName: this.interfaceName,
      outputPath: basePath,
      ports: trial.streams.map((stream) => stream.udp_port),
      streamIds: trial.streamIds,
      binary: this.tcpdumpBinary,
      spawn: this.spawn,
    })
    trial.baseCapture.start((trial.durationS + this.captureEdgeMarginS) * 1000)
    trial.phase = 'base_capturing'
    trial.progress = 0.15
    trial.startedAtEpochMs = Date.now()
    trial.clock = { browser: browserClock, chrony }
    trial.token = crypto.randomBytes(32).toString('base64url')
    trial.phase = 'waiting_for_rover_artifacts'
    trial.progress = 0.25
    trial.edgeTimer = setTimeout(() => {
      if (trial.phase !== 'waiting_for_rover_artifacts') return
      void trial.baseCapture.stop().finally(() => {
        const browserMissing = !trial.browserSamplesReceived
        this.failTrial(
          trial,
          browserMissing ? 'browser_samples_missing' : 'rover_artifact_timeout',
          browserMissing
            ? 'Browser frame samples did not arrive before the trial timeout'
            : 'Rover capture artifacts did not arrive before the trial timeout'
        )
      })
    }, (trial.durationS + this.captureEdgeMarginS + 30) * 1000)
    trial.edgeTimer.unref?.()
    return {
      ...this.serialize(trial),
      upload_base_url: `${this.publicBaseUrl}/latency/uplink/trials/${trial.id}`,
      upload_token: trial.token,
    }
  }

  getTrial(id) {
    return this.serialize(this.requireTrial(id))
  }

  async cancelTrial(id) {
    const trial = this.requireTrial(id)
    if (!ACTIVE_PHASES.has(trial.phase)) return this.serialize(trial)
    trial.phase = 'cancelled'
    trial.error = { code: 'trial_cancelled', message: 'Uplink trial was cancelled' }
    trial.completedAtEpochMs = Date.now()
    trial.token = null
    if (trial.edgeTimer) {
      clearTimeout(trial.edgeTimer)
      trial.edgeTimer = null
    }
    if (trial.analyzerChild && !trial.analyzerChild.killed) {
      trial.analyzerChild.kill('SIGTERM')
    }
    if (trial.baseCapture) await trial.baseCapture.stop()
    if (this.activeTrialId === id) this.activeTrialId = null
    return this.serialize(trial)
  }

  async stop() {
    const active = this.activeTrialId
    if (active) await this.cancelTrial(active)
  }

  async receiveArtifact(id, kind, request, headers) {
    const trial = this.requireTrial(id)
    if (trial.phase !== 'waiting_for_rover_artifacts') {
      throw new Error(`trial does not accept artifacts in phase ${trial.phase}`)
    }
    this.verifyToken(trial, headers.authorization)
    const length = Number(headers['content-length'])
    if (!Number.isInteger(length) || length < 1 || length > this.maxUploadBytes) {
      throw new Error('invalid artifact content-length')
    }
    const metadata = kind === 'rover-metadata'
    const expectedType = metadata ? 'application/json' : 'application/vnd.tcpdump.pcap'
    if (!String(headers['content-type'] || '').startsWith(expectedType)) {
      throw new Error(`artifact content-type must be ${expectedType}`)
    }
    if (metadata && trial.roverMetadataReceived) throw new Error('rover metadata already uploaded')
    if (!metadata && trial.roverCaptureReceived) throw new Error('rover capture already uploaded')
    const targetPath = metadata
      ? path.join(trial.directory, 'rover.pcap.metadata.json')
      : path.join(trial.directory, 'rover.pcap')
    const receivedBytes = await this.store.receive(
      request,
      targetPath,
      metadata ? this.maxJsonBytes : this.maxUploadBytes
    )
    if (receivedBytes !== length) {
      fs.rmSync(targetPath, { force: true })
      throw new Error('artifact content-length does not match uploaded bytes')
    }
    if (metadata) {
      try {
        const value = JSON.parse(fs.readFileSync(targetPath, 'utf8'))
        const streamIds = Array.isArray(value.stream_ids) ? value.stream_ids : []
        if (value.role !== 'rover' || value.trial_id !== trial.id ||
            streamIds.length !== trial.streamIds.length ||
            !trial.streamIds.every((streamId) => streamIds.includes(streamId))) {
          throw new Error('rover metadata does not match the trial')
        }
      } catch (error) {
        fs.rmSync(targetPath, { force: true })
        throw new Error(error.message || 'rover metadata is invalid')
      }
    }
    if (metadata) trial.roverMetadataReceived = true
    else trial.roverCaptureReceived = true
    trial.progress = trial.roverMetadataReceived && trial.roverCaptureReceived
      ? trial.browserSamplesReceived ? 0.75 : 0.65
      : 0.5
    if (trial.roverMetadataReceived && trial.roverCaptureReceived) {
      trial.token = null
      void this.analyzeWhenReady(trial)
    }
    return this.serialize(trial)
  }

  saveBrowserSamples(id, samples) {
    const trial = this.requireTrial(id)
    if (trial.phase !== 'waiting_for_rover_artifacts') {
      throw new Error(`trial does not accept browser samples in phase ${trial.phase}`)
    }
    if (!Array.isArray(samples) || samples.length === 0) {
      throw new Error('browser samples must be a non-empty array')
    }
    if (samples.length > 20000) throw new Error('too many browser samples')
    const browserClockOffsetUs = Number(trial.clock?.browser?.offset_us)
    if (!Number.isFinite(browserClockOffsetUs)) {
      throw new Error('trial browser clock offset is unavailable')
    }
    const allowedStreams = new Set(trial.streamIds)
    const earliestEpochUs = trial.startedAtEpochMs * 1000 - 5_000_000
    const latestEpochUs = Date.now() * 1000 + 5_000_000
    for (const [index, sample] of samples.entries()) {
      if (!sample || !allowedStreams.has(sample.stream_id)) {
        throw new Error(`browser sample ${index} has an invalid stream_id`)
      }
      const numericFields = [
        'ssrc',
        'rtp_timestamp',
        'marker_sequence',
        'browser_receive_epoch_us',
        'browser_render_epoch_us',
      ]
      if (numericFields.some((field) => !Number.isFinite(Number(sample[field])))) {
        throw new Error(`browser sample ${index} has invalid numeric fields`)
      }
      if (!Number.isInteger(sample.ssrc) || sample.ssrc < 0 || sample.ssrc > 0xffffffff ||
          !Number.isInteger(sample.rtp_timestamp) || sample.rtp_timestamp < 0 ||
          sample.rtp_timestamp > 0xffffffff || !Number.isInteger(sample.marker_sequence) ||
          sample.marker_sequence < 0 || sample.marker_sequence > 0xffff) {
        throw new Error(`browser sample ${index} has an invalid RTP key`)
      }
      // Browser samples deliberately remain in the browser clock domain; the
      // analyzer applies this same offset when calculating latency. Normalize
      // only for validation against the Base trial window.
      const receiveInBaseEpochUs = sample.browser_receive_epoch_us + browserClockOffsetUs
      const renderInBaseEpochUs = sample.browser_render_epoch_us + browserClockOffsetUs
      if (receiveInBaseEpochUs < earliestEpochUs ||
          renderInBaseEpochUs > latestEpochUs ||
          sample.browser_render_epoch_us < sample.browser_receive_epoch_us) {
        throw new Error(`browser sample ${index} is outside the trial time range`)
      }
    }
    fs.writeFileSync(
      path.join(trial.directory, 'browser-samples.json'),
      `${JSON.stringify({ schema_version: 1, samples })}\n`,
      { mode: 0o600 }
    )
    trial.browserSamplesReceived = true
    trial.progress = trial.roverMetadataReceived && trial.roverCaptureReceived ? 0.75 : 0.55
    void this.analyzeWhenReady(trial)
    return this.serialize(trial)
  }

  async analyzeWhenReady(trial) {
    if (trial.analysisStarted || !trial.roverMetadataReceived ||
        !trial.roverCaptureReceived || !trial.browserSamplesReceived) return
    trial.analysisStarted = true
    if (trial.edgeTimer) {
      clearTimeout(trial.edgeTimer)
      trial.edgeTimer = null
    }
    try {
      trial.phase = 'stopping_base_capture'
      trial.progress = 0.75
      const captureResult = await trial.baseCapture.stop()
      if (captureResult.exitCode !== 0) {
        throw new Error(`base tcpdump exited with code ${captureResult.exitCode}`)
      }
      trial.phase = 'analyzing'
      trial.progress = 0.85
      const reportPath = path.join(trial.directory, 'report.json')
      const args = [
        this.analyzerPath,
        '--rover', path.join(trial.directory, 'rover.pcap'),
        '--base', path.join(trial.directory, 'base.pcap'),
        '--config', this.videoConfigPath,
        '--clock-offset-us', '0',
        '--browser-samples', path.join(trial.directory, 'browser-samples.json'),
        '--browser-clock-offset-us', String(trial.clock.browser.offset_us),
        '--trial-id', trial.id,
        '--feed-count', String(trial.feedCount),
        '--output', reportPath,
      ]
      for (const streamId of trial.streamIds) args.push('--stream-id', streamId)
      await this.runAnalyzer(this.pythonBinary, args, trial)
      trial.analyzerChild = null
      if (trial.phase === 'cancelled') return
      trial.report = JSON.parse(fs.readFileSync(reportPath, 'utf8'))
      trial.phase = 'completed'
      trial.progress = 1
      trial.completedAtEpochMs = Date.now()
      if (this.activeTrialId === trial.id) this.activeTrialId = null
    } catch (error) {
      trial.analyzerChild = null
      if (trial.phase === 'cancelled') return
      const message = error.message || String(error)
      const code = message.includes('no RTP marker frames could be correlated')
        ? 'frame_correlation_insufficient'
        : 'analysis_failed'
      this.failTrial(trial, code, message)
    }
  }

  failTrial(trial, code, message) {
    if (trial.phase === 'completed' || trial.phase === 'failed' || trial.phase === 'cancelled') {
      return
    }
    if (trial.edgeTimer) {
      clearTimeout(trial.edgeTimer)
      trial.edgeTimer = null
    }
    trial.phase = 'failed'
    trial.error = { code, message }
    trial.token = null
    trial.completedAtEpochMs = Date.now()
    if (this.activeTrialId === trial.id) this.activeTrialId = null
  }

  requireTrial(id) {
    const trial = this.trials.get(id)
    if (!trial) throw new Error('uplink trial not found')
    return trial
  }

  verifyToken(trial, authorization) {
    const prefix = 'Bearer '
    const supplied = typeof authorization === 'string' && authorization.startsWith(prefix)
      ? authorization.slice(prefix.length)
      : ''
    if (!trial.token || supplied.length !== trial.token.length ||
        !crypto.timingSafeEqual(Buffer.from(supplied), Buffer.from(trial.token))) {
      throw new Error('invalid upload token')
    }
  }

  serialize(trial) {
    return {
      schema_version: 1,
      trial_id: trial.id,
      phase: trial.phase,
      progress: trial.progress,
      feed_count: trial.feedCount,
      stream_ids: trial.streamIds,
      duration_s: trial.durationS,
      rover_metadata_received: trial.roverMetadataReceived,
      rover_capture_received: trial.roverCaptureReceived,
      browser_samples_received: trial.browserSamplesReceived,
      error: trial.error,
      report: trial.report,
      created_at_epoch_ms: trial.createdAtEpochMs,
      started_at_epoch_ms: trial.startedAtEpochMs,
      completed_at_epoch_ms: trial.completedAtEpochMs,
    }
  }
}

module.exports = {
  ACTIVE_PHASES,
  UplinkTrialManager,
  runCommand,
  validateDuration,
}
