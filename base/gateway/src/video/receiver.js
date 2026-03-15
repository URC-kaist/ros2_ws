'use strict'

const { spawn } = require('child_process')

const { AnnexBAccessUnitParser } = require('./h264')

function buildGstReceiveArgs(stream, options = {}) {
  const latencyMs = Math.max(0, Math.floor(options.jitterLatencyMs || 0))

  return [
    '-q',
    'udpsrc',
    `port=${stream.udp_port}`,
    'caps=application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000',
    '!',
    'rtpjitterbuffer',
    `latency=${latencyMs}`,
    'drop-on-latency=true',
    '!',
    'rtph264depay',
    '!',
    'h264parse',
    'disable-passthrough=true',
    '!',
    'video/x-h264,stream-format=byte-stream,alignment=au',
    '!',
    'fdsink',
    'fd=1',
    'sync=false',
  ]
}

function createVideoStreamReceiver(options = {}) {
  const stream = options.stream
  const spawnImpl = options.spawnImpl || spawn
  const log = typeof options.log === 'function' ? options.log : () => {}
  const onAccessUnit =
    typeof options.onAccessUnit === 'function' ? options.onAccessUnit : () => {}
  const onAvailabilityChange =
    typeof options.onAvailabilityChange === 'function'
      ? options.onAvailabilityChange
      : () => {}
  const gstBinary = options.gstBinary || 'gst-launch-1.0'
  const restartMs = Math.max(250, Math.floor(options.restartMs || 1000))
  const idleFlushMs = Math.max(1, Math.floor(options.idleFlushMs || 10))

  let child = null
  let stopping = false
  let restartTimer = null
  let flushTimer = null
  let parser = new AnnexBAccessUnitParser()
  let available = false

  function setAvailability(nextAvailable) {
    if (available === nextAvailable) return
    available = nextAvailable
    onAvailabilityChange(stream.stream_id, nextAvailable)
  }

  function clearFlushTimer() {
    if (!flushTimer) return
    clearTimeout(flushTimer)
    flushTimer = null
  }

  function flushParser() {
    clearFlushTimer()
    for (const accessUnit of parser.flush()) {
      if (!accessUnit.payload || accessUnit.payload.length === 0) continue
      setAvailability(true)
      onAccessUnit(stream.stream_id, {
        ...accessUnit,
        timestamp_us: Date.now() * 1000,
      })
    }
    parser = new AnnexBAccessUnitParser()
  }

  function scheduleFlush() {
    clearFlushTimer()
    flushTimer = setTimeout(() => {
      flushParser()
    }, idleFlushMs)
  }

  function scheduleRestart() {
    if (stopping || restartTimer) return
    restartTimer = setTimeout(() => {
      restartTimer = null
      start()
    }, restartMs)
  }

  function handleStdout(chunk) {
    clearFlushTimer()
    for (const accessUnit of parser.push(chunk)) {
      if (!accessUnit.payload || accessUnit.payload.length === 0) continue
      setAvailability(true)
      onAccessUnit(stream.stream_id, {
        ...accessUnit,
        timestamp_us: Date.now() * 1000,
      })
    }
    scheduleFlush()
  }

  function handleStderr(chunk) {
    const text = String(chunk).trim()
    if (text) {
      log(`[video:${stream.stream_id}] ${text}`)
    }
  }

  function start() {
    if (child || stopping) return

    const args = buildGstReceiveArgs(stream, {
      jitterLatencyMs: options.jitterLatencyMs,
    })
    log(`[video:${stream.stream_id}] spawning ${gstBinary} ${args.join(' ')}`)

    parser = new AnnexBAccessUnitParser()
    child = spawnImpl(gstBinary, args, {
      stdio: ['ignore', 'pipe', 'pipe'],
    })

    child.stdout.on('data', handleStdout)
    child.stderr.on('data', handleStderr)
    child.on('exit', (code, signal) => {
      clearFlushTimer()
      if (!stopping) {
        flushParser()
      }
      child = null
      setAvailability(false)
      log(
        `[video:${stream.stream_id}] receiver exited code=${code ?? 'null'} signal=${
          signal || 'null'
        }`
      )
      scheduleRestart()
    })
  }

  function stop() {
    stopping = true
    clearFlushTimer()
    if (restartTimer) {
      clearTimeout(restartTimer)
      restartTimer = null
    }
    if (!child) {
      setAvailability(false)
      return
    }
    child.kill('SIGTERM')
    child = null
    setAvailability(false)
  }

  return {
    start,
    stop,
  }
}

module.exports = {
  buildGstReceiveArgs,
  createVideoStreamReceiver,
}
