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
  const availabilityStaleMs = Math.max(250, Math.floor(options.availabilityStaleMs || 1000))

  let child = null
  let stopping = false
  let restartTimer = null
  let availabilityTimer = null
  let parser = new AnnexBAccessUnitParser()
  let available = false

  function setAvailability(nextAvailable) {
    if (available === nextAvailable) return
    available = nextAvailable
    onAvailabilityChange(stream.stream_id, nextAvailable)
  }

  function clearAvailabilityTimer() {
    if (!availabilityTimer) return
    clearTimeout(availabilityTimer)
    availabilityTimer = null
  }

  function touchAvailability() {
    setAvailability(true)
    clearAvailabilityTimer()
    availabilityTimer = setTimeout(() => {
      availabilityTimer = null
      setAvailability(false)
    }, availabilityStaleMs)
  }

  function scheduleRestart() {
    if (stopping || restartTimer) return
    restartTimer = setTimeout(() => {
      restartTimer = null
      start()
    }, restartMs)
  }

  function handleStdout(chunk) {
    for (const accessUnit of parser.push(chunk)) {
      if (!accessUnit.payload || accessUnit.payload.length === 0) continue
      touchAvailability()
      onAccessUnit(stream.stream_id, {
        ...accessUnit,
        timestamp_us: Date.now() * 1000,
      })
    }
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
    let ended = false

    function finalizeChild(logMessage) {
      if (ended) return
      ended = true
      clearAvailabilityTimer()
      parser = new AnnexBAccessUnitParser()
      child = null
      setAvailability(false)
      if (logMessage) {
        log(logMessage)
      }
      scheduleRestart()
    }

    child.stdout.on('data', handleStdout)
    child.stderr.on('data', handleStderr)
    child.on('error', (error) => {
      finalizeChild(
        `[video:${stream.stream_id}] receiver process error: ${error.message || String(error)}`
      )
    })
    child.on('exit', (code, signal) => {
      finalizeChild(
        `[video:${stream.stream_id}] receiver exited code=${code ?? 'null'} signal=${
          signal || 'null'
        }`
      )
    })
  }

  function stop() {
    stopping = true
    clearAvailabilityTimer()
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
