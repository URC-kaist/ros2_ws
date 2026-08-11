'use strict'

const { spawn } = require('child_process')

const { AnnexBAccessUnitParser } = require('./h264')
const { RtpStreamParser } = require('./rtp_stream_parser')

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
    'tee',
    'name=rtp',
    'rtp.',
    '!',
    'queue',
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
    'rtp.',
    '!',
    'queue',
    '!',
    'rtpstreampay',
    '!',
    'fdsink',
    'fd=3',
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
  let rtpParser = new RtpStreamParser()
  let pendingAccessUnits = []
  let pendingMarkers = []
  let pairingTimer = null
  let available = false

  const maximumPairingQueue = Math.max(4, Math.floor(options.maximumPairingQueue || 64))
  const pairingTimeoutMs = Math.max(25, Math.floor(options.pairingTimeoutMs || 100))

  function resetCorrelation() {
    if (pairingTimer) {
      clearTimeout(pairingTimer)
      pairingTimer = null
    }
    parser = new AnnexBAccessUnitParser()
    rtpParser = new RtpStreamParser()
    pendingAccessUnits = []
    pendingMarkers = []
  }

  function emitAccessUnit(accessUnit, marker = null) {
    touchAvailability()
    onAccessUnit(stream.stream_id, {
      ...accessUnit,
      ...(marker ? { correlation: marker } : {}),
      timestamp_us: Date.now() * 1000,
    })
  }

  function schedulePairingFlush() {
    if (pairingTimer || (pendingAccessUnits.length === 0 && pendingMarkers.length === 0)) return
    pairingTimer = setTimeout(() => {
      pairingTimer = null
      const cutoff = Date.now() - pairingTimeoutMs
      while (pendingAccessUnits[0]?.queuedAt <= cutoff) {
        emitAccessUnit(pendingAccessUnits.shift().value)
        log(`[video:${stream.stream_id}] emitted an access unit without RTP correlation`)
      }
      while (pendingMarkers[0]?.queuedAt <= cutoff) {
        pendingMarkers.shift()
        log(`[video:${stream.stream_id}] discarded an unpaired RTP marker`)
      }
      pairFrames()
    }, pairingTimeoutMs)
    pairingTimer.unref?.()
  }

  function pairFrames() {
    while (pendingAccessUnits.length > 0 && pendingMarkers.length > 0) {
      const accessUnit = pendingAccessUnits.shift().value
      const marker = pendingMarkers.shift().value
      emitAccessUnit(accessUnit, marker)
    }
    if (pendingAccessUnits.length > maximumPairingQueue) {
      pendingAccessUnits.splice(0, pendingAccessUnits.length - maximumPairingQueue)
      log(`[video:${stream.stream_id}] dropped unpaired H.264 access units`)
    }
    if (pendingMarkers.length > maximumPairingQueue) {
      pendingMarkers.splice(0, pendingMarkers.length - maximumPairingQueue)
      log(`[video:${stream.stream_id}] dropped unpaired RTP markers`)
    }
    schedulePairingFlush()
  }

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
      pendingAccessUnits.push({ queuedAt: Date.now(), value: accessUnit })
    }
    pairFrames()
  }

  function handleRtpStream(chunk) {
    try {
      for (const packet of rtpParser.push(chunk)) {
        if (!packet.marker) continue
        pendingMarkers.push({
          queuedAt: Date.now(),
          value: {
            marker_sequence: packet.sequence,
            rtp_timestamp: packet.rtp_timestamp,
            ssrc: packet.ssrc,
          },
        })
      }
      pairFrames()
    } catch (error) {
      log(`[video:${stream.stream_id}] RTP metadata parse error: ${error.message || error}`)
      rtpParser.reset()
      pendingMarkers = []
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

    resetCorrelation()
    child = spawnImpl(gstBinary, args, {
      stdio: ['ignore', 'pipe', 'pipe', 'pipe'],
    })
    let ended = false

    function finalizeChild(logMessage) {
      if (ended) return
      ended = true
      clearAvailabilityTimer()
      resetCorrelation()
      child = null
      setAvailability(false)
      if (logMessage) {
        log(logMessage)
      }
      scheduleRestart()
    }

    child.stdout.on('data', handleStdout)
    child.stderr.on('data', handleStderr)
    if (child.stdio?.[3]?.on) {
      child.stdio[3].on('data', handleRtpStream)
    } else {
      log(`[video:${stream.stream_id}] RTP metadata output fd=3 is unavailable`)
    }
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
