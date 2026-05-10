'use strict'

const { spawn } = require('child_process')

function createMavproxyArgs(config) {
  return [
    `--master=${config.mavproxyMasterDevice},${config.mavproxyMasterBaud}`,
    `--out=${config.mavproxyOut}`,
    `--default-modules=${config.mavproxyDefaultModules || ''}`,
    '--non-interactive',
  ]
}

function startMavproxy(options = {}) {
  const config = options.config || {}
  const log = options.log || (() => {})
  const spawnFn = options.spawn || spawn

  if (!config.mavproxyEnable) {
    return { stop() {} }
  }

  const binary = config.mavproxyBinary || 'mavproxy.py'
  const args = createMavproxyArgs(config)
  const child = spawnFn(binary, args, {
    stdio: ['ignore', 'pipe', 'pipe'],
  })
  let stopped = false

  log(`MAVProxy starting: ${binary} ${args.join(' ')}`)

  if (child.stdout) {
    child.stdout.on('data', (data) => {
      const text = String(data).trim()
      if (text) log(`MAVProxy: ${text}`)
    })
  }

  if (child.stderr) {
    child.stderr.on('data', (data) => {
      const text = String(data).trim()
      if (text) log(`MAVProxy stderr: ${text}`)
    })
  }

  child.on('error', (err) => {
    log(`MAVProxy failed to start: ${err.message || err}`)
  })

  child.on('exit', (code, signal) => {
    if (stopped) return
    log(`MAVProxy exited code=${code === null ? 'null' : code} signal=${signal || 'none'}`)
  })

  return {
    stop() {
      stopped = true
      if (!child.killed) {
        child.kill('SIGTERM')
      }
    },
  }
}

module.exports = {
  createMavproxyArgs,
  startMavproxy,
}
