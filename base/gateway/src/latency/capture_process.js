'use strict'

const fs = require('fs')
const path = require('path')
const { spawn } = require('child_process')

const INTERFACE_PATTERN = /^[A-Za-z0-9_.:@-]{1,64}$/

function buildTcpdumpArgs(interfaceName, outputPath, ports) {
  if (!INTERFACE_PATTERN.test(interfaceName)) {
    throw new Error('invalid capture interface')
  }
  const selectedPorts = [...new Set(ports)].sort((left, right) => left - right)
  if (selectedPorts.length === 0) throw new Error('at least one RTP port is required')
  const filter = `udp and (${selectedPorts.map((port) => `dst port ${port}`).join(' or ')})`
  return [
    '-i',
    interfaceName,
    '-n',
    '-U',
    '-s',
    '192',
    '-B',
    '4096',
    '--time-stamp-precision=nano',
    '-w',
    outputPath,
    filter,
  ]
}

class RtpCaptureProcess {
  constructor(options) {
    this.role = options.role
    this.interfaceName = options.interfaceName
    this.outputPath = options.outputPath
    this.ports = options.ports
    this.streamIds = options.streamIds
    this.binary = options.binary || 'tcpdump'
    this.spawn = options.spawn || spawn
    this.child = null
    this.stderr = ''
    this.startedEpochUs = null
    this.finishedEpochUs = null
    this.exitCode = null
    this.waitPromise = null
    this.stopTimer = null
  }

  start(durationMs) {
    if (this.child) throw new Error('capture process is already started')
    const args = buildTcpdumpArgs(this.interfaceName, this.outputPath, this.ports)
    this.startedEpochUs = Date.now() * 1000
    this.child = this.spawn(this.binary, args, { stdio: ['ignore', 'ignore', 'pipe'] })
    if (this.child.stderr) {
      this.child.stderr.on('data', (chunk) => {
        this.stderr += chunk.toString('utf8')
      })
    }
    this.waitPromise = new Promise((resolve) => {
      const finish = (exitCode, error) => {
        if (this.exitCode != null) return
        this.exitCode = Number.isInteger(exitCode) ? exitCode : 1
        this.finishedEpochUs = Date.now() * 1000
        if (error) this.stderr += `${error.message || error}\n`
        this.writeMetadata()
        resolve({ exitCode: this.exitCode, stderr: this.stderr })
      }
      this.child.once('error', (error) => finish(1, error))
      this.child.once('close', (code) => finish(code, null))
    })
    if (durationMs > 0) {
      this.stopTimer = setTimeout(() => this.stop(), durationMs)
      this.stopTimer.unref?.()
    }
    return this
  }

  async stop() {
    if (!this.child || this.exitCode != null) return this.wait()
    if (this.stopTimer) {
      clearTimeout(this.stopTimer)
      this.stopTimer = null
    }
    this.child.kill('SIGINT')
    return this.wait()
  }

  wait() {
    return this.waitPromise || Promise.resolve({ exitCode: this.exitCode, stderr: this.stderr })
  }

  writeMetadata() {
    const metadataPath = `${this.outputPath}.metadata.json`
    fs.writeFileSync(
      metadataPath,
      `${JSON.stringify(
        {
          schema_version: 1,
          kind: 'mr2_rtp_capture_metadata',
          role: this.role,
          interface: this.interfaceName,
          streams: this.streamIds,
          started_epoch_us: this.startedEpochUs,
          finished_epoch_us: this.finishedEpochUs,
          tcpdump_exit_code: this.exitCode,
          tcpdump_stderr: this.stderr.trim(),
          output_path: path.basename(this.outputPath),
        },
        null,
        2
      )}\n`
    )
  }
}

module.exports = {
  INTERFACE_PATTERN,
  RtpCaptureProcess,
  buildTcpdumpArgs,
}
