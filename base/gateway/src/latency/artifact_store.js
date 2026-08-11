'use strict'

const fs = require('fs')
const path = require('path')

class ArtifactStore {
  constructor(rootPath) {
    this.rootPath = path.resolve(rootPath)
    fs.mkdirSync(this.rootPath, { recursive: true, mode: 0o700 })
    this.removeStaleParts()
  }

  createTrialDirectory() {
    return fs.mkdtempSync(path.join(this.rootPath, 'uplink-'))
  }

  receive(request, targetPath, maximumBytes) {
    return new Promise((resolve, reject) => {
      const partialPath = `${targetPath}.part`
      let received = 0
      let settled = false
      const output = fs.createWriteStream(partialPath, { flags: 'wx', mode: 0o600 })
      const fail = (error) => {
        if (settled) return
        settled = true
        output.destroy()
        fs.rm(partialPath, { force: true }, () => reject(error))
      }
      request.on('data', (chunk) => {
        received += chunk.length
        if (received > maximumBytes) {
          fail(new Error('artifact exceeds maximum size'))
          request.destroy()
          return
        }
        if (!output.write(chunk)) request.pause()
      })
      output.on('drain', () => request.resume())
      request.once('aborted', () => fail(new Error('artifact upload was interrupted')))
      request.once('error', fail)
      output.once('error', fail)
      request.once('end', () => {
        if (settled) return
        output.end(() => {
          if (settled) return
          settled = true
          fs.rename(partialPath, targetPath, (error) => {
            if (error) reject(error)
            else resolve(received)
          })
        })
      })
    })
  }

  removeStaleParts() {
    for (const entry of fs.readdirSync(this.rootPath, { withFileTypes: true })) {
      if (!entry.isDirectory()) continue
      const directory = path.join(this.rootPath, entry.name)
      for (const name of fs.readdirSync(directory)) {
        if (name.endsWith('.part')) fs.rmSync(path.join(directory, name), { force: true })
      }
    }
  }
}

module.exports = { ArtifactStore }
