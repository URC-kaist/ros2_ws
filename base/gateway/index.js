'use strict'

const { loadGatewayEnv, parseGatewayConfig } = require('./src/config')
const { createGatewayApp } = require('./src/app/create_gateway_app')

loadGatewayEnv(__dirname)

const config = parseGatewayConfig(process.argv.slice(2), process.env)
const app = createGatewayApp({
  config,
  env: process.env,
})

let shuttingDown = false

async function shutdown(signal) {
  if (shuttingDown) return
  shuttingDown = true

  // eslint-disable-next-line no-console
  console.log(`[gateway] ${signal} received, shutting down`)

  try {
    await app.stop()
    process.exit(0)
  } catch (err) {
    // eslint-disable-next-line no-console
    console.error(`[gateway] Shutdown failed: ${err.message || err}`)
    process.exit(1)
  }
}

process.on('SIGINT', () => {
  shutdown('SIGINT')
})

process.on('SIGTERM', () => {
  shutdown('SIGTERM')
})

app.start().catch((err) => {
  // eslint-disable-next-line no-console
  console.error(`[gateway] Startup failed: ${err.message || err}`)
  process.exit(1)
})
