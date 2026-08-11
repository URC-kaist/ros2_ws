'use strict'

const assert = require('node:assert/strict')
const fs = require('node:fs')
const os = require('node:os')
const path = require('node:path')
const { Readable } = require('node:stream')
const test = require('node:test')

const { ArtifactStore } = require('../src/latency/artifact_store')
const { buildTcpdumpArgs } = require('../src/latency/capture_process')
const { UplinkTrialManager } = require('../src/latency/uplink_trial_manager')

function createManager(root, events) {
  return new UplinkTrialManager({
    enabled: true,
    interfaceName: 'eth0',
    publicBaseUrl: 'http://192.168.1.101:8081',
    artifactRoot: root,
    analyzerPath: '/repo/scripts/latency/analyze_rtp_latency.py',
    videoConfigPath: '/repo/video_streams.json',
    streams: [
      { stream_id: 'front', udp_port: 5000 },
      { stream_id: 'rear', udp_port: 5002 },
    ],
    store: new ArtifactStore(root),
    captureFactory: (options) => ({
      start() {
        events.push(`capture-start:${options.ports.join(',')}`)
        return this
      },
      async stop() {
        events.push('capture-stop')
        fs.writeFileSync(options.outputPath, Buffer.from('base pcap'))
        fs.writeFileSync(`${options.outputPath}.metadata.json`, '{}')
        return { exitCode: 0, stderr: '' }
      },
    }),
    runAnalyzer: async (_binary, args) => {
      events.push('analyze')
      const outputPath = args[args.indexOf('--output') + 1]
      fs.writeFileSync(outputPath, JSON.stringify({ schema_version: 1, streams: [] }))
    },
  })
}

async function upload(manager, trialId, token, kind, body, contentType) {
  const request = Readable.from([body])
  return manager.receiveArtifact(trialId, kind, request, {
    authorization: `Bearer ${token}`,
    'content-length': String(body.length),
    'content-type': contentType,
  })
}

test('uplink trial starts base capture before accepting rover artifacts and analyzes both', async (t) => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mr2-uplink-manager-'))
  t.after(() => fs.rmSync(root, { recursive: true, force: true }))
  const events = []
  const manager = createManager(root, events)
  const created = manager.createTrial({
    feed_count: 2,
    stream_ids: ['front', 'rear'],
    duration_s: 5,
  })
  assert.throws(
    () => manager.createTrial({ feed_count: 1, stream_ids: ['front'], duration_s: 5 }),
    /another uplink trial/
  )
  const started = manager.startTrial(created.trial_id, {
    browser_clock: {
      offset_us: 10,
      rtt_us: 100,
      sampled_at_epoch_us: Date.now() * 1000,
    },
    chrony_snapshot: { ready: true },
  })
  assert.equal(events[0], 'capture-start:5000,5002')
  assert.ok(started.upload_token)

  await assert.rejects(
    upload(
      manager,
      created.trial_id,
      'wrong-token-that-is-long-enough',
      'rover-metadata',
      Buffer.from('{}'),
      'application/json'
    ),
    /invalid upload token/
  )
  await upload(
    manager,
    created.trial_id,
    started.upload_token,
    'rover-metadata',
    Buffer.from(JSON.stringify({
      role: 'rover',
      trial_id: created.trial_id,
      stream_ids: ['front', 'rear'],
    })),
    'application/json'
  )
  await upload(
    manager,
    created.trial_id,
    started.upload_token,
    'rover-capture',
    Buffer.from('rover pcap'),
    'application/vnd.tcpdump.pcap'
  )
  const browserEpochUs = Date.now() * 1000
  manager.saveBrowserSamples(created.trial_id, [
    {
      stream_id: 'front',
      ssrc: 1,
      rtp_timestamp: 90000,
      marker_sequence: 10,
      browser_receive_epoch_us: browserEpochUs,
      browser_render_epoch_us: browserEpochUs + 1000,
    },
  ])

  for (let attempt = 0; attempt < 20; attempt += 1) {
    if (manager.getTrial(created.trial_id).phase === 'completed') break
    await new Promise((resolve) => setImmediate(resolve))
  }
  const completed = manager.getTrial(created.trial_id)
  assert.equal(completed.phase, 'completed')
  assert.deepEqual(events, ['capture-start:5000,5002', 'capture-stop', 'analyze'])
  assert.equal(completed.browser_samples_received, true)
})

test('uplink trial validates feed count and tcpdump interface without shell strings', (t) => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mr2-uplink-validation-'))
  t.after(() => fs.rmSync(root, { recursive: true, force: true }))
  const manager = createManager(root, [])
  assert.throws(
    () => manager.createTrial({ feed_count: 2, stream_ids: ['front'], duration_s: 5 }),
    /feed_count/
  )
  assert.throws(
    () => buildTcpdumpArgs('eth0;shutdown', '/tmp/test.pcap', [5000]),
    /invalid capture interface/
  )
  assert.equal(
    buildTcpdumpArgs('eth0', '/tmp/test.pcap', [5002, 5000]).at(-1),
    'udp and (dst port 5000 or dst port 5002)'
  )
})

test('cancelling analysis terminates the analyzer and keeps a terminal cancelled phase', async (t) => {
  const root = fs.mkdtempSync(path.join(os.tmpdir(), 'mr2-uplink-cancel-'))
  t.after(() => fs.rmSync(root, { recursive: true, force: true }))
  const manager = createManager(root, [])
  const created = manager.createTrial({
    feed_count: 1,
    stream_ids: ['front'],
    duration_s: 5,
  })
  const trial = manager.trials.get(created.trial_id)
  let killed = false
  trial.phase = 'analyzing'
  trial.analyzerChild = {
    killed: false,
    kill(signal) {
      killed = signal === 'SIGTERM'
      this.killed = true
    },
  }

  await manager.cancelTrial(created.trial_id)
  manager.failTrial(trial, 'late_failure', 'must be ignored')

  assert.equal(killed, true)
  assert.equal(manager.getTrial(created.trial_id).phase, 'cancelled')
})
