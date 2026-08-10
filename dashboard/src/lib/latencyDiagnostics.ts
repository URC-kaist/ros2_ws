export type LatencyStage =
  | 'input_t0'
  | 'gateway_command_received_t1'
  | 'rover_command_received_t3'
  | 'rover_cmd_vel_published_t4'

export type LatencySource = 'dashboard' | 'gateway' | 'rover'

export type LatencyRecord = {
  schema_version: 1
  session_id: string
  trial_id: string | null
  stage: LatencyStage | 'session'
  source: LatencySource
  stream_id?: string
  sequence?: number
  epoch_us: number
  clock_offset_to_base_us: number | null
  delta_t0_ms: number | null
  metadata: Record<string, unknown>
}

export type CommandTraceTag = {
  trial_id: string
  client_tx_epoch_us: number
}

export type GatewayLatencyTrace = {
  type: 'latency_trace'
  trial_id: string
  client_tx_epoch_us: number
  gateway_rx_epoch_us: number
  xbee_seq: number
  wire_timestamp_ms: number
}

export type RoverLatencyTrace = {
  schema_version?: number
  event: 'command'
  xbee_seq: number
  wire_timestamp_ms: number
  rover_rx_epoch_us?: number
  cmd_publish_epoch_us?: number
  target?: { x: number; y: number; yaw: number }
  output?: { x: number; y: number; yaw: number }
}

export type TrialStages = Partial<Record<LatencyStage, number>>

export type LatencyTrialSnapshot = {
  id: string
  stages: TrialStages
  metrics: {
    computerToGatewayMs: number | null
    computerToRoverMs: number | null
    roverPublishMs: number | null
  }
}

export type VideoTimingDistribution = {
  count: number
  p50Ms: number | null
  p95Ms: number | null
}

export type LatencyDiagnosticsSnapshot = {
  sessionId: string
  armed: boolean
  neutralSeen: boolean
  pendingCommandTrace: boolean
  selectedStreamId: string | null
  currentTrial: LatencyTrialSnapshot | null
  recordCount: number
  droppedRecordCount: number
  videoTiming: {
    sampleCount: number
    baseToBrowser: VideoTimingDistribution
    decodeRender: VideoTimingDistribution
  }
  clock: {
    syncing: boolean
    offsetUs: number | null
    rttUs: number | null
    syncedAtEpochUs: number | null
    error: string | null
  }
}

type DriveVector = { x: number; y: number; yaw: number }

type MutableTrial = {
  id: string
  stages: TrialStages
}

type PendingRoverTrace = RoverLatencyTrace

type VideoReceiveMetadata = {
  streamId: string
  baseIngestEpochUs: number
  browserReceiveEpochUs: number
}

type VideoTimingSample = VideoReceiveMetadata & {
  browserRenderEpochUs: number
}

const MAX_RECORDS = 20_000
const MAX_PENDING_ROVER_TRACES = 512
const MAX_TRACE_MAPPINGS = 2_048
const MAX_VIDEO_TIMING_SAMPLES = 600
const DRIVE_TRIGGER_THRESHOLD = 0.02
const VIDEO_SAMPLES_PER_UI_UPDATE = 8

export function browserEpochUs() {
  if (typeof performance !== 'undefined') {
    return Math.round((performance.timeOrigin + performance.now()) * 1000)
  }
  return Date.now() * 1000
}

function createSessionId() {
  return `latency-${new Date().toISOString().replace(/[-:.]/g, '').replace('Z', 'Z')}`
}

function traceKey(sequence: number, wireTimestampMs: number) {
  return `${sequence & 0xff}:${wireTimestampMs >>> 0}`
}

function median(values: number[]) {
  if (values.length === 0) return null
  const sorted = [...values].sort((left, right) => left - right)
  const middle = Math.floor(sorted.length / 2)
  return sorted.length % 2 === 0
    ? (sorted[middle - 1] + sorted[middle]) / 2
    : sorted[middle]
}

function percentile(values: number[], quantile: number) {
  if (values.length === 0) return null
  const sorted = [...values].sort((left, right) => left - right)
  if (sorted.length === 1) return sorted[0]
  const position = (sorted.length - 1) * quantile
  const lower = Math.floor(position)
  const upper = Math.ceil(position)
  if (lower === upper) return sorted[lower]
  return sorted[lower] + (sorted[upper] - sorted[lower]) * (position - lower)
}

function resolveLatencyTimeUrl() {
  const explicit = import.meta.env.VITE_XBEE_WS_URL as string | undefined
  if (explicit) {
    const url = new URL(explicit, window.location.href)
    url.protocol = url.protocol === 'wss:' ? 'https:' : 'http:'
    url.pathname = '/latency/time'
    url.search = ''
    url.hash = ''
    return url.toString()
  }
  return `${window.location.origin}/latency/time`
}

export class LatencyDiagnostics {
  private sessionId = createSessionId()
  private trialCounter = 0
  private armed = false
  private neutralSeen = false
  private pendingCommandTrace: CommandTraceTag | null = null
  private currentTrial: MutableTrial | null = null
  private selectedStreamId: string | null = null
  private records: LatencyRecord[] = []
  private droppedRecordCount = 0
  private gatewayClockOffsetUs: number | null = null
  private gatewayClockRttUs: number | null = null
  private clockSyncedAtEpochUs: number | null = null
  private clockSyncing = false
  private clockError: string | null = null
  private trialByTraceKey = new Map<string, string>()
  private pendingRoverTraces = new Map<string, PendingRoverTrace[]>()
  private videoReceives = new Map<string, VideoReceiveMetadata>()
  private videoTimingSamples: VideoTimingSample[] = []
  private videoSamplesSinceEmit = 0
  private chronyStatusSnapshot: Record<string, unknown> | null = null
  private listeners = new Set<() => void>()
  private snapshot: LatencyDiagnosticsSnapshot

  constructor() {
    this.snapshot = this.buildSnapshot()
    this.appendRecord({
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: null,
      stage: 'session',
      source: 'dashboard',
      epoch_us: browserEpochUs(),
      clock_offset_to_base_us: this.gatewayClockOffsetUs,
      delta_t0_ms: null,
      metadata: { event: 'session_started' },
    })
  }

  subscribe = (listener: () => void) => {
    this.listeners.add(listener)
    return () => this.listeners.delete(listener)
  }

  getSnapshot = () => this.snapshot

  armTrial() {
    this.armed = true
    this.neutralSeen = false
    this.pendingCommandTrace = null
    this.currentTrial = null
    this.emit()
  }

  observeControlInput(vector: DriveVector) {
    if (!this.armed) return
    const magnitude = Math.max(Math.abs(vector.x), Math.abs(vector.y), Math.abs(vector.yaw))
    if (magnitude < DRIVE_TRIGGER_THRESHOLD) {
      if (!this.neutralSeen) {
        this.neutralSeen = true
        this.emit()
      }
      return
    }
    if (!this.neutralSeen) return

    const t0 = browserEpochUs()
    this.trialCounter += 1
    const id = `${this.sessionId}-${String(this.trialCounter).padStart(6, '0')}`
    this.currentTrial = {
      id,
      stages: { input_t0: t0 },
    }
    this.pendingCommandTrace = { trial_id: id, client_tx_epoch_us: t0 }
    this.armed = false
    this.appendStageRecord('input_t0', 'dashboard', t0, {
      input: vector,
    })
    this.emit()
  }

  claimPendingCommandTrace(): CommandTraceTag | undefined {
    const pending = this.pendingCommandTrace
    if (!pending) return undefined
    this.pendingCommandTrace = null
    this.emit()
    return pending
  }

  selectStream(streamId: string | null) {
    this.selectedStreamId = streamId
    this.videoReceives.clear()
    this.videoTimingSamples = []
    this.videoSamplesSinceEmit = 0
    this.emit()
  }

  setChronyStatusSnapshot(snapshot: Record<string, unknown> | null) {
    this.chronyStatusSnapshot = snapshot
  }

  ingestGatewayTrace(trace: GatewayLatencyTrace) {
    if (!trace.trial_id || !Number.isFinite(trace.gateway_rx_epoch_us)) return
    const key = traceKey(trace.xbee_seq, trace.wire_timestamp_ms)
    this.trialByTraceKey.set(key, trace.trial_id)
    while (this.trialByTraceKey.size > MAX_TRACE_MAPPINGS) {
      const oldestKey = this.trialByTraceKey.keys().next().value as string | undefined
      if (!oldestKey) break
      this.trialByTraceKey.delete(oldestKey)
    }
    this.setTrialStage(
      trace.trial_id,
      'gateway_command_received_t1',
      trace.gateway_rx_epoch_us,
      'gateway',
      {
        xbee_seq: trace.xbee_seq,
        wire_timestamp_ms: trace.wire_timestamp_ms,
        client_tx_epoch_us: trace.client_tx_epoch_us,
      }
    )

    const pending = this.pendingRoverTraces.get(key) || []
    this.pendingRoverTraces.delete(key)
    for (const roverTrace of pending) {
      this.applyRoverTrace(trace.trial_id, roverTrace)
    }
    this.emit()
  }

  ingestRoverTrace(trace: RoverLatencyTrace) {
    if (!Number.isFinite(trace.xbee_seq) || !Number.isFinite(trace.wire_timestamp_ms)) return
    const key = traceKey(trace.xbee_seq, trace.wire_timestamp_ms)
    const trialId = this.trialByTraceKey.get(key)
    if (!trialId) {
      const pending = this.pendingRoverTraces.get(key) || []
      pending.push(trace)
      this.pendingRoverTraces.set(key, pending)
      this.trimPendingRoverTraces()
      return
    }
    this.applyRoverTrace(trialId, trace)
    this.emit()
  }

  observeVideoReceive(
    streamId: string,
    baseIngestEpochUs: number,
    browserReceiveEpochUs: number
  ) {
    if (this.selectedStreamId == null) {
      this.selectedStreamId = streamId
      this.emit()
    }
    if (streamId !== this.selectedStreamId) return
    const key = this.videoFrameKey(streamId, baseIngestEpochUs)
    this.videoReceives.set(key, {
      streamId,
      baseIngestEpochUs,
      browserReceiveEpochUs,
    })
    if (this.videoReceives.size > MAX_VIDEO_TIMING_SAMPLES * 2) {
      const oldestKey = this.videoReceives.keys().next().value as string | undefined
      if (oldestKey) this.videoReceives.delete(oldestKey)
    }
  }

  observeVideoRender(streamId: string, baseIngestEpochUs: number, browserRenderEpochUs: number) {
    if (streamId !== this.selectedStreamId) return
    const key = this.videoFrameKey(streamId, baseIngestEpochUs)
    const receive = this.videoReceives.get(key)
    this.videoReceives.delete(key)
    if (!receive) return

    this.videoTimingSamples.push({
      ...receive,
      browserRenderEpochUs,
    })
    if (this.videoTimingSamples.length > MAX_VIDEO_TIMING_SAMPLES) {
      this.videoTimingSamples.splice(
        0,
        this.videoTimingSamples.length - MAX_VIDEO_TIMING_SAMPLES
      )
    }
    this.videoSamplesSinceEmit += 1
    if (
      this.videoTimingSamples.length === 1 ||
      this.videoSamplesSinceEmit >= VIDEO_SAMPLES_PER_UI_UPDATE
    ) {
      this.videoSamplesSinceEmit = 0
      this.emit()
    }
  }

  async synchronizeGatewayClock(sampleCount = 8) {
    if (this.clockSyncing) return
    this.clockSyncing = true
    this.clockError = null
    this.emit()

    try {
      const samples: Array<{ offsetUs: number; rttUs: number }> = []
      const url = resolveLatencyTimeUrl()
      for (let index = 0; index < sampleCount; index += 1) {
        const browserSendUs = browserEpochUs()
        const response = await fetch(url, { cache: 'no-store' })
        const browserReceiveUs = browserEpochUs()
        if (!response.ok) throw new Error(`Clock endpoint returned ${response.status}`)
        const body = (await response.json()) as {
          server_receive_epoch_us?: number
          server_send_epoch_us?: number
        }
        const serverReceiveUs = Number(body.server_receive_epoch_us)
        const serverSendUs = Number(body.server_send_epoch_us)
        if (!Number.isFinite(serverReceiveUs) || !Number.isFinite(serverSendUs)) {
          throw new Error('Clock endpoint returned invalid timestamps')
        }
        const rttUs =
          browserReceiveUs - browserSendUs - Math.max(0, serverSendUs - serverReceiveUs)
        const offsetUs =
          ((serverReceiveUs - browserSendUs) + (serverSendUs - browserReceiveUs)) / 2
        samples.push({ offsetUs, rttUs })
      }
      samples.sort((left, right) => left.rttUs - right.rttUs)
      const selected = samples.slice(0, Math.max(1, Math.ceil(samples.length / 2)))
      this.gatewayClockOffsetUs = median(selected.map((sample) => sample.offsetUs))
      this.gatewayClockRttUs = median(selected.map((sample) => sample.rttUs))
      this.clockSyncedAtEpochUs = browserEpochUs()
    } catch (error) {
      this.clockError = error instanceof Error ? error.message : 'Clock synchronization failed'
      this.gatewayClockOffsetUs = null
      this.gatewayClockRttUs = null
    } finally {
      this.clockSyncing = false
      this.emit()
    }
  }

  resetSession() {
    this.sessionId = createSessionId()
    this.trialCounter = 0
    this.armed = false
    this.neutralSeen = false
    this.pendingCommandTrace = null
    this.currentTrial = null
    this.records = []
    this.droppedRecordCount = 0
    this.trialByTraceKey.clear()
    this.pendingRoverTraces.clear()
    this.videoReceives.clear()
    this.videoTimingSamples = []
    this.videoSamplesSinceEmit = 0
    this.appendRecord({
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: null,
      stage: 'session',
      source: 'dashboard',
      epoch_us: browserEpochUs(),
      clock_offset_to_base_us: this.gatewayClockOffsetUs,
      delta_t0_ms: null,
      metadata: { event: 'session_started' },
    })
    this.emit()
  }

  exportJsonl() {
    const header: LatencyRecord = {
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: null,
      stage: 'session',
      source: 'dashboard',
      epoch_us: browserEpochUs(),
      clock_offset_to_base_us: this.gatewayClockOffsetUs,
      delta_t0_ms: null,
      metadata: {
        event: 'export',
        selected_stream_id: this.selectedStreamId,
        gateway_clock_offset_us: this.gatewayClockOffsetUs,
        gateway_clock_rtt_us: this.gatewayClockRttUs,
        dropped_record_count: this.droppedRecordCount,
        video_timing_sample_count: this.videoTimingSamples.length,
        rocket_link_measurement: 'external_matched_rtp_pcap_report',
        rover_base_clock_sync_required: true,
        chrony_status: this.chronyStatusSnapshot,
      },
    }
    const jsonl = [header, ...this.records].map((record) => JSON.stringify(record)).join('\n')
    const blob = new Blob([`${jsonl}\n`], { type: 'application/x-ndjson;charset=utf-8' })
    const url = URL.createObjectURL(blob)
    const link = document.createElement('a')
    link.href = url
    link.download = `${this.sessionId}.jsonl`
    link.click()
    URL.revokeObjectURL(url)
  }

  private applyRoverTrace(trialId: string, trace: RoverLatencyTrace) {
    if (trace.event === 'command') {
      if (Number.isFinite(trace.rover_rx_epoch_us)) {
        this.setTrialStage(
          trialId,
          'rover_command_received_t3',
          Number(trace.rover_rx_epoch_us),
          'rover',
          { xbee_seq: trace.xbee_seq, wire_timestamp_ms: trace.wire_timestamp_ms }
        )
      }
      if (Number.isFinite(trace.cmd_publish_epoch_us)) {
        this.setTrialStage(
          trialId,
          'rover_cmd_vel_published_t4',
          Number(trace.cmd_publish_epoch_us),
          'rover',
          { target: trace.target, output: trace.output }
        )
      }
    }
  }

  private setTrialStage(
    trialId: string,
    stage: LatencyStage,
    epochUs: number,
    source: LatencySource,
    metadata: Record<string, unknown>,
    streamId?: string
  ) {
    const trial = this.currentTrial
    if (!trial || trial.id !== trialId || trial.stages[stage] != null) return
    trial.stages[stage] = epochUs
    this.appendStageRecord(stage, source, epochUs, metadata, streamId)
  }

  private appendStageRecord(
    stage: LatencyStage,
    source: LatencySource,
    epochUs: number,
    metadata: Record<string, unknown>,
    streamId?: string
  ) {
    const trial = this.currentTrial
    if (!trial) return
    const t0 = trial.stages.input_t0
    const normalizedT0 =
      source === 'dashboard' || this.gatewayClockOffsetUs == null
        ? t0
        : t0 != null
          ? t0 + this.gatewayClockOffsetUs
          : undefined
    this.appendRecord({
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: trial.id,
      stage,
      source,
      stream_id: streamId,
      sequence:
        typeof metadata.xbee_seq === 'number' ? Math.floor(metadata.xbee_seq) : undefined,
      epoch_us: epochUs,
      clock_offset_to_base_us: source === 'dashboard' ? this.gatewayClockOffsetUs : 0,
      delta_t0_ms:
        normalizedT0 == null ? null : Math.round(((epochUs - normalizedT0) / 1000) * 1000) / 1000,
      metadata,
    })
  }

  private appendRecord(record: LatencyRecord) {
    if (this.records.length >= MAX_RECORDS) {
      this.records.shift()
      this.droppedRecordCount += 1
    }
    this.records.push(record)
  }

  private trimPendingRoverTraces() {
    let count = 0
    for (const traces of this.pendingRoverTraces.values()) count += traces.length
    while (count > MAX_PENDING_ROVER_TRACES) {
      const oldestKey = this.pendingRoverTraces.keys().next().value as string | undefined
      if (!oldestKey) break
      count -= this.pendingRoverTraces.get(oldestKey)?.length || 0
      this.pendingRoverTraces.delete(oldestKey)
    }
  }

  private videoFrameKey(streamId: string, baseIngestEpochUs: number) {
    return `${streamId}:${baseIngestEpochUs}`
  }

  private buildSnapshot(): LatencyDiagnosticsSnapshot {
    const trial = this.currentTrial
    const stages = trial?.stages || {}
    const metric = (end?: number, start?: number) =>
      end != null && start != null ? (end - start) / 1000 : null
    const normalizedT0 =
      stages.input_t0 != null && this.gatewayClockOffsetUs != null
        ? stages.input_t0 + this.gatewayClockOffsetUs
        : stages.input_t0
    const baseToBrowserValues =
      this.gatewayClockOffsetUs == null
        ? []
        : this.videoTimingSamples.map(
            (sample) =>
              (sample.browserReceiveEpochUs + this.gatewayClockOffsetUs! -
                sample.baseIngestEpochUs) /
              1000
          )
    const decodeRenderValues = this.videoTimingSamples.map(
      (sample) => (sample.browserRenderEpochUs - sample.browserReceiveEpochUs) / 1000
    )
    const timingDistribution = (values: number[]): VideoTimingDistribution => ({
      count: values.length,
      p50Ms: percentile(values, 0.5),
      p95Ms: percentile(values, 0.95),
    })
    return {
      sessionId: this.sessionId,
      armed: this.armed,
      neutralSeen: this.neutralSeen,
      pendingCommandTrace: this.pendingCommandTrace != null,
      selectedStreamId: this.selectedStreamId,
      currentTrial: trial
        ? {
            id: trial.id,
            stages: { ...trial.stages },
            metrics: {
              computerToGatewayMs: metric(
                stages.gateway_command_received_t1,
                normalizedT0
              ),
              computerToRoverMs: metric(stages.rover_command_received_t3, normalizedT0),
              roverPublishMs: metric(
                stages.rover_cmd_vel_published_t4,
                stages.rover_command_received_t3
              ),
            },
          }
        : null,
      recordCount: this.records.length,
      droppedRecordCount: this.droppedRecordCount,
      videoTiming: {
        sampleCount: this.videoTimingSamples.length,
        baseToBrowser: timingDistribution(baseToBrowserValues),
        decodeRender: timingDistribution(decodeRenderValues),
      },
      clock: {
        syncing: this.clockSyncing,
        offsetUs: this.gatewayClockOffsetUs,
        rttUs: this.gatewayClockRttUs,
        syncedAtEpochUs: this.clockSyncedAtEpochUs,
        error: this.clockError,
      },
    }
  }

  private emit() {
    this.snapshot = this.buildSnapshot()
    for (const listener of this.listeners) listener()
  }
}

export const latencyDiagnostics = new LatencyDiagnostics()
