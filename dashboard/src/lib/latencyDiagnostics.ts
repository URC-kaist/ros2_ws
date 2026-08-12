import {
  buildDownlinkSegments,
  calculateDownlinkMetrics,
  isDownlinkActive,
} from './latency/downlinkTrial'
import { median, percentile } from './latency/statistics'
import { isUplinkActive } from './latency/uplinkTrial'
import type {
  DownlinkError,
  DownlinkPhase,
  DownlinkStage,
  DownlinkStages,
  DownlinkTrialSnapshot,
  UplinkPhase,
} from './latency/types'
import type { AutomatedUplinkLatencyReport } from './rtpLatencyReport'
import type { VideoFrameCorrelation } from './videoProtocol'

export type LatencyStage = DownlinkStage

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

export type TrialStages = DownlinkStages

export type LatencyTrialSnapshot = DownlinkTrialSnapshot

export type VideoTimingDistribution = {
  count: number
  p50Ms: number | null
  p95Ms: number | null
}

export type LatencyDiagnosticsSnapshot = {
  sessionId: string
  pendingCommandTrace: boolean
  selectedStreamId: string | null
  currentTrial: LatencyTrialSnapshot | null
  downlink: {
    phase: DownlinkPhase
    active: boolean
    neutralSeen: boolean
    error: DownlinkError | null
  }
  uplink: {
    phase: UplinkPhase
    active: boolean
    trialId: string | null
    feedCount: number
    streamIds: string[]
    progress: number
    error: DownlinkError | null
    report: AutomatedUplinkLatencyReport | null
  }
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
  startedAtEpochUs: number
  completedAtEpochUs: number | null
  startBrowserToBaseOffsetUs: number | null
  startBrowserRttUs: number | null
  endBrowserToBaseOffsetUs: number | null
  endBrowserRttUs: number | null
}

type PendingRoverTrace = RoverLatencyTrace

type VideoReceiveMetadata = {
  streamId: string
  decodeTimestampUs: number
  baseIngestEpochUs: number
  browserReceiveEpochUs: number
  correlation: VideoFrameCorrelation | null
}

type VideoTimingSample = VideoReceiveMetadata & {
  browserRenderEpochUs: number
}

export type UplinkBrowserSample = {
  stream_id: string
  ssrc: number
  rtp_timestamp: number
  marker_sequence: number
  browser_receive_epoch_us: number
  browser_render_epoch_us: number
}

const MAX_RECORDS = 20_000
const MAX_PENDING_ROVER_TRACES = 512
const MAX_TRACE_MAPPINGS = 2_048
const MAX_VIDEO_TIMING_SAMPLES = 600
const DRIVE_TRIGGER_THRESHOLD = 0.02
const VIDEO_SAMPLES_PER_UI_UPDATE = 8
const DOWNLINK_PREFLIGHT_TIMEOUT_MS = 15_000
const DOWNLINK_NEUTRAL_TIMEOUT_MS = 15_000
const DOWNLINK_INPUT_TIMEOUT_MS = 30_000
const DOWNLINK_TRACE_TIMEOUT_MS = 10_000
const DOWNLINK_END_CLOCK_TIMEOUT_MS = 15_000
const DOWNLINK_MAX_BROWSER_CLOCK_DRIFT_US = 2_000

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
  private downlinkPhase: DownlinkPhase = 'idle'
  private downlinkError: DownlinkError | null = null
  private downlinkTimeout: ReturnType<typeof setTimeout> | null = null
  private uplinkPhase: UplinkPhase = 'idle'
  private uplinkTrialId: string | null = null
  private uplinkFeedCount = 1
  private uplinkStreamIds: string[] = []
  private uplinkProgress = 0
  private uplinkError: DownlinkError | null = null
  private uplinkReport: AutomatedUplinkLatencyReport | null = null
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
  private clockSyncPromise: Promise<boolean> | null = null
  private clockError: string | null = null
  private trialByTraceKey = new Map<string, string>()
  private pendingRoverTraces = new Map<string, PendingRoverTrace[]>()
  private videoReceives = new Map<string, VideoReceiveMetadata>()
  private videoTimingSamples: VideoTimingSample[] = []
  private videoSamplesSinceEmit = 0
  private uplinkBrowserCaptureTrialId: string | null = null
  private uplinkBrowserCaptureStreamIds = new Set<string>()
  private uplinkBrowserSamples: UplinkBrowserSample[] = []
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

  startDownlinkPreflight() {
    if (isDownlinkActive(this.downlinkPhase)) return false
    this.clearDownlinkTimeout()
    this.trialCounter += 1
    const startedAtEpochUs = browserEpochUs()
    this.currentTrial = {
      id: `${this.sessionId}-${String(this.trialCounter).padStart(6, '0')}`,
      stages: {},
      startedAtEpochUs,
      completedAtEpochUs: null,
      startBrowserToBaseOffsetUs: null,
      startBrowserRttUs: null,
      endBrowserToBaseOffsetUs: null,
      endBrowserRttUs: null,
    }
    this.downlinkPhase = 'checking_clocks'
    this.downlinkError = null
    this.neutralSeen = false
    this.pendingCommandTrace = null
    this.trialByTraceKey.clear()
    this.pendingRoverTraces.clear()
    this.scheduleDownlinkTimeout(
      DOWNLINK_PREFLIGHT_TIMEOUT_MS,
      'preflight_timeout',
      'Clock preflight did not complete in time.'
    )
    this.emit()
    return true
  }

  armDownlinkTrial() {
    const trial = this.currentTrial
    if (
      this.downlinkPhase !== 'checking_clocks' ||
      !trial ||
      this.gatewayClockOffsetUs == null
    ) {
      return false
    }
    trial.startBrowserToBaseOffsetUs = this.gatewayClockOffsetUs
    trial.startBrowserRttUs = this.gatewayClockRttUs
    this.downlinkPhase = 'waiting_for_neutral'
    this.scheduleDownlinkTimeout(
      DOWNLINK_NEUTRAL_TIMEOUT_MS,
      'neutral_timeout',
      'Controller neutral position was not detected.'
    )
    this.emit()
    return true
  }

  failDownlinkTrial(code: string, message: string) {
    if (!isDownlinkActive(this.downlinkPhase)) return
    this.finishDownlinkTrial('failed', { code, message })
  }

  cancelDownlinkTrial() {
    if (!isDownlinkActive(this.downlinkPhase)) return
    this.finishDownlinkTrial('cancelled', {
      code: 'trial_cancelled',
      message: 'Downlink measurement was cancelled.',
    })
  }

  startUplinkPreflight(feedCount: number, streamIds: string[]) {
    if (isUplinkActive(this.uplinkPhase)) return false
    this.uplinkPhase = 'checking_clocks'
    this.uplinkTrialId = null
    this.uplinkFeedCount = feedCount
    this.uplinkStreamIds = [...streamIds]
    this.uplinkProgress = 0.05
    this.uplinkError = null
    this.uplinkReport = null
    this.clearUplinkBrowserCapture()
    this.emit()
    return true
  }

  bindUplinkTrial(trialId: string) {
    if (!isUplinkActive(this.uplinkPhase)) return
    this.uplinkTrialId = trialId
    this.uplinkPhase = 'preparing'
    this.uplinkProgress = Math.max(this.uplinkProgress, 0.1)
    this.emit()
  }

  updateUplinkPhase(phase: UplinkPhase, progress: number) {
    if (!isUplinkActive(this.uplinkPhase)) return
    this.uplinkPhase = phase
    this.uplinkProgress = Math.min(1, Math.max(0, progress))
    this.emit()
  }

  completeUplinkTrial(report: AutomatedUplinkLatencyReport) {
    if (!isUplinkActive(this.uplinkPhase)) return
    this.uplinkPhase = 'completed'
    this.uplinkProgress = 1
    this.uplinkReport = report
    this.clearUplinkBrowserCapture()
    this.uplinkError = null
    this.emit()
  }

  failUplinkTrial(code: string, message: string) {
    if (!isUplinkActive(this.uplinkPhase)) return
    this.uplinkPhase = 'failed'
    this.uplinkError = { code, message }
    this.clearUplinkBrowserCapture()
    this.emit()
  }

  cancelUplinkMeasurement() {
    if (!isUplinkActive(this.uplinkPhase)) return
    this.uplinkPhase = 'cancelled'
    this.uplinkError = {
      code: 'trial_cancelled',
      message: 'Uplink measurement was cancelled.',
    }
    this.clearUplinkBrowserCapture()
    this.emit()
  }

  completeDownlinkClockVerification() {
    const trial = this.currentTrial
    if (
      this.downlinkPhase !== 'verifying_clocks' ||
      !trial ||
      this.gatewayClockOffsetUs == null
    ) {
      return false
    }
    trial.endBrowserToBaseOffsetUs = this.gatewayClockOffsetUs
    trial.endBrowserRttUs = this.gatewayClockRttUs
    const startOffsetUs = trial.startBrowserToBaseOffsetUs
    if (startOffsetUs == null) {
      this.finishDownlinkTrial('failed', {
        code: 'start_clock_unavailable',
        message: 'The starting browser/base clock offset is unavailable.',
      })
      return false
    }
    const driftUs = this.gatewayClockOffsetUs - startOffsetUs
    if (Math.abs(driftUs) > DOWNLINK_MAX_BROWSER_CLOCK_DRIFT_US) {
      this.finishDownlinkTrial('failed', {
        code: 'browser_clock_drift',
        message: 'Browser/base clock offset drift exceeded 2 ms during the trial.',
      })
      return false
    }

    trial.completedAtEpochUs = browserEpochUs()
    this.downlinkPhase = 'completed'
    this.downlinkError = null
    this.pendingCommandTrace = null
    this.clearDownlinkTimeout()
    this.appendRecord({
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: trial.id,
      stage: 'session',
      source: 'dashboard',
      epoch_us: trial.completedAtEpochUs,
      clock_offset_to_base_us: startOffsetUs,
      delta_t0_ms: null,
      metadata: {
        event: 'downlink_completed',
        end_browser_to_base_offset_us: this.gatewayClockOffsetUs,
        browser_clock_drift_us: driftUs,
      },
    })
    this.emit()
    return true
  }

  observeControlInput(vector: DriveVector) {
    if (
      this.downlinkPhase !== 'waiting_for_neutral' &&
      this.downlinkPhase !== 'waiting_for_input'
    ) {
      return
    }
    const magnitude = Math.max(Math.abs(vector.x), Math.abs(vector.y), Math.abs(vector.yaw))
    if (magnitude < DRIVE_TRIGGER_THRESHOLD) {
      if (this.downlinkPhase === 'waiting_for_neutral') {
        this.neutralSeen = true
        this.downlinkPhase = 'waiting_for_input'
        this.scheduleDownlinkTimeout(
          DOWNLINK_INPUT_TIMEOUT_MS,
          'input_timeout',
          'No controller command was detected.'
        )
        this.emit()
      }
      return
    }
    if (this.downlinkPhase !== 'waiting_for_input' || !this.neutralSeen) return

    const t0 = browserEpochUs()
    const trial = this.currentTrial
    if (!trial) return
    trial.stages.input_t0 = t0
    this.pendingCommandTrace = { trial_id: trial.id, client_tx_epoch_us: t0 }
    this.downlinkPhase = 'waiting_for_trace'
    this.scheduleDownlinkTimeout(
      DOWNLINK_TRACE_TIMEOUT_MS,
      'trace_timeout',
      'The gateway or rover command trace did not arrive in time.'
    )
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
    if (
      this.downlinkPhase !== 'waiting_for_trace' ||
      !trace.trial_id ||
      trace.trial_id !== this.currentTrial?.id ||
      !Number.isFinite(trace.gateway_rx_epoch_us)
    ) {
      return
    }
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
    this.completeDownlinkIfReady()
    this.emit()
  }

  ingestRoverTrace(trace: RoverLatencyTrace) {
    if (this.downlinkPhase !== 'waiting_for_trace') return
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
    this.completeDownlinkIfReady()
    this.emit()
  }

  observeVideoReceive(
    streamId: string,
    decodeTimestampUs: number,
    baseIngestEpochUs: number,
    browserReceiveEpochUs: number,
    correlation: VideoFrameCorrelation | null
  ) {
    if (this.selectedStreamId == null) {
      this.selectedStreamId = streamId
      this.emit()
    }
    const collectingUplink =
      this.uplinkBrowserCaptureTrialId != null &&
      this.uplinkBrowserCaptureStreamIds.has(streamId) &&
      correlation != null
    if (streamId !== this.selectedStreamId && !collectingUplink) return
    const key = this.videoFrameKey(streamId, decodeTimestampUs)
    this.videoReceives.set(key, {
      streamId,
      decodeTimestampUs,
      baseIngestEpochUs,
      browserReceiveEpochUs,
      correlation,
    })
    if (this.videoReceives.size > MAX_VIDEO_TIMING_SAMPLES * 2) {
      const oldestKey = this.videoReceives.keys().next().value as string | undefined
      if (oldestKey) this.videoReceives.delete(oldestKey)
    }
  }

  observeVideoRender(streamId: string, decodeTimestampUs: number, browserRenderEpochUs: number) {
    const key = this.videoFrameKey(streamId, decodeTimestampUs)
    const receive = this.videoReceives.get(key)
    this.videoReceives.delete(key)
    if (!receive) return

    if (
      this.uplinkBrowserCaptureTrialId != null &&
      this.uplinkBrowserCaptureStreamIds.has(streamId) &&
      receive.correlation
    ) {
      this.uplinkBrowserSamples.push({
        stream_id: streamId,
        ssrc: receive.correlation.ssrc,
        rtp_timestamp: receive.correlation.rtpTimestamp,
        marker_sequence: receive.correlation.markerSequence,
        browser_receive_epoch_us: receive.browserReceiveEpochUs,
        browser_render_epoch_us: browserRenderEpochUs,
      })
      if (this.uplinkBrowserSamples.length > MAX_VIDEO_TIMING_SAMPLES * 4) {
        this.uplinkBrowserSamples.shift()
      }
    }

    if (streamId !== this.selectedStreamId) return

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

  beginUplinkBrowserCapture(trialId: string, streamIds: string[]) {
    if (trialId !== this.uplinkTrialId || !isUplinkActive(this.uplinkPhase)) return false
    this.uplinkBrowserCaptureTrialId = trialId
    this.uplinkBrowserCaptureStreamIds = new Set(streamIds)
    this.uplinkBrowserSamples = []
    this.videoReceives.clear()
    return true
  }

  finishUplinkBrowserCapture(trialId: string) {
    if (trialId !== this.uplinkBrowserCaptureTrialId) return []
    const samples = [...this.uplinkBrowserSamples]
    this.clearUplinkBrowserCapture()
    return samples
  }

  private clearUplinkBrowserCapture() {
    this.uplinkBrowserCaptureTrialId = null
    this.uplinkBrowserCaptureStreamIds.clear()
    this.uplinkBrowserSamples = []
    this.videoReceives.clear()
  }

  synchronizeGatewayClock(sampleCount = 8): Promise<boolean> {
    if (this.clockSyncPromise) return this.clockSyncPromise
    this.clockSyncing = true
    this.clockError = null
    this.emit()

    this.clockSyncPromise = (async () => {
      try {
        const samples: Array<{ offsetUs: number; rttUs: number }> = []
        let latestBaseSampleEpochUs: number | null = null
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
            browserReceiveUs - browserSendUs -
            Math.max(0, serverSendUs - serverReceiveUs)
          const offsetUs =
            ((serverReceiveUs - browserSendUs) +
              (serverSendUs - browserReceiveUs)) /
            2
          samples.push({ offsetUs, rttUs })
          // The gateway uses this timestamp to reject old preflight snapshots.
          // Base it on the server wall clock: performance.timeOrigin remains
          // fixed when NTP/Chrony steps wall time in a long-lived browser tab.
          latestBaseSampleEpochUs = serverSendUs
        }
        samples.sort((left, right) => left.rttUs - right.rttUs)
        const selected = samples.slice(0, Math.max(1, Math.ceil(samples.length / 2)))
        this.gatewayClockOffsetUs = median(selected.map((sample) => sample.offsetUs))
        this.gatewayClockRttUs = median(selected.map((sample) => sample.rttUs))
        this.clockSyncedAtEpochUs = latestBaseSampleEpochUs
        return this.gatewayClockOffsetUs != null
      } catch (error) {
        this.clockError = error instanceof Error ? error.message : 'Clock synchronization failed'
        this.gatewayClockOffsetUs = null
        this.gatewayClockRttUs = null
        return false
      } finally {
        this.clockSyncing = false
        this.clockSyncPromise = null
        this.emit()
      }
    })()
    return this.clockSyncPromise
  }

  resetSession() {
    this.clearDownlinkTimeout()
    this.sessionId = createSessionId()
    this.trialCounter = 0
    this.downlinkPhase = 'idle'
    this.downlinkError = null
    this.neutralSeen = false
    this.pendingCommandTrace = null
    this.currentTrial = null
    this.uplinkPhase = 'idle'
    this.uplinkTrialId = null
    this.uplinkProgress = 0
    this.uplinkError = null
    this.uplinkReport = null
    this.clearUplinkBrowserCapture()
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
        downlink_result: this.snapshot.currentTrial,
        uplink_report: this.uplinkReport,
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

  private completeDownlinkIfReady() {
    const trial = this.currentTrial
    if (this.downlinkPhase !== 'waiting_for_trace' || !trial) return
    const stages = trial.stages
    if (
      stages.input_t0 == null ||
      stages.gateway_command_received_t1 == null ||
      stages.rover_command_received_t3 == null ||
      stages.rover_cmd_vel_published_t4 == null ||
      trial.startBrowserToBaseOffsetUs == null
    ) {
      return
    }
    this.downlinkPhase = 'verifying_clocks'
    this.pendingCommandTrace = null
    this.scheduleDownlinkTimeout(
      DOWNLINK_END_CLOCK_TIMEOUT_MS,
      'end_clock_timeout',
      'The ending clock verification did not complete in time.'
    )
    this.appendRecord({
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: trial.id,
      stage: 'session',
      source: 'dashboard',
      epoch_us: browserEpochUs(),
      clock_offset_to_base_us: trial.startBrowserToBaseOffsetUs,
      delta_t0_ms: null,
      metadata: { event: 'downlink_trace_completed' },
    })
  }

  private finishDownlinkTrial(
    phase: Extract<DownlinkPhase, 'failed' | 'cancelled'>,
    error: DownlinkError
  ) {
    this.clearDownlinkTimeout()
    this.downlinkPhase = phase
    this.downlinkError = error
    this.pendingCommandTrace = null
    const finishedAtEpochUs = browserEpochUs()
    if (this.currentTrial) {
      this.currentTrial.completedAtEpochUs = finishedAtEpochUs
    }
    this.appendRecord({
      schema_version: 1,
      session_id: this.sessionId,
      trial_id: this.currentTrial?.id ?? null,
      stage: 'session',
      source: 'dashboard',
      epoch_us: finishedAtEpochUs,
      clock_offset_to_base_us: this.gatewayClockOffsetUs,
      delta_t0_ms: null,
      metadata: { event: `downlink_${phase}`, error_code: error.code, error: error.message },
    })
    this.emit()
  }

  private scheduleDownlinkTimeout(timeoutMs: number, code: string, message: string) {
    this.clearDownlinkTimeout()
    this.downlinkTimeout = setTimeout(() => {
      this.downlinkTimeout = null
      this.failDownlinkTrial(code, message)
    }, timeoutMs)
  }

  private clearDownlinkTimeout() {
    if (this.downlinkTimeout == null) return
    clearTimeout(this.downlinkTimeout)
    this.downlinkTimeout = null
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
    if (
      this.downlinkPhase !== 'waiting_for_trace' ||
      !trial ||
      trial.id !== trialId ||
      trial.stages[stage] != null
    ) {
      return
    }
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

  private videoFrameKey(streamId: string, decodeTimestampUs: number) {
    return `${streamId}:${decodeTimestampUs}`
  }

  private buildSnapshot(): LatencyDiagnosticsSnapshot {
    const trial = this.currentTrial
    const stages = trial?.stages || {}
    const browserClockDriftUs =
      trial?.startBrowserToBaseOffsetUs != null &&
      trial.endBrowserToBaseOffsetUs != null
        ? trial.endBrowserToBaseOffsetUs - trial.startBrowserToBaseOffsetUs
        : null
    const downlinkMetrics = calculateDownlinkMetrics(
      stages,
      trial?.startBrowserToBaseOffsetUs ?? null,
      browserClockDriftUs
    )
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
      pendingCommandTrace: this.pendingCommandTrace != null,
      selectedStreamId: this.selectedStreamId,
      currentTrial: trial
        ? {
            id: trial.id,
            phase: this.downlinkPhase,
            stages: { ...trial.stages },
            metrics: downlinkMetrics,
            segments: buildDownlinkSegments(downlinkMetrics),
            startedAtEpochUs: trial.startedAtEpochUs,
            completedAtEpochUs: trial.completedAtEpochUs,
            clock: {
              startBrowserToBaseOffsetUs: trial.startBrowserToBaseOffsetUs,
              startBrowserRttUs: trial.startBrowserRttUs,
              endBrowserToBaseOffsetUs: trial.endBrowserToBaseOffsetUs,
              endBrowserRttUs: trial.endBrowserRttUs,
              browserClockDriftUs,
            },
            error: this.downlinkError,
          }
        : null,
      downlink: {
        phase: this.downlinkPhase,
        active: isDownlinkActive(this.downlinkPhase),
        neutralSeen: this.neutralSeen,
        error: this.downlinkError,
      },
      uplink: {
        phase: this.uplinkPhase,
        active: isUplinkActive(this.uplinkPhase),
        trialId: this.uplinkTrialId,
        feedCount: this.uplinkFeedCount,
        streamIds: [...this.uplinkStreamIds],
        progress: this.uplinkProgress,
        error: this.uplinkError,
        report: this.uplinkReport,
      },
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
