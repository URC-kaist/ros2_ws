import { useCallback, useEffect, useRef, useState } from 'react'
import { FiArrowDown, FiArrowUp, FiClock, FiDownload, FiRotateCcw, FiX } from 'react-icons/fi'
import { useLatencyClockReadiness } from '../hooks/useLatencyClockReadiness'
import { useLatencyDiagnostics } from '../hooks/useLatencyDiagnostics'
import { useRosBridge } from '../hooks/useRosBridge'
import { useVideoStreams } from '../hooks/useVideoStreams'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import { downlinkPhaseLabel } from '../lib/latency/downlinkTrial'
import {
  cancelUplinkTrial,
  createUplinkTrial,
  getUplinkTrial,
  startUplinkTrial,
  uploadUplinkBrowserSamples,
} from '../lib/latency/latencyApi'
import { mapGatewayUplinkPhase, uplinkPhaseLabel } from '../lib/latency/uplinkTrial'
import { selectOnlineUplinkStreams } from '../lib/latency/uplinkStreamSelection'
import {
  chronyStatusForLog,
} from '../lib/chronyStatus'
import {
  latencyDiagnostics,
  type RoverLatencyTrace,
} from '../lib/latencyDiagnostics'
import { parseAutomatedUplinkReport } from '../lib/rtpLatencyReport'
import { fetchVideoStreams } from '../lib/videoGateway'
import type { UplinkTrialStatusMsg } from '../lib/rosMessages'
import LatencyResultTable from './LatencyResultTable'
import VideoStreamCard from './VideoStreamCard'
import './ControlPanel/LatencyDiagnosticsPanel.css'

function formatMs(value: number | null | undefined) {
  return value == null || !Number.isFinite(value) ? '--' : `${value.toFixed(1)} ms`
}

function formatUsAsMs(value: number | null | undefined) {
  return value == null || !Number.isFinite(value) ? '--' : `${(value / 1000).toFixed(2)} ms`
}

function uplinkErrorCode(error: unknown) {
  const explicit = (error as { code?: unknown } | null)?.code
  if (typeof explicit === 'string') return explicit
  const message = error instanceof Error ? error.message.toLowerCase() : ''
  if (message.includes('browser/base') || message.includes('clock offset')) {
    return 'browser_clock_unreliable'
  }
  if (message.includes('clock') || message.includes('chrony')) return 'clock_not_ready'
  if (message.includes('correlated browser frame')) return 'browser_samples_missing'
  if (message.includes('render') || message.includes('video feed')) return 'stream_warmup_timeout'
  if (message.includes('ros bridge') || message.includes('rover')) return 'rover_agent_unavailable'
  return 'uplink_measurement_failed'
}

const LatencyDiagnosticsPanel = () => {
  const snapshot = useLatencyDiagnostics()
  const { gateway, connected: gatewayConnected } = useXbeeGateway()
  const { ros, connected: rosConnected } = useRosBridge()
  const clockReadiness = useLatencyClockReadiness(ros)
  const checkClockNow = clockReadiness.checkNow
  const { streams, loading: streamsLoading, error: streamsError } = useVideoStreams()
  const [excludedUplinkStreamIds, setExcludedUplinkStreamIds] = useState<Set<string>>(
    () => new Set()
  )
  const [uplinkElapsedS, setUplinkElapsedS] = useState(0)
  const uplinkOperationRef = useRef(0)
  const renderedUplinkStreamsRef = useRef(new Set<string>())
  const roverUplinkStatusRef = useRef<UplinkTrialStatusMsg | null>(null)

  const markUplinkFrameRendered = useCallback((streamId: string) => {
    renderedUplinkStreamsRef.current.add(streamId)
  }, [])

  useEffect(
    () => gateway.onLatencyTrace((trace) => latencyDiagnostics.ingestGatewayTrace(trace)),
    [gateway]
  )

  useEffect(() => {
    return ros.subscribe<{ data?: string }>(
      '/latency/trace',
      'std_msgs/msg/String',
      (message) => {
        if (typeof message.data !== 'string') return
        try {
          const trace = JSON.parse(message.data) as RoverLatencyTrace
          if (trace.event !== 'command') return
          latencyDiagnostics.ingestRoverTrace(trace)
        } catch {
          // Malformed diagnostics must not affect command or video operation.
        }
      },
      { queueSize: 10 }
    )
  }, [ros])

  useEffect(() => {
    return ros.subscribe<UplinkTrialStatusMsg>(
      '/latency/uplink/status',
      'mr2_latency_msgs/msg/UplinkTrialStatus',
      (message) => {
        roverUplinkStatusRef.current = message
      },
      { queueSize: 10 }
    )
  }, [ros])

  useEffect(() => {
    if (!snapshot.uplink.active) {
      setUplinkElapsedS(0)
      return
    }
    const startedAt = Date.now()
    const timer = window.setInterval(() => {
      setUplinkElapsedS(Math.floor((Date.now() - startedAt) / 1000))
    }, 500)
    return () => window.clearInterval(timer)
  }, [snapshot.uplink.active])

  const trial = snapshot.currentTrial
  const onlineUplinkStreams = streams.filter((stream) => stream.available)
  const selectedUplinkStreams = selectOnlineUplinkStreams(
    streams,
    excludedUplinkStreamIds
  )
  const measuredUplinkStreams = snapshot.uplink.active
    ? streams.filter((stream) => snapshot.uplink.streamIds.includes(stream.stream_id))
    : selectedUplinkStreams
  const automatedUplinkReport = snapshot.uplink.report
  const downlinkStatusTone =
    snapshot.downlink.phase === 'completed'
      ? trial?.metrics.valid
        ? 'ready'
        : 'blocked'
      : snapshot.downlink.phase === 'failed'
        ? 'blocked'
        : snapshot.downlink.active
          ? 'measuring'
          : 'idle'
  const uplinkStatusTone =
    snapshot.uplink.phase === 'completed'
      ? 'ready'
      : snapshot.uplink.phase === 'failed'
        ? 'blocked'
        : snapshot.uplink.active
          ? 'measuring'
          : 'idle'

  const startDownlinkMeasurement = async () => {
    if (!latencyDiagnostics.startDownlinkPreflight()) return
    if (!gatewayConnected) {
      latencyDiagnostics.failDownlinkTrial(
        'gateway_unavailable',
        'Base gateway WebSocket is not connected.'
      )
      return
    }
    if (!rosConnected) {
      latencyDiagnostics.failDownlinkTrial(
        'rover_trace_unavailable',
        'ROS bridge is not connected for rover trace collection.'
      )
      return
    }

    const readiness = await clockReadiness.checkNow()
    if (!readiness.ready) {
      latencyDiagnostics.failDownlinkTrial(
        'clock_not_ready',
        readiness.reasons.join(' ')
      )
      return
    }

    const browserClockReady = await latencyDiagnostics.synchronizeGatewayClock()
    if (!browserClockReady) {
      latencyDiagnostics.failDownlinkTrial(
        'browser_clock_unavailable',
        latencyDiagnostics.getSnapshot().clock.error ||
          'Browser/base clock offset could not be measured.'
      )
      return
    }
    latencyDiagnostics.armDownlinkTrial()
  }

  const waitForSelectedStreams = async (streamIds: string[], operationId: number) => {
    const deadline = Date.now() + 12_000
    while (Date.now() < deadline) {
      if (uplinkOperationRef.current !== operationId) {
        throw new Error('Uplink measurement was cancelled.')
      }
      const nextStreams = await fetchVideoStreams()
      const byId = new Map(nextStreams.map((stream) => [stream.stream_id, stream]))
      if (streamIds.every((streamId) => byId.get(streamId)?.available)) return
      await new Promise((resolve) => window.setTimeout(resolve, 500))
    }
    throw new Error('Selected video feeds did not become available in time.')
  }

  const waitForRenderedStreams = async (streamIds: string[], operationId: number) => {
    const deadline = Date.now() + 12_000
    while (Date.now() < deadline) {
      if (uplinkOperationRef.current !== operationId) {
        throw new Error('Uplink measurement was cancelled.')
      }
      if (streamIds.every((streamId) => renderedUplinkStreamsRef.current.has(streamId))) return
      await new Promise((resolve) => window.setTimeout(resolve, 100))
    }
    throw new Error('Selected video feeds did not render in the browser in time.')
  }

  const throwIfRoverTrialFailed = (trialId: string) => {
    const status = roverUplinkStatusRef.current
    if (status?.trial_id !== trialId || status.phase !== 'failed') return
    const error = new Error(
      status.message || 'Rover capture failed before artifacts were uploaded.'
    ) as Error & { code?: string }
    error.code = status.error_code || 'rover_capture_failed'
    throw error
  }

  const waitForCaptureWindow = async (
    trialId: string,
    durationS: number,
    operationId: number
  ) => {
    const deadline = Date.now() + (durationS + 0.75) * 1000
    while (Date.now() < deadline) {
      if (uplinkOperationRef.current !== operationId) return false
      throwIfRoverTrialFailed(trialId)
      await new Promise((resolve) => window.setTimeout(resolve, 100))
    }
    throwIfRoverTrialFailed(trialId)
    return true
  }

  const startUplinkMeasurement = async () => {
    const streamIds = selectedUplinkStreams.map((stream) => stream.stream_id)
    const feedCount = streamIds.length
    if (feedCount === 0) return
    if (!latencyDiagnostics.startUplinkPreflight(feedCount, streamIds)) return
    renderedUplinkStreamsRef.current.clear()
    roverUplinkStatusRef.current = null
    const operationId = ++uplinkOperationRef.current
    let trialId: string | null = null
    let roverPrepared = false
    try {
      if (!gatewayConnected) throw new Error('Base gateway WebSocket is not connected.')
      if (!rosConnected) throw new Error('ROS bridge is not connected.')
      const initialReadiness = await checkClockNow()
      if (!initialReadiness.ready) throw new Error(initialReadiness.reasons.join(' '))

      // Five seconds yields many matched RTP marker frames at camera frame
      // rates without delaying results with an unnecessarily long pcap.
      const created = await createUplinkTrial(feedCount, streamIds, 5)
      trialId = created.trial_id
      latencyDiagnostics.bindUplinkTrial(trialId)
      const prepared = await ros.callService<
        { trial_id: string; stream_ids: string[] },
        { accepted: boolean; message: string }
      >(
        '/latency/uplink/prepare',
        'mr2_latency_msgs/srv/PrepareUplinkTrial',
        { trial_id: trialId, stream_ids: streamIds }
      )
      if (!prepared.accepted) throw new Error(prepared.message || 'Rover prepare failed.')
      roverPrepared = true
      await waitForSelectedStreams(streamIds, operationId)
      await waitForRenderedStreams(streamIds, operationId)

      // Preparation and stream warm-up can take longer than the gateway's
      // five-second freshness window. Take both authoritative clock snapshots
      // only after video is ready and immediately before starting capture.
      const startReadiness = await checkClockNow()
      if (!startReadiness.ready) throw new Error(startReadiness.reasons.join(' '))
      if (!(await latencyDiagnostics.synchronizeGatewayClock())) {
        throw new Error(
          latencyDiagnostics.getSnapshot().clock.error ||
            'Browser/base clock offset could not be measured.'
        )
      }
      const clock = latencyDiagnostics.getSnapshot().clock
      if (clock.offsetUs == null || clock.rttUs == null || clock.syncedAtEpochUs == null) {
        throw new Error('Browser/base clock snapshot is unavailable.')
      }
      const chronySnapshot = {
        ready: startReadiness.ready,
        reasons: startReadiness.reasons,
        rover_error_bound_s: startReadiness.roverErrorBoundS,
        base: chronyStatusForLog(startReadiness.base),
        rover: chronyStatusForLog(startReadiness.rover),
      }
      const started = await startUplinkTrial(
        trialId,
        {
          offset_us: clock.offsetUs,
          rtt_us: clock.rttUs,
          sampled_at_epoch_us: clock.syncedAtEpochUs,
        },
        chronySnapshot
      )
      if (!started.upload_base_url || !started.upload_token) {
        throw new Error('Base gateway did not provide rover upload credentials.')
      }
      const startBrowserClockOffsetUs = clock.offsetUs
      if (!latencyDiagnostics.beginUplinkBrowserCapture(trialId, streamIds)) {
        throw new Error('Browser frame capture could not be started.')
      }
      const roverStarted = await ros.callService<
        {
          trial_id: string
          duration_s: number
          upload_base_url: string
          upload_token: string
        },
        { accepted: boolean; message: string }
      >(
        '/latency/uplink/start',
        'mr2_latency_msgs/srv/StartUplinkTrial',
        {
          trial_id: trialId,
          duration_s: started.duration_s,
          upload_base_url: started.upload_base_url,
          upload_token: started.upload_token,
        }
      )
      if (!roverStarted.accepted) {
        throw new Error(roverStarted.message || 'Rover capture failed to start.')
      }
      latencyDiagnostics.updateUplinkPhase('capturing', 0.3)

      if (!(await waitForCaptureWindow(trialId, started.duration_s, operationId))) return
      const browserSamples = latencyDiagnostics.finishUplinkBrowserCapture(trialId)
      if (browserSamples.length === 0) {
        throw new Error('No correlated browser frames were rendered during capture.')
      }
      const endReadiness = await checkClockNow()
      if (!endReadiness.ready) {
        throw new Error(`Clock verification failed after capture: ${endReadiness.reasons.join(' ')}`)
      }
      if (!(await latencyDiagnostics.synchronizeGatewayClock(4))) {
        throw new Error('Ending browser/base clock offset could not be measured.')
      }
      const endOffsetUs = latencyDiagnostics.getSnapshot().clock.offsetUs
      if (endOffsetUs == null || Math.abs(endOffsetUs - startBrowserClockOffsetUs) > 2_000) {
        throw new Error('Browser/base clock offset drift exceeded 2 ms during capture.')
      }
      latencyDiagnostics.updateUplinkPhase('uploading', 0.6)
      await uploadUplinkBrowserSamples(trialId, browserSamples)

      const deadline = Date.now() + (started.duration_s + 45) * 1000
      while (Date.now() < deadline) {
        if (uplinkOperationRef.current !== operationId) return
        throwIfRoverTrialFailed(trialId)
        const status = await getUplinkTrial(trialId)
        const phase = status.phase === 'waiting_for_rover_artifacts' &&
          status.browser_samples_received
          ? 'uploading'
          : mapGatewayUplinkPhase(status.phase)
        if (phase === 'completed') {
          if (!status.report) throw new Error('Uplink report is missing.')
          latencyDiagnostics.completeUplinkTrial(parseAutomatedUplinkReport(status.report))
          return
        }
        if (phase === 'failed' || phase === 'cancelled') {
          const failure = new Error(status.error?.message || `Uplink trial ${phase}.`) as Error & {
            code?: string
          }
          failure.code = status.error?.code
          throw failure
        }
        latencyDiagnostics.updateUplinkPhase(phase, status.progress)
        await new Promise((resolve) => window.setTimeout(resolve, 750))
      }
      throw new Error('Uplink measurement timed out.')
    } catch (error) {
      if (uplinkOperationRef.current !== operationId) return
      if (roverPrepared && trialId) {
        void ros.callService(
          '/latency/uplink/cancel',
          'mr2_latency_msgs/srv/CancelUplinkTrial',
          { trial_id: trialId }
        ).catch(() => undefined)
      }
      if (trialId) void cancelUplinkTrial(trialId).catch(() => undefined)
      latencyDiagnostics.failUplinkTrial(
        uplinkErrorCode(error),
        error instanceof Error ? error.message : 'Uplink measurement failed.'
      )
    }
  }

  const cancelUplinkMeasurement = async () => {
    const trialId = snapshot.uplink.trialId
    uplinkOperationRef.current += 1
    latencyDiagnostics.cancelUplinkMeasurement()
    if (!trialId) return
    await Promise.allSettled([
      cancelUplinkTrial(trialId),
      ros.callService(
        '/latency/uplink/cancel',
        'mr2_latency_msgs/srv/CancelUplinkTrial',
        { trial_id: trialId }
      ),
    ])
  }

  useEffect(() => {
    latencyDiagnostics.setChronyStatusSnapshot({
      ready: clockReadiness.readiness.ready,
      reasons: clockReadiness.readiness.reasons,
      rover_error_bound_s: clockReadiness.readiness.roverErrorBoundS,
      base: chronyStatusForLog(clockReadiness.base),
      rover: chronyStatusForLog(clockReadiness.rover),
    })
  }, [
    clockReadiness.base,
    clockReadiness.readiness.ready,
    clockReadiness.readiness.reasons,
    clockReadiness.readiness.roverErrorBoundS,
    clockReadiness.rover,
  ])

  useEffect(() => {
    if (snapshot.downlink.phase !== 'verifying_clocks') return
    let active = true
    const verifyEndClocks = async () => {
      const readiness = await checkClockNow()
      if (!active) return
      if (!readiness.ready) {
        latencyDiagnostics.failDownlinkTrial(
          'end_clock_not_ready',
          readiness.reasons.join(' ')
        )
        return
      }
      const browserClockReady = await latencyDiagnostics.synchronizeGatewayClock(4)
      if (!active) return
      if (!browserClockReady) {
        latencyDiagnostics.failDownlinkTrial(
          'end_browser_clock_unavailable',
          latencyDiagnostics.getSnapshot().clock.error ||
            'The ending browser/base clock offset could not be measured.'
        )
        return
      }
      latencyDiagnostics.completeDownlinkClockVerification()
    }
    void verifyEndClocks()
    return () => {
      active = false
    }
  }, [checkClockNow, snapshot.downlink.phase])

  return (
    <section className="latency-diagnostics" aria-label="Latency diagnostics">
      <div className="latency-diagnostics__header">
        <strong>Latency diagnostics</strong>
        <span>{snapshot.recordCount} events</span>
      </div>

      <fieldset className="latency-diagnostics__camera-selection">
        <legend>Uplink cameras ({onlineUplinkStreams.length} online)</legend>
        {onlineUplinkStreams.map((stream) => (
          <label key={stream.stream_id} className="latency-diagnostics__camera-option">
            <input
              type="checkbox"
              checked={!excludedUplinkStreamIds.has(stream.stream_id)}
              disabled={snapshot.uplink.active}
              onChange={(event) => {
                const checked = event.target.checked
                setExcludedUplinkStreamIds((current) => {
                  const next = new Set(current)
                  if (checked) {
                    next.delete(stream.stream_id)
                  } else {
                    next.add(stream.stream_id)
                  }
                  return next
                })
              }}
            />
            <span>{stream.display.label || stream.stream_id}</span>
          </label>
        ))}
        {onlineUplinkStreams.length === 0 && (
          <small>{streamsLoading ? 'Checking camera availability...' : 'No cameras online'}</small>
        )}
      </fieldset>
      {streamsError && (
        <small className="latency-diagnostics__error">{streamsError}</small>
      )}
      <small>
        {selectedUplinkStreams.length > 0
          ? `${selectedUplinkStreams.length} camera${selectedUplinkStreams.length === 1 ? '' : 's'} selected`
          : 'Select at least one online camera'}
      </small>

      <div className="latency-diagnostics__actions latency-diagnostics__actions--primary">
        <button
          type="button"
          disabled={snapshot.downlink.active}
          title="Measure downlink latency"
          aria-label="Measure downlink latency"
          onClick={() => void startDownlinkMeasurement()}
        >
          <FiArrowDown aria-hidden="true" />
          {snapshot.downlink.phase === 'checking_clocks'
            ? 'Checking...'
            : 'Downlink'}
        </button>
        <button
          type="button"
          disabled={snapshot.uplink.active || selectedUplinkStreams.length === 0}
          title="Measure uplink latency"
          aria-label="Measure uplink latency"
          onClick={() => void startUplinkMeasurement()}
        >
          <FiArrowUp aria-hidden="true" />
          {snapshot.uplink.phase === 'checking_clocks'
            ? 'Checking...'
            : 'Uplink'}
        </button>
        <button
          type="button"
          disabled={clockReadiness.checking}
          title="Check chrony synchronization"
          aria-label="Check chrony synchronization"
          onClick={() => void clockReadiness.checkNow()}
        >
          <FiClock aria-hidden="true" />
          {clockReadiness.checking ? 'Checking...' : 'Chrony sync'}
        </button>
      </div>

      {snapshot.uplink.active && measuredUplinkStreams.length > 0 && (
        <div className="latency-diagnostics__feed-grid" aria-label="Measured Uplink feeds">
          {measuredUplinkStreams.map((stream) => (
            <div key={stream.stream_id} className="latency-diagnostics__feed">
              <span>{stream.display.label || stream.stream_id}</span>
              <VideoStreamCard
                stream={stream}
                onFrameRendered={markUplinkFrameRendered}
              />
            </div>
          ))}
        </div>
      )}

      <div
        className={`latency-diagnostics__readiness latency-diagnostics__readiness--${downlinkStatusTone}`}
        role="status"
      >
        {downlinkPhaseLabel(snapshot.downlink.phase)}
      </div>
      {snapshot.downlink.active && (
        <button
          className="latency-diagnostics__secondary-command"
          type="button"
          onClick={() => latencyDiagnostics.cancelDownlinkTrial()}
        >
          <FiX aria-hidden="true" />
          Cancel measurement
        </button>
      )}
      {snapshot.downlink.error && (
        <div className="latency-diagnostics__error">
          {snapshot.downlink.error.message}
        </div>
      )}

      <div className="latency-diagnostics__section-title">Downlink result</div>
      <div className="latency-diagnostics__segments">
        {(trial?.segments ?? []).map((segment) => (
          <div key={segment.id}>
            <span>{segment.label}</span>
            <strong>{formatMs(segment.distribution.p50Ms)}</strong>
          </div>
        ))}
        {!trial && <small>No downlink measurement yet.</small>}
      </div>
      {trial?.metrics.warning && (
        <div className="latency-diagnostics__error">{trial.metrics.warning}</div>
      )}
      {trial?.clock.browserClockDriftUs != null && (
        <div className="latency-diagnostics__clock">
          Browser/base clock drift {formatUsAsMs(trial.clock.browserClockDriftUs)}
        </div>
      )}

      <div className="latency-diagnostics__section-title">Uplink result</div>
      <div
        className={`latency-diagnostics__readiness latency-diagnostics__readiness--${uplinkStatusTone}`}
        role="status"
      >
        {uplinkPhaseLabel(snapshot.uplink.phase)}
        {snapshot.uplink.active &&
          ` / ${uplinkElapsedS}s / ${snapshot.uplink.feedCount} feeds / ${Math.round(snapshot.uplink.progress * 100)}%`}
      </div>
      {snapshot.uplink.active && (
        <button
          className="latency-diagnostics__secondary-command"
          type="button"
          onClick={() => void cancelUplinkMeasurement()}
        >
          <FiX aria-hidden="true" />
          Cancel uplink measurement
        </button>
      )}
      {snapshot.uplink.error && (
        <div className="latency-diagnostics__error">{snapshot.uplink.error.message}</div>
      )}
      {automatedUplinkReport ? (
        <>
          <div className="latency-diagnostics__result-heading">
            <strong>Aggregate</strong>
            <span>
              {automatedUplinkReport.matched_frames} frames / {automatedUplinkReport.feed_count} feeds
            </span>
          </div>
          <LatencyResultTable segments={automatedUplinkReport.aggregate} />
        </>
      ) : (
        <small>No Uplink measurement yet.</small>
      )}
      {automatedUplinkReport && (
        <div className="latency-diagnostics__stream-results">
          {automatedUplinkReport.streams.map((stream) => (
            <section key={stream.stream_id} className="latency-diagnostics__stream-result">
              <div className="latency-diagnostics__result-heading">
                <strong>{stream.stream_id}</strong>
                <span>
                  {stream.matched_frames} frames /{' '}
                  {stream.packet_loss_percent == null
                    ? '-- loss'
                    : `${stream.packet_loss_percent.toFixed(1)}% loss`}
                </span>
              </div>
              <LatencyResultTable segments={stream.segments} />
              {stream.warnings.map((warning) => (
                <small className="latency-diagnostics__warning" key={warning}>{warning}</small>
              ))}
            </section>
          ))}
          {automatedUplinkReport.warnings.map((warning) => (
            <small className="latency-diagnostics__warning" key={warning}>{warning}</small>
          ))}
        </div>
      )}

      <div className="latency-diagnostics__clock">
        Base - browser{' '}
        {formatMs(snapshot.clock.offsetUs == null ? null : snapshot.clock.offsetUs / 1000)}
        {' / '}RTT{' '}
        {formatMs(snapshot.clock.rttUs == null ? null : snapshot.clock.rttUs / 1000)}
        {snapshot.clock.error && (
          <span className="latency-diagnostics__error"> {snapshot.clock.error}</span>
        )}
      </div>

      <div className="latency-diagnostics__section-title">Base/rover chrony</div>
      <div
        className={`latency-diagnostics__readiness latency-diagnostics__readiness--${clockReadiness.readiness.ready ? 'ready' : 'blocked'}`}
      >
        {clockReadiness.readiness.ready
          ? 'Ready for synchronized measurement'
          : 'Clock sync not ready'}
      </div>
      <div className="latency-diagnostics__summary">
        <div>
          <span>Base</span>
          <strong>
            {clockReadiness.base?.synchronized
              ? 'Synchronized'
              : clockReadiness.base
                ? 'Not ready'
                : '--'}
          </strong>
        </div>
        <div>
          <span>Base reference</span>
          <strong>
            {clockReadiness.base?.referenceName ??
              clockReadiness.base?.referenceId ??
              '--'}
          </strong>
        </div>
        <div>
          <span>Rover</span>
          <strong>
            {clockReadiness.rover?.synchronized
              ? 'Synchronized'
              : clockReadiness.rover
                ? 'Not ready'
                : '--'}
          </strong>
        </div>
        <div>
          <span>Rover reference</span>
          <strong>
            {clockReadiness.rover?.referenceName ??
              clockReadiness.rover?.referenceId ??
              '--'}
          </strong>
        </div>
        <div>
          <span>Rover residual</span>
          <strong>
            {formatUsAsMs(
              clockReadiness.rover?.systemTimeOffsetS == null
                ? null
                : clockReadiness.rover.systemTimeOffsetS * 1_000_000
            )}
          </strong>
        </div>
        <div>
          <span>Rover error bound</span>
          <strong>
            {formatUsAsMs(
              clockReadiness.readiness.roverErrorBoundS == null
                ? null
                : clockReadiness.readiness.roverErrorBoundS * 1_000_000
            )}
          </strong>
        </div>
        <div>
          <span>Stratum base/rover</span>
          <strong>
            {clockReadiness.base?.stratum ?? '--'} /{' '}
            {clockReadiness.rover?.stratum ?? '--'}
          </strong>
        </div>
      </div>
      {!clockReadiness.readiness.ready && (
        <div className="latency-diagnostics__reasons">
          {clockReadiness.readiness.reasons.map((reason) => (
            <small key={reason}>{reason}</small>
          ))}
        </div>
      )}
      {clockReadiness.error && (
        <div className="latency-diagnostics__error">{clockReadiness.error}</div>
      )}

      <div className="latency-diagnostics__secondary-actions">
        <button
          type="button"
          title="Export latency session"
          aria-label="Export latency session"
          onClick={() => latencyDiagnostics.exportJsonl()}
        >
          <FiDownload aria-hidden="true" />
        </button>
        <button
          type="button"
          disabled={snapshot.downlink.active || snapshot.uplink.active}
          title="Reset latency session"
          aria-label="Reset latency session"
          onClick={() => latencyDiagnostics.resetSession()}
        >
          <FiRotateCcw aria-hidden="true" />
        </button>
      </div>
    </section>
  )
}

export default LatencyDiagnosticsPanel
