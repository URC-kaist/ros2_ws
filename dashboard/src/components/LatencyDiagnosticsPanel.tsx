import { useEffect, useState } from 'react'
import { useLatencyDiagnostics } from '../hooks/useLatencyDiagnostics'
import { useRosBridge } from '../hooks/useRosBridge'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import { fetchVideoStreams, type VideoStreamInfo } from '../lib/videoGateway'
import type { DiagnosticArray } from '../lib/rosMessages'
import {
  chronyStatusForLog,
  evaluateChronyReadiness,
  fetchBaseChronyStatus,
  parseRoverChronyDiagnostic,
  type ChronyStatus,
} from '../lib/chronyStatus'
import {
  latencyDiagnostics,
  type LatencyStage,
  type RoverLatencyTrace,
} from '../lib/latencyDiagnostics'
import {
  parseRtpLatencyReport,
  type RtpLatencyReport,
} from '../lib/rtpLatencyReport'
import './ControlPanel/LatencyDiagnosticsPanel.css'

const STAGE_ROWS: Array<{ stage: LatencyStage; label: string }> = [
  { stage: 'input_t0', label: 'T0 Input' },
  { stage: 'gateway_command_received_t1', label: 'T1 Gateway' },
  { stage: 'rover_command_received_t3', label: 'T3 Rover RX' },
  { stage: 'rover_cmd_vel_published_t4', label: 'T4 cmd_vel' },
]

function formatMs(value: number | null | undefined) {
  return value == null || !Number.isFinite(value) ? '--' : `${value.toFixed(1)} ms`
}

function formatUsAsMs(value: number | null | undefined) {
  return value == null || !Number.isFinite(value) ? '--' : `${(value / 1000).toFixed(2)} ms`
}

function formatMbps(value: number | null | undefined) {
  return value == null || !Number.isFinite(value) ? '--' : `${(value / 1_000_000).toFixed(2)} Mbps`
}

const LatencyDiagnosticsPanel = () => {
  const snapshot = useLatencyDiagnostics()
  const { gateway } = useXbeeGateway()
  const { ros } = useRosBridge()
  const [streams, setStreams] = useState<VideoStreamInfo[]>([])
  const [rtpReport, setRtpReport] = useState<RtpLatencyReport | null>(null)
  const [rtpReportError, setRtpReportError] = useState<string | null>(null)
  const [baseChrony, setBaseChrony] = useState<ChronyStatus | null>(null)
  const [roverChrony, setRoverChrony] = useState<ChronyStatus | null>(null)
  const [chronyChecking, setChronyChecking] = useState(false)
  const [chronyError, setChronyError] = useState<string | null>(null)
  const [chronyNowMs, setChronyNowMs] = useState(Date.now())
  const [chronyCopyStatus, setChronyCopyStatus] = useState<string | null>(null)

  useEffect(() => gateway.onLatencyTrace((trace) => latencyDiagnostics.ingestGatewayTrace(trace)), [gateway])

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
          // Ignore malformed diagnostics without affecting control or video.
        }
      },
      { queueSize: 10 }
    )
  }, [ros])

  useEffect(() => {
    return ros.subscribe<DiagnosticArray>(
      '/system_status/clock',
      'diagnostic_msgs/DiagnosticArray',
      (message) => {
        try {
          setRoverChrony(parseRoverChronyDiagnostic(message))
        } catch (error) {
          setChronyError(
            error instanceof Error ? error.message : 'Invalid rover chrony status'
          )
        }
      },
      { throttleRate: 500, queueSize: 5 }
    )
  }, [ros])

  useEffect(() => {
    const timer = window.setInterval(() => setChronyNowMs(Date.now()), 1_000)
    return () => window.clearInterval(timer)
  }, [])

  useEffect(() => {
    let active = true
    void fetchVideoStreams()
      .then((nextStreams) => {
        if (!active) return
        setStreams(nextStreams)
      })
      .catch(() => undefined)
    void latencyDiagnostics.synchronizeGatewayClock()
    return () => {
      active = false
    }
  }, [])

  const trial = snapshot.currentTrial
  const t0 = trial?.stages.input_t0
  const clockOffsetUs = snapshot.clock.offsetUs
  const stageDelta = (stage: LatencyStage) => {
    const value = trial?.stages[stage]
    if (value == null || t0 == null) return null
    const isBrowserStage = stage === 'input_t0'
    const normalizedT0 = isBrowserStage || clockOffsetUs == null ? t0 : t0 + clockOffsetUs
    return (value - normalizedT0) / 1000
  }
  const reportStream =
    rtpReport?.streams.find((stream) => stream.stream_id === snapshot.selectedStreamId) ??
    rtpReport?.streams[0] ??
    null
  const chronyReadiness = evaluateChronyReadiness(baseChrony, roverChrony, chronyNowMs)

  const importRtpReport = async (file: File | undefined) => {
    if (!file) return
    try {
      const report = parseRtpLatencyReport(await file.text())
      setRtpReport(report)
      setRtpReportError(null)
      if (
        snapshot.selectedStreamId == null ||
        !report.streams.some((stream) => stream.stream_id === snapshot.selectedStreamId)
      ) {
        latencyDiagnostics.selectStream(report.streams[0].stream_id)
      }
    } catch (error) {
      setRtpReport(null)
      setRtpReportError(error instanceof Error ? error.message : 'Invalid RTP report')
    }
  }

  const checkChronySync = async () => {
    if (chronyChecking) return
    setChronyChecking(true)
    setChronyError(null)
    setChronyCopyStatus(null)
    try {
      setBaseChrony(await fetchBaseChronyStatus())
      setChronyNowMs(Date.now())
    } catch (error) {
      setBaseChrony(null)
      setChronyError(error instanceof Error ? error.message : 'Chrony status check failed')
    } finally {
      setChronyChecking(false)
    }
  }

  const copyZeroOffset = async () => {
    if (!chronyReadiness.ready) return
    try {
      await navigator.clipboard.writeText('--clock-offset-us 0')
      setChronyCopyStatus('Copied')
    } catch {
      setChronyCopyStatus('Copy failed')
    }
  }

  useEffect(() => {
    latencyDiagnostics.setChronyStatusSnapshot({
      ready: chronyReadiness.ready,
      reasons: chronyReadiness.reasons,
      rover_error_bound_s: chronyReadiness.roverErrorBoundS,
      base: chronyStatusForLog(baseChrony),
      rover: chronyStatusForLog(roverChrony),
    })
  }, [baseChrony, chronyReadiness.ready, chronyReadiness.reasons, chronyReadiness.roverErrorBoundS, roverChrony])

  return (
    <section className="latency-diagnostics" aria-label="Latency diagnostics">
      <div className="latency-diagnostics__header">
        <strong>Latency diagnostics</strong>
        <span>{snapshot.recordCount} events</span>
      </div>

      <label className="latency-diagnostics__field">
        <span>Video stream</span>
        <select
          value={snapshot.selectedStreamId ?? ''}
          onChange={(event) => latencyDiagnostics.selectStream(event.target.value || null)}
        >
          {!snapshot.selectedStreamId && (
            <option value="">{streams.length === 0 ? 'No streams' : 'Waiting for video'}</option>
          )}
          {streams.map((stream) => (
            <option key={stream.stream_id} value={stream.stream_id}>
              {stream.display.label || stream.stream_id}
            </option>
          ))}
        </select>
      </label>

      <div className="latency-diagnostics__actions">
        <button type="button" onClick={() => latencyDiagnostics.armTrial()}>
          {snapshot.armed
            ? snapshot.neutralSeen
              ? 'Send command'
              : 'Return neutral'
            : 'Arm command'}
        </button>
        <button
          type="button"
          onClick={() => void latencyDiagnostics.synchronizeGatewayClock()}
        >
          {snapshot.clock.syncing ? 'Syncing…' : 'Measure browser/base'}
        </button>
        <button type="button" onClick={() => latencyDiagnostics.exportJsonl()}>
          Export
        </button>
        <button type="button" onClick={() => latencyDiagnostics.resetSession()}>
          Reset
        </button>
        <button type="button" disabled={chronyChecking} onClick={() => void checkChronySync()}>
          {chronyChecking ? 'Checking chrony…' : 'Check chrony sync'}
        </button>
        <label className="latency-diagnostics__import">
          Import RTP report
          <input
            type="file"
            accept="application/json,.json"
            onChange={(event) => {
              void importRtpReport(event.target.files?.[0])
              event.target.value = ''
            }}
          />
        </label>
      </div>

      <div className="latency-diagnostics__clock">
        Base − browser {formatMs(snapshot.clock.offsetUs == null ? null : snapshot.clock.offsetUs / 1000)}
        {' · '}RTT {formatMs(snapshot.clock.rttUs == null ? null : snapshot.clock.rttUs / 1000)}
        {snapshot.clock.error && <span className="latency-diagnostics__error"> {snapshot.clock.error}</span>}
      </div>

      <div className="latency-diagnostics__section-title">Base/rover chrony</div>
      <div
        className={`latency-diagnostics__readiness latency-diagnostics__readiness--${chronyReadiness.ready ? 'ready' : 'blocked'}`}
      >
        {chronyReadiness.ready ? 'Ready for synchronized RTP capture' : 'Clock sync not ready'}
      </div>
      <div className="latency-diagnostics__summary">
        <div>
          <span>Base</span>
          <strong>{baseChrony?.synchronized ? 'Synchronized' : baseChrony ? 'Not ready' : '--'}</strong>
        </div>
        <div>
          <span>Base reference</span>
          <strong>{baseChrony?.referenceName ?? baseChrony?.referenceId ?? '--'}</strong>
        </div>
        <div>
          <span>Rover</span>
          <strong>{roverChrony?.synchronized ? 'Synchronized' : roverChrony ? 'Not ready' : '--'}</strong>
        </div>
        <div>
          <span>Rover reference</span>
          <strong>{roverChrony?.referenceName ?? roverChrony?.referenceId ?? '--'}</strong>
        </div>
        <div>
          <span>Rover residual</span>
          <strong>{formatUsAsMs(roverChrony?.systemTimeOffsetS == null ? null : roverChrony.systemTimeOffsetS * 1_000_000)}</strong>
        </div>
        <div>
          <span>Rover error bound</span>
          <strong>{formatUsAsMs(chronyReadiness.roverErrorBoundS == null ? null : chronyReadiness.roverErrorBoundS * 1_000_000)}</strong>
        </div>
        <div>
          <span>Stratum base/rover</span>
          <strong>{baseChrony?.stratum ?? '--'} / {roverChrony?.stratum ?? '--'}</strong>
        </div>
      </div>
      {chronyReadiness.ready ? (
        <button className="latency-diagnostics__copy" type="button" onClick={() => void copyZeroOffset()}>
          Copy --clock-offset-us 0{chronyCopyStatus ? ` · ${chronyCopyStatus}` : ''}
        </button>
      ) : (
        <div className="latency-diagnostics__reasons">
          {chronyReadiness.reasons.map((reason) => <small key={reason}>{reason}</small>)}
        </div>
      )}
      {chronyError && <div className="latency-diagnostics__error">{chronyError}</div>}
      <small>chronyd synchronizes continuously; this button only checks status and never steps a clock.</small>

      <div className="latency-diagnostics__section-title">Command path</div>
      <div className="latency-diagnostics__stages">
        {STAGE_ROWS.map(({ stage, label }) => (
          <div key={stage}>
            <span>{label}</span>
            <strong>{formatMs(stageDelta(stage))}</strong>
          </div>
        ))}
      </div>

      <div className="latency-diagnostics__summary">
        <div><span>Computer → gateway</span><strong>{formatMs(trial?.metrics.computerToGatewayMs)}</strong></div>
        <div><span>Computer → rover</span><strong>{formatMs(trial?.metrics.computerToRoverMs)}</strong></div>
        <div><span>Rover RX → publish</span><strong>{formatMs(trial?.metrics.roverPublishMs)}</strong></div>
      </div>

      <div className="latency-diagnostics__section-title">Rocket M2 RTP report</div>
      {rtpReportError && <div className="latency-diagnostics__error">{rtpReportError}</div>}
      {reportStream ? (
        <div className="latency-diagnostics__summary">
          <div><span>Report stream</span><strong>{reportStream.stream_id}</strong></div>
          <div><span>Matched packets</span><strong>{reportStream.matched_packets}</strong></div>
          <div><span>Packet loss</span><strong>{reportStream.loss_percent == null ? '--' : `${reportStream.loss_percent.toFixed(2)}%`}</strong></div>
          <div><span>Link latency p50</span><strong>{formatUsAsMs(reportStream.link_latency_us.p50)}</strong></div>
          <div><span>Link latency p95</span><strong>{formatUsAsMs(reportStream.link_latency_us.p95)}</strong></div>
          <div><span>Link latency p99</span><strong>{formatUsAsMs(reportStream.link_latency_us.p99)}</strong></div>
          <div><span>Link latency max</span><strong>{formatUsAsMs(reportStream.link_latency_us.max)}</strong></div>
          <div><span>Rover offered</span><strong>{formatMbps(reportStream.rover_offered_bitrate_bps)}</strong></div>
          <div><span>Base delivered</span><strong>{formatMbps(reportStream.base_delivered_bitrate_bps)}</strong></div>
          <div><span>Clock offset</span><strong>{formatUsAsMs(rtpReport?.clock_offset_us)}</strong></div>
          {rtpReport?.clock_offset_assumed && (
            <div className="latency-diagnostics__error">Clock offset was assumed to be zero.</div>
          )}
          {rtpReport?.warnings.map((warning) => (
            <div className="latency-diagnostics__error" key={warning}>{warning}</div>
          ))}
        </div>
      ) : (
        <small>Run the dual RTP capture analyzer, then import its JSON report.</small>
      )}

      <div className="latency-diagnostics__section-title">Browser live</div>
      <div className="latency-diagnostics__summary">
        <div><span>Frame samples</span><strong>{snapshot.videoTiming.sampleCount}</strong></div>
        <div><span>Base → browser p50</span><strong>{formatMs(snapshot.videoTiming.baseToBrowser.p50Ms)}</strong></div>
        <div><span>Base → browser p95</span><strong>{formatMs(snapshot.videoTiming.baseToBrowser.p95Ms)}</strong></div>
        <div><span>Decode/render p50</span><strong>{formatMs(snapshot.videoTiming.decodeRender.p50Ms)}</strong></div>
        <div><span>Decode/render p95</span><strong>{formatMs(snapshot.videoTiming.decodeRender.p95Ms)}</strong></div>
      </div>

      <small>Rocket latency comes only from matched rover/base RTP pcaps. Synchronize rover and base clocks.</small>
    </section>
  )
}

export default LatencyDiagnosticsPanel
