import { useEffect, useMemo, useState } from 'react'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import { type RocketM2Status } from '../lib/xbeeGateway'
import './RocketM2Card.css'

const STALE_MS = 15000
const TARGETS: Array<{ target: RocketM2Status['target']; label: string }> = [
  { target: 'base', label: 'Base' },
  { target: 'drone', label: 'Drone' },
  { target: 'rover', label: 'Rover' },
]

const formatDbm = (value: number | null | undefined, digits = 0) => {
  if (!Number.isFinite(value)) return '--'
  return `${(value as number).toFixed(digits)} dBm`
}

const formatNumber = (value: number | null | undefined, digits = 0) => {
  if (!Number.isFinite(value)) return '--'
  return (value as number).toFixed(digits)
}

const formatChain = (values: number[] | null | undefined, idx: number) => {
  if (!values || idx < 0 || idx >= values.length) return '--'
  const value = values[idx]
  return Number.isFinite(value) ? `${Math.round(value)}` : '--'
}

const RocketM2Card = () => {
  const { gateway } = useXbeeGateway()
  const [statuses, setStatuses] = useState<Record<string, RocketM2Status>>({})
  const [nowMs, setNowMs] = useState(() => Date.now())

  useEffect(() => {
    const unsubscribe = gateway.onRocketM2Status((payload) => {
      setStatuses((current) => ({ ...current, [payload.target]: payload }))
    })
    return () => {
      unsubscribe()
    }
  }, [gateway])

  useEffect(() => {
    const interval = window.setInterval(() => setNowMs(Date.now()), 1000)
    return () => window.clearInterval(interval)
  }, [])

  const rows = useMemo(() => TARGETS.map(({ target, label }) => {
    const status = statuses[target] ?? null
    const signal = typeof status?.signal === 'number' ? status.signal : null
    const noise = typeof status?.noisef === 'number' ? status.noisef : null
    const snr = signal != null && noise != null ? signal - noise : null
    const lastSuccess = status?.last_success_ms ?? null
    const isStale = lastSuccess == null || nowMs - lastSuccess > STALE_MS
    const isUp = !!status && status.connected && !isStale
    const hasSuccess = status?.last_success_ms != null
    const stateLabel = status
      ? isUp
        ? 'Up'
        : !hasSuccess
          ? 'Down'
          : isStale
            ? 'Stale'
            : status.connected
              ? 'Degraded'
              : 'Down'
      : 'Waiting'
    const stateClass = !status ? '' : isUp ? 'status-good' : 'status-warn'
    const updatedLabel = status?.updated_at_ms
      ? new Date(status.updated_at_ms).toLocaleTimeString()
      : null
    const errorText = status?.error && !status.connected ? status.error : null
    return {
      target,
      label: status?.label ?? label,
      status,
      signal,
      noise,
      snr,
      stateLabel,
      stateClass,
      updatedLabel,
      errorText,
    }
  }), [statuses, nowMs])

  return (
    <article className="card rocket-m2-card">
      <h3>Rocket M2 Links</h3>
      <p>Base, drone, and rover Rocket M2 management status.</p>
      <div className="rocket-m2-grid">
        {rows.map((row) => (
          <section className="rocket-m2-target" key={row.target}>
            <div className="rocket-m2-target__header">
              <strong>{row.label}</strong>
              <span className={row.stateClass}>{row.stateLabel}</span>
            </div>
            <div className="status-list">
              <div className="status-item">
                <span>Signal</span>
                <strong>{formatDbm(row.signal)}</strong>
              </div>
              <div className="status-item">
                <span>RSSI</span>
                <strong>{formatNumber(row.status?.rssi)}</strong>
              </div>
              <div className="status-item">
                <span>Noise</span>
                <strong>{formatDbm(row.noise)}</strong>
              </div>
              <div className="status-item">
                <span>SNR</span>
                <strong>{row.snr != null ? `${row.snr.toFixed(0)} dB` : '--'}</strong>
              </div>
              <div className="status-item">
                <span>Channel</span>
                <strong>
                  {Number.isFinite(row.status?.chwidth) ? `${row.status?.chwidth} MHz` : '--'}
                </strong>
              </div>
              <div className="status-item">
                <span>Chains</span>
                <strong>
                  {formatChain(row.status?.chainrssi, 0)} / {formatChain(row.status?.chainrssi, 1)}
                </strong>
              </div>
            </div>
            {row.errorText ? <div className="rocket-m2-error">Error: {row.errorText}</div> : null}
            {row.updatedLabel ? <div className="status-updated">Updated {row.updatedLabel}</div> : null}
          </section>
        ))}
      </div>
    </article>
  )
}

export default RocketM2Card
