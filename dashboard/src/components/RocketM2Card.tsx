import { useEffect, useMemo, useState } from 'react'
import { getSikGatewayClient, type RocketM2Status } from '../lib/sikGateway'
import './RocketM2Card.css'

const STALE_MS = 15000

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
  const [status, setStatus] = useState<RocketM2Status | null>(null)
  const [nowMs, setNowMs] = useState(() => Date.now())

  useEffect(() => {
    const gateway = getSikGatewayClient()
    gateway.connect()
    const unsubscribe = gateway.onRocketM2Status((payload) => {
      setStatus(payload)
    })
    return () => {
      unsubscribe()
    }
  }, [])

  useEffect(() => {
    const interval = window.setInterval(() => setNowMs(Date.now()), 1000)
    return () => window.clearInterval(interval)
  }, [])

  const meta = useMemo(() => {
    const signal = typeof status?.signal === 'number' ? status.signal : null
    const noise = typeof status?.noisef === 'number' ? status.noisef : null
    const snr = signal != null && noise != null ? signal - noise : null
    const lastSuccess = status?.last_success_ms ?? null
    const isStale = lastSuccess == null || nowMs - lastSuccess > STALE_MS
    const isUp = !!status && status.connected && !isStale
    return {
      signal,
      noise,
      snr,
      isStale,
      isUp,
    }
  }, [status, nowMs])

  const hasSuccess = status?.last_success_ms != null
  const stateLabel = status
    ? meta.isUp
      ? 'Up'
      : !hasSuccess
        ? 'Down'
        : meta.isStale
          ? 'Stale'
          : status.connected
            ? 'Degraded'
            : 'Down'
    : 'Waiting'
  const stateClass = !status ? '' : meta.isUp ? 'status-good' : 'status-warn'
  const updatedLabel = status?.updated_at_ms
    ? new Date(status.updated_at_ms).toLocaleTimeString()
    : null
  const errorText = status?.error && !status.connected ? status.error : null

  return (
    <article className="card rocket-m2-card">
      <h3>Rocket M2 LAN</h3>
      <p>Rover LAN link status from Rocket M2.</p>
      <div className="status-list">
        <div className="status-item">
          <span>Status</span>
          <strong className={stateClass}>{stateLabel}</strong>
        </div>
        <div className="status-item">
          <span>Signal</span>
          <strong>{formatDbm(meta.signal)}</strong>
        </div>
        <div className="status-item">
          <span>RSSI</span>
          <strong>{formatNumber(status?.rssi)}</strong>
        </div>
        <div className="status-item">
          <span>Noise floor</span>
          <strong>{formatDbm(meta.noise)}</strong>
        </div>
        <div className="status-item">
          <span>SNR</span>
          <strong>{meta.snr != null ? `${meta.snr.toFixed(0)} dB` : '--'}</strong>
        </div>
        <div className="status-item">
          <span>Channel</span>
          <strong>
            {Number.isFinite(status?.chwidth) ? `${status?.chwidth} MHz` : '--'}
          </strong>
        </div>
        <div className="status-item">
          <span>Chain 1</span>
          <strong>{formatChain(status?.chainrssi, 0)}</strong>
        </div>
        <div className="status-item">
          <span>Chain 2</span>
          <strong>{formatChain(status?.chainrssi, 1)}</strong>
        </div>
      </div>
      {errorText ? <div className="rocket-m2-error">Error: {errorText}</div> : null}
      {updatedLabel ? <div className="status-updated">Updated {updatedLabel}</div> : null}
    </article>
  )
}

export default RocketM2Card
