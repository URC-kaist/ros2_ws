import { useEffect, useRef, useState } from 'react'
import { getSikGatewayClient, type CanEstopResponse } from '../../lib/sikGateway'
import './ControlEstopSection.css'

const E_STOP_TIMEOUT_MS = 4000

const ControlEstopSection = () => {
  const gateway = getSikGatewayClient()
  const timeoutsRef = useRef<Map<number, number>>(new Map())
  const [pending, setPending] = useState<{ estop: number | null; clear: number | null }>({
    estop: null,
    clear: null,
  })
  const [lastResult, setLastResult] = useState<CanEstopResponse | null>(null)

  const clearTimeoutFor = (requestId: number) => {
    const existing = timeoutsRef.current.get(requestId)
    if (existing != null) {
      window.clearTimeout(existing)
      timeoutsRef.current.delete(requestId)
    }
  }

  const scheduleTimeout = (requestId: number, enabled: boolean) => {
    clearTimeoutFor(requestId)
    const timeoutId = window.setTimeout(() => {
      setPending((prev) => {
        if (prev.estop === requestId) {
          return { ...prev, estop: null }
        }
        if (prev.clear === requestId) {
          return { ...prev, clear: null }
        }
        return prev
      })
      setLastResult({
        request_id: requestId,
        enabled,
        success: false,
      })
      timeoutsRef.current.delete(requestId)
    }, E_STOP_TIMEOUT_MS)
    timeoutsRef.current.set(requestId, timeoutId)
  }

  useEffect(() => {
    gateway.connect()
    const off = gateway.onCanEstop((resp) => {
      clearTimeoutFor(resp.request_id)
      setLastResult(resp)
      setPending((prev) => {
        let next = prev
        if (prev.estop === resp.request_id) {
          next = { ...next, estop: null }
        }
        if (prev.clear === resp.request_id) {
          next = { ...next, clear: null }
        }
        return next
      })
    })
    return () => {
      off()
      for (const timeoutId of timeoutsRef.current.values()) {
        window.clearTimeout(timeoutId)
      }
      timeoutsRef.current.clear()
    }
  }, [gateway])

  const handleEstop = () => {
    gateway.sendMissionControl({
      command: 3,
      clear_costmap: true,
      mission_id: 0,
    })
    const requestId = gateway.sendCanEstop(true)
    setPending((prev) => ({ ...prev, estop: requestId }))
    scheduleTimeout(requestId, true)
  }

  const handleClear = () => {
    const requestId = gateway.sendCanEstop(false)
    setPending((prev) => ({ ...prev, clear: requestId }))
    scheduleTimeout(requestId, false)
  }

  const resultLabel = lastResult ? (lastResult.success ? 'OK' : 'FAILED') : 'No response yet'

  const resultClass = lastResult
    ? lastResult.success
      ? 'estop-result estop-result--ok'
      : 'estop-result estop-result--fail'
    : 'estop-result'

  return (
    <section className="panel-section estop-section">
      <div className={resultClass}>{resultLabel}</div>
      <div className="estop-actions">
        <button
          className={`btn danger estop ${pending.estop != null ? 'is-pressed' : ''}`}
          type="button"
          onClick={handleEstop}
        >
          E-Stop
        </button>
        <button
          className={`btn estop estop-clear ${pending.clear != null ? 'is-pressed' : ''}`}
          type="button"
          onClick={handleClear}
        >
          Clear E-Stop
        </button>
      </div>
    </section>
  )
}

export default ControlEstopSection
