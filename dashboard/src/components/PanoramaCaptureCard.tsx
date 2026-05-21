import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'
import { sendRosbridgeActionGoal, type ActiveActionGoal } from '../lib/rosActionBridge'
import type {
  PanoramaCaptureFeedback,
  PanoramaCaptureGoal,
  PanoramaCaptureResult,
} from '../lib/rosMessages'

const ACTION_NAME = '/panorama_capture'
const ACTION_TYPE = 'mr2_action_interface/action/PanoramaCapture'

const DEFAULT_GOAL: PanoramaCaptureGoal = {
  angle_step_deg: 30,
  yaw_tolerance_deg: 5,
  timeout_sec: 180,
  jpeg_quality: 90,
}

type CaptureState = 'idle' | 'running' | 'canceling' | 'succeeded' | 'failed'

const clamp = (value: number, min: number, max: number) =>
  Math.min(max, Math.max(min, value))

const toFiniteNumber = (value: string, fallback: number) => {
  const parsed = Number(value)
  return Number.isFinite(parsed) ? parsed : fallback
}

const formatDegrees = (value?: number) =>
  Number.isFinite(value) ? `${value?.toFixed(1)} deg` : '--'

const bytesToBlobUrl = (data?: string | number[], format = 'jpeg') => {
  if (!data) return null
  const mime = format.toLowerCase().includes('png') ? 'image/png' : 'image/jpeg'
  const bytes =
    typeof data === 'string'
      ? Uint8Array.from(atob(data), (char) => char.charCodeAt(0))
      : Uint8Array.from(data)
  return URL.createObjectURL(new Blob([bytes], { type: mime }))
}

const PanoramaCaptureCard = () => {
  const { connected: rosConnected } = useRosBridge()
  const [goal, setGoal] = useState<PanoramaCaptureGoal>(DEFAULT_GOAL)
  const [captureState, setCaptureState] = useState<CaptureState>('idle')
  const [feedback, setFeedback] = useState<PanoramaCaptureFeedback | null>(null)
  const [result, setResult] = useState<PanoramaCaptureResult | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [imageUrl, setImageUrl] = useState<string | null>(null)
  const activeGoalRef = useRef<ActiveActionGoal | null>(null)

  const progress = useMemo(() => {
    const captures = feedback?.captures_taken ?? result?.captures_taken ?? 0
    const expected = feedback?.expected_captures ?? result?.expected_captures ?? 0
    const value = expected > 0 ? clamp(captures / expected, 0, 1) : 0
    return { captures, expected, value }
  }, [feedback, result])

  const isRunning = captureState === 'running' || captureState === 'canceling'

  const updateGoal = useCallback(
    (field: keyof PanoramaCaptureGoal, rawValue: string) => {
      setGoal((current) => {
        const fallback = current[field]
        const value = toFiniteNumber(rawValue, fallback)
        const next = { ...current }
        if (field === 'angle_step_deg') next[field] = clamp(value, 1, 180)
        if (field === 'yaw_tolerance_deg') next[field] = clamp(value, 1, 90)
        if (field === 'timeout_sec') next[field] = Math.max(1, value)
        if (field === 'jpeg_quality') next[field] = Math.round(clamp(value, 1, 100))
        return next
      })
    },
    []
  )

  const clearImageUrl = useCallback(() => {
    setImageUrl((current) => {
      if (current) URL.revokeObjectURL(current)
      return null
    })
  }, [])

  const startCapture = useCallback(() => {
    if (!rosConnected || isRunning) return
    activeGoalRef.current?.dispose()
    clearImageUrl()
    setFeedback(null)
    setResult(null)
    setError(null)
    setCaptureState('running')

    activeGoalRef.current = sendRosbridgeActionGoal<
      PanoramaCaptureGoal,
      PanoramaCaptureFeedback,
      PanoramaCaptureResult
    >({
      action: ACTION_NAME,
      actionType: ACTION_TYPE,
      goal,
      onFeedback: setFeedback,
      onError: (message) => {
        setError(message)
        setCaptureState((current) => (current === 'succeeded' ? current : 'failed'))
      },
      onResult: (nextResult, _status, successful) => {
        setResult(nextResult)
        setCaptureState(successful && nextResult.success !== false ? 'succeeded' : 'failed')
        if (!successful || nextResult.success === false) {
          setError(nextResult.message || 'Panorama capture failed')
          return
        }
        const nextUrl = bytesToBlobUrl(nextResult.panorama?.data, nextResult.panorama?.format)
        if (!nextUrl) {
          setError('Panorama result did not include image data')
          return
        }
        setImageUrl(nextUrl)
      },
    })
  }, [clearImageUrl, goal, isRunning, rosConnected])

  const cancelCapture = useCallback(() => {
    if (!activeGoalRef.current || captureState !== 'running') return
    setCaptureState('canceling')
    activeGoalRef.current.cancel()
  }, [captureState])

  const downloadPanorama = useCallback(() => {
    if (!imageUrl) return
    const link = document.createElement('a')
    const timestamp = new Date().toISOString().replace(/[:.]/g, '-')
    link.href = imageUrl
    link.download = `panorama_${timestamp}.jpg`
    link.click()
  }, [imageUrl])

  useEffect(() => {
    return () => {
      activeGoalRef.current?.dispose()
      clearImageUrl()
    }
  }, [clearImageUrl])

  const statusLabel =
    captureState === 'running'
      ? 'capturing'
      : captureState === 'canceling'
        ? 'canceling'
        : captureState === 'succeeded'
          ? 'complete'
          : captureState === 'failed'
            ? 'error'
            : 'standby'

  return (
    <article className="card panorama-card">
      <header className="panorama-card__header">
        <div>
          <h3>Panorama Capture</h3>
          <p>Manual rover rotation capture through the panorama action server.</p>
        </div>
        <span className={`pill ${rosConnected ? 'pill--on' : 'pill--off'}`}>
          {rosConnected ? statusLabel : 'ROS offline'}
        </span>
      </header>

      <div className="panorama-card__settings">
        <label className="panorama-card__field">
          <span>Step (deg)</span>
          <input
            type="number"
            min={1}
            max={180}
            step={1}
            value={goal.angle_step_deg}
            disabled={isRunning}
            onChange={(event) => updateGoal('angle_step_deg', event.target.value)}
          />
        </label>
        <label className="panorama-card__field">
          <span>Tolerance (deg)</span>
          <input
            type="number"
            min={1}
            max={90}
            step={0.5}
            value={goal.yaw_tolerance_deg}
            disabled={isRunning}
            onChange={(event) => updateGoal('yaw_tolerance_deg', event.target.value)}
          />
        </label>
        <label className="panorama-card__field">
          <span>Timeout (sec)</span>
          <input
            type="number"
            min={1}
            step={5}
            value={goal.timeout_sec}
            disabled={isRunning}
            onChange={(event) => updateGoal('timeout_sec', event.target.value)}
          />
        </label>
        <label className="panorama-card__field">
          <span>JPEG quality</span>
          <input
            type="number"
            min={1}
            max={100}
            step={1}
            value={goal.jpeg_quality}
            disabled={isRunning}
            onChange={(event) => updateGoal('jpeg_quality', event.target.value)}
          />
        </label>
      </div>

      <div className="panorama-card__actions">
        <button
          className="spectro-card__button"
          type="button"
          disabled={!rosConnected || isRunning}
          onClick={startCapture}
        >
          Start capture
        </button>
        <button
          className="spectro-card__button spectro-card__button--secondary"
          type="button"
          disabled={captureState !== 'running'}
          onClick={cancelCapture}
        >
          Cancel
        </button>
        <button
          className="spectro-card__button spectro-card__button--secondary"
          type="button"
          disabled={!imageUrl}
          onClick={downloadPanorama}
        >
          Download JPEG
        </button>
      </div>

      <div className="panorama-card__preview">
        {imageUrl ? (
          <img src={imageUrl} alt="Stitched panorama result" />
        ) : (
          <div className="video-feed-placeholder panorama-card__placeholder">
            {isRunning ? 'Capturing panorama...' : 'No panorama captured yet.'}
          </div>
        )}
      </div>

      <div className="panorama-card__progress">
        <div className="panorama-card__progress-meta">
          <span>
            Captures {progress.captures} / {progress.expected || '--'}
          </span>
          <span>{Math.round(progress.value * 100)}%</span>
        </div>
        <progress value={progress.value} max={1} aria-label="Panorama capture progress" />
      </div>

      <div className="panorama-card__stats">
        <div>
          <span>Current yaw</span>
          <strong>{formatDegrees(feedback?.current_yaw_deg)}</strong>
        </div>
        <div>
          <span>Next target</span>
          <strong>{formatDegrees(feedback?.next_target_yaw_deg)}</strong>
        </div>
        <div>
          <span>Result</span>
          <strong>{result?.message || '--'}</strong>
        </div>
      </div>

      {error && <div className="spectro-card__error">{error}</div>}
    </article>
  )
}

export default PanoramaCaptureCard
