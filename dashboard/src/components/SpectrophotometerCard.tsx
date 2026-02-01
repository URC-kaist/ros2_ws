import { useCallback, useEffect, useMemo, useState } from 'react'
import { getRosBridgeClient } from '../lib/rosBridge'
import './SpectrophotometerCard.css'

type SpectrumMsg = {
  header?: { stamp?: { sec?: number; nanosec?: number }; frame_id?: string }
  wavelength_nm: number[]
  intensity: number[]
  mode: number
}

type GetSpectrumRequest = {
  use_absorbance: boolean
  publish_topic: boolean
}

type GetSpectrumResponse = {
  success: boolean
  message: string
  spectrum: SpectrumMsg
}

const SERVICE_NAME = '/get_spectrum'
const SERVICE_TYPE = 'mr2_spectrophotometer_msgs/srv/GetSpectrum'

const formatTimestamp = (timestamp: number | null) => {
  if (!timestamp) return '—'
  return new Date(timestamp).toLocaleTimeString()
}

const buildPath = (x: number[], y: number[]) => {
  if (x.length === 0 || y.length === 0 || x.length !== y.length) {
    return ''
  }
  const xMin = Math.min(...x)
  const xMax = Math.max(...x)
  const yMin = Math.min(...y)
  const yMax = Math.max(...y)
  const xSpan = xMax - xMin || 1
  const ySpan = yMax - yMin || 1

  const points = x.map((xVal, idx) => {
    const xNorm = ((xVal - xMin) / xSpan) * 100
    const yNorm = 100 - ((y[idx] - yMin) / ySpan) * 100
    return `${xNorm.toFixed(2)},${yNorm.toFixed(2)}`
  })
  return points.length > 0 ? `M ${points.join(' L ')}` : ''
}

const buildAreaPath = (linePath: string) => {
  if (!linePath) return ''
  return `${linePath} L 100,100 L 0,100 Z`
}

const SpectrophotometerCard = () => {
  const [rosConnected, setRosConnected] = useState(false)
  const [useAbsorbance, setUseAbsorbance] = useState(true)
  const [isLoading, setIsLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [spectrum, setSpectrum] = useState<SpectrumMsg | null>(null)
  const [lastUpdated, setLastUpdated] = useState<number | null>(null)

  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
    const offConnection = ros.onConnectionStatus(setRosConnected)
    return () => offConnection()
  }, [])

  const handleCapture = useCallback(async () => {
    const ros = getRosBridgeClient()
    ros.connect()
    setIsLoading(true)
    setError(null)

    if (!ros.isConnected()) {
      setIsLoading(false)
      setError('ROS bridge is offline')
      return
    }

    try {
      const response = await ros.callService<GetSpectrumRequest, GetSpectrumResponse>(
        SERVICE_NAME,
        SERVICE_TYPE,
        {
          use_absorbance: useAbsorbance,
          publish_topic: false,
        }
      )
      if (!response.success) {
        setError(response.message || 'Spectrometer returned an error')
        return
      }
      if (!response.spectrum || !response.spectrum.wavelength_nm?.length) {
        setError('No spectrum data received')
        return
      }
      setSpectrum(response.spectrum)
      setLastUpdated(Date.now())
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Service call failed'
      setError(message)
    } finally {
      setIsLoading(false)
    }
  }, [useAbsorbance])

  const handleExport = useCallback(() => {
    if (!spectrum) return
    const header = [
      `# mode: ${useAbsorbance ? 'absorbance' : 'transmittance'}`,
      `# captured_at: ${lastUpdated ? new Date(lastUpdated).toISOString() : 'unknown'}`,
      'wavelength_nm,intensity',
    ]
    const rows = spectrum.wavelength_nm.map((wl, idx) => {
      const y = spectrum.intensity[idx] ?? 0
      return `${wl},${y}`
    })
    const csv = [...header, ...rows].join('\n')
    const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' })
    const url = URL.createObjectURL(blob)
    const stamp = lastUpdated ? new Date(lastUpdated) : new Date()
    const timestamp = stamp.toISOString().replace(/[:.]/g, '-')
    const filename = `spectrogram_${useAbsorbance ? 'abs' : 'trans'}_${timestamp}.csv`
    const link = document.createElement('a')
    link.href = url
    link.download = filename
    link.click()
    URL.revokeObjectURL(url)
  }, [spectrum, useAbsorbance, lastUpdated])

  const plot = useMemo(() => {
    if (!spectrum) {
      return { linePath: '', areaPath: '', peak: null as null | { wl: number; intensity: number } }
    }
    const { wavelength_nm: wl, intensity } = spectrum
    const linePath = buildPath(wl, intensity)
    const areaPath = buildAreaPath(linePath)
    if (!wl.length || !intensity.length) {
      return { linePath, areaPath, peak: null }
    }
    let peakIdx = 0
    for (let i = 1; i < intensity.length; i += 1) {
      if (intensity[i] > intensity[peakIdx]) peakIdx = i
    }
    return {
      linePath,
      areaPath,
      peak: { wl: wl[peakIdx], intensity: intensity[peakIdx] },
    }
  }, [spectrum])

  const range = useMemo(() => {
    if (!spectrum || !spectrum.wavelength_nm.length) {
      return null
    }
    const min = Math.min(...spectrum.wavelength_nm)
    const max = Math.max(...spectrum.wavelength_nm)
    return { min, max }
  }, [spectrum])

  const modeLabel = useAbsorbance ? 'Absorbance' : 'Transmittance'

  return (
    <article className="card card--span-2 spectro-card">
      <header className="spectro-card__header">
        <div>
          <h3>Spectrophotometer</h3>
          <p>On-demand spectrum capture via ROS service.</p>
        </div>
        <div className="spectro-card__status">
          <span
            className={`spectro-card__dot ${rosConnected ? '' : 'spectro-card__dot--off'}`}
            aria-hidden="true"
          />
          <span>{rosConnected ? 'ROS online' : 'ROS offline'}</span>
        </div>
      </header>

      <div className="spectro-card__controls">
        <button
          className="spectro-card__button"
          type="button"
          onClick={handleCapture}
          disabled={!rosConnected || isLoading}
        >
          {isLoading ? 'Capturing…' : 'Capture spectrum'}
        </button>
        <button
          className="spectro-card__button spectro-card__button--secondary"
          type="button"
          onClick={handleExport}
          disabled={!spectrum}
        >
          Export CSV
        </button>
        <label className="spectro-card__toggle">
          <input
            type="checkbox"
            checked={useAbsorbance}
            onChange={(event) => setUseAbsorbance(event.target.checked)}
          />
          <span>{modeLabel}</span>
        </label>
      </div>

      <div className="spectro-card__plot" role="img" aria-label="Spectrum plot">
        {plot.linePath ? (
          <svg viewBox="0 0 100 100" preserveAspectRatio="none">
            <defs>
              <linearGradient id="spectro-line" x1="0" y1="0" x2="1" y2="0">
                <stop offset="0%" stopColor="#35d3c3" />
                <stop offset="100%" stopColor="#f7b24d" />
              </linearGradient>
              <linearGradient id="spectro-fill" x1="0" y1="0" x2="0" y2="1">
                <stop offset="0%" stopColor="rgba(53, 211, 195, 0.35)" />
                <stop offset="100%" stopColor="rgba(15, 23, 36, 0.0)" />
              </linearGradient>
            </defs>
            <g className="spectro-card__grid">
              {[20, 40, 60, 80].map((x) => (
                <line key={`x-${x}`} x1={x} y1="0" x2={x} y2="100" />
              ))}
              {[25, 50, 75].map((y) => (
                <line key={`y-${y}`} x1="0" y1={y} x2="100" y2={y} />
              ))}
            </g>
            <path className="spectro-card__area" d={plot.areaPath} />
            <path className="spectro-card__line" d={plot.linePath} />
          </svg>
        ) : (
          <div className="spectro-card__placeholder">
            {rosConnected ? 'No spectrum captured yet.' : 'Waiting for ROS connection…'}
          </div>
        )}
      </div>

      <div className="spectro-card__stats">
        <div>
          <span>Last capture</span>
          <strong>{formatTimestamp(lastUpdated)}</strong>
        </div>
        <div>
          <span>Mode</span>
          <strong>{modeLabel}</strong>
        </div>
        <div>
          <span>Range</span>
          <strong>
            {range ? `${range.min.toFixed(0)}–${range.max.toFixed(0)} nm` : '—'}
          </strong>
        </div>
        <div>
          <span>Peak</span>
          <strong>
            {plot.peak
              ? `${plot.peak.wl.toFixed(1)} nm`
              : '—'}
          </strong>
        </div>
      </div>

      {error && <div className="spectro-card__error">{error}</div>}
    </article>
  )
}

export default SpectrophotometerCard
