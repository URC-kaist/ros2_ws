import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'
import type {
  GetSpectrumRequest,
  GetSpectrumResponse,
  SpectrumMsg,
} from '../lib/rosMessages'
import './SpectrophotometerCard.css'

type UPlotInstance = { destroy: () => void; setData: (data: number[][]) => void }

type UPlotConstructor = new (
  options: UPlotOptions,
  data: number[][],
  target: HTMLElement
) => UPlotInstance

type UPlotOptions = {
  width: number
  height: number
  legend: { show: boolean }
  cursor: { show: boolean }
  scales: { x: { time: boolean } }
  padding: [number, number, number, number]
  axes: Array<{
    label: string
    stroke: string
    grid: { stroke: string }
    font: string
  }>
  series: Array<
    | Record<string, never>
    | { label: string; stroke: string; width: number; fill: string }
  >
}

const SERVICE_NAME = '/get_spectrum'
const SERVICE_TYPE = 'mr2_spectrophotometer_msgs/srv/GetSpectrum'

const formatTimestamp = (timestamp: number | null) => {
  if (!timestamp) return '—'
  return new Date(timestamp).toLocaleTimeString()
}

const makePlotOptions = (label: string, width: number, height: number): UPlotOptions => ({
  width,
  height,
  legend: { show: false },
  cursor: { show: true },
  scales: { x: { time: false } },
  padding: [10, 12, 6, 6],
  axes: [
    {
      label: 'Wavelength (nm)',
      stroke: 'rgba(232, 238, 245, 0.7)',
      grid: { stroke: 'rgba(255, 255, 255, 0.08)' },
      font: '12px "IBM Plex Sans", system-ui',
    },
    {
      label,
      stroke: 'rgba(232, 238, 245, 0.7)',
      grid: { stroke: 'rgba(255, 255, 255, 0.08)' },
      font: '12px "IBM Plex Sans", system-ui',
    },
  ],
  series: [
    {},
    {
      label,
      stroke: '#35d3c3',
      width: 1.2,
      fill: 'rgba(53, 211, 195, 0.18)',
    },
  ],
})

const SpectrophotometerCard = () => {
  const { ros, connected: rosConnected } = useRosBridge()
  const [useAbsorbance, setUseAbsorbance] = useState(true)
  const [isLoading, setIsLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [spectrum, setSpectrum] = useState<SpectrumMsg | null>(null)
  const [lastUpdated, setLastUpdated] = useState<number | null>(null)
  const [plotSize, setPlotSize] = useState({ width: 0, height: 0 })
  const [uplotCtor, setUplotCtor] = useState<UPlotConstructor | null>(null)
  const plotContainerRef = useRef<HTMLDivElement | null>(null)
  const plotInstanceRef = useRef<UPlotInstance | null>(null)
  const modeLabel = useAbsorbance ? 'Absorbance' : 'Transmittance'

  useEffect(() => {
    if (typeof window === 'undefined') return
    const ctor = (window as unknown as { uPlot?: UPlotConstructor }).uPlot
    if (!ctor) {
      setError('Plotting library not loaded')
      return
    }
    setUplotCtor(() => ctor)
  }, [])

  const handleCapture = useCallback(async () => {
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
  }, [ros, useAbsorbance])

  useEffect(() => {
    const container = plotContainerRef.current
    if (!container || typeof ResizeObserver === 'undefined') return

    const observer = new ResizeObserver((entries) => {
      const rect = entries[0]?.contentRect
      if (!rect) return
      const nextWidth = Math.max(1, Math.floor(rect.width))
      const nextHeight = Math.max(180, Math.floor(rect.height))
      setPlotSize((prev) =>
        prev.width === nextWidth && prev.height === nextHeight
          ? prev
          : { width: nextWidth, height: nextHeight }
      )
    })
    observer.observe(container)
    return () => observer.disconnect()
  }, [])

  useEffect(() => {
    const container = plotContainerRef.current
    if (!container || plotSize.width === 0 || plotSize.height === 0 || !uplotCtor) return

    plotInstanceRef.current?.destroy()
    const options = makePlotOptions(modeLabel, plotSize.width, plotSize.height)
    const initial: [number[], number[]] = [[], []]
    const plot = new uplotCtor(options, initial, container)
    plotInstanceRef.current = plot
    return () => plot.destroy()
  }, [plotSize, modeLabel, uplotCtor])

  useEffect(() => {
    const plot = plotInstanceRef.current
    if (!plot) return
    if (!spectrum) {
      plot.setData([[], []])
      return
    }
    plot.setData([spectrum.wavelength_nm, spectrum.intensity])
  }, [spectrum])

  const handleExport = useCallback(() => {
    if (!spectrum) return
    const frameId = spectrum.header?.frame_id ?? 'unknown'
    const calibrationPath =
      (spectrum as unknown as { calibration_path?: string }).calibration_path ??
      (import.meta.env.VITE_SPECTRO_CALIBRATION_PATH as string | undefined) ??
      'unknown'
    const header = [
      `# mode: ${useAbsorbance ? 'absorbance' : 'transmittance'}`,
      `# captured_at: ${lastUpdated ? new Date(lastUpdated).toISOString() : 'unknown'}`,
      `# frame_id: ${frameId}`,
      `# calibration_path: ${calibrationPath}`,
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

  const peak = useMemo(() => {
    if (!spectrum || !spectrum.intensity.length) return null
    const { wavelength_nm: wl, intensity } = spectrum
    let peakIdx = 0
    for (let i = 1; i < intensity.length; i += 1) {
      if (intensity[i] > intensity[peakIdx]) peakIdx = i
    }
    return { wl: wl[peakIdx], intensity: intensity[peakIdx] }
  }, [spectrum])

  const range = useMemo(() => {
    if (!spectrum || !spectrum.wavelength_nm.length) {
      return null
    }
    const min = Math.min(...spectrum.wavelength_nm)
    const max = Math.max(...spectrum.wavelength_nm)
    return { min, max }
  }, [spectrum])

  return (
    <article className="card spectro-card">
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
        <div className="spectro-card__plot-inner" ref={plotContainerRef} />
        {!spectrum && (
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
            {peak ? `${peak.wl.toFixed(1)} nm` : '—'}
          </strong>
        </div>
      </div>

      {error && <div className="spectro-card__error">{error}</div>}
    </article>
  )
}

export default SpectrophotometerCard
