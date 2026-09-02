import { useState, type CSSProperties } from 'react'
import './ControlVectorPlot.css'

export type CmdVel = {
  x: number
  y: number
  yaw: number
}

export type ControlMode = 'manual' | 'orbit'
export type SensitivityLevel = 'low' | 'med' | 'high'
export type OrbitCenter = { x: number; y: number }

type ControlVectorPlotProps = {
  cmdVel: CmdVel
  controlMode: ControlMode
  onSelectControlMode: (mode: ControlMode) => void
  modeSwitchProgress: number
  sensitivity: SensitivityLevel
  rotationSensitivity: SensitivityLevel
  centerSensitivity: SensitivityLevel
  orbitCenter: OrbitCenter
  orbitYaw: number
  requestedOrbitYaw: number
  safeOrbitYaw: number
  orbitNeedsNeutral: boolean
  isConnected: boolean
  gamepads: Array<{ index: number; id: string }>
  selectedGamepadIndex: number | null
  onSelectGamepad: (index: number | null) => void
  onSelectSensitivity: (level: SensitivityLevel) => void
  onSelectRotationSensitivity: (level: SensitivityLevel) => void
  onSelectCenterSensitivity: (level: SensitivityLevel) => void
  onSetOrbitCenter: (center: OrbitCenter) => void
}

const sensitivityScale = {
  low: 0.2,
  med: 0.4,
  high: 0.8,
} as const

const centerSensitivityScale = {
  low: 0.1,
  med: 0.25,
  high: 0.5,
} as const

const RADIUS = 0.57725 // meters
const ARC_CENTER = 30
const ORBIT_PLOT_CENTER = 120
const ORBIT_PLOT_RADIUS = 88

const polarPoint = (center: number, radius: number, degrees: number) => {
  const radians = (degrees * Math.PI) / 180
  return {
    x: center + radius * Math.cos(radians),
    y: center + radius * Math.sin(radians),
  }
}

const formatSigned = (value: number, digits = 1) =>
  `${value >= 0 ? '+' : ''}${value.toFixed(digits)}`

type GamepadPickerProps = Pick<
  ControlVectorPlotProps,
  'gamepads' | 'selectedGamepadIndex' | 'onSelectGamepad'
>

const GamepadPicker = ({
  gamepads,
  selectedGamepadIndex,
  onSelectGamepad,
}: GamepadPickerProps) => (
  <label className="gamepad-picker">
    <span className="gamepad-header">Gamepad</span>
    <select
      value={selectedGamepadIndex ?? ''}
      onChange={(event) =>
        onSelectGamepad(event.target.value === '' ? null : Number(event.target.value))
      }
    >
      <option value="">No control</option>
      {gamepads.length === 0 ? (
        <option value="" disabled>
          No gamepad detected
        </option>
      ) : (
        gamepads.map((pad) => (
          <option key={pad.index} value={pad.index}>
            {pad.id}
          </option>
        ))
      )}
    </select>
  </label>
)

type OrbitControlViewProps = Pick<
  ControlVectorPlotProps,
  | 'centerSensitivity'
  | 'isConnected'
  | 'onSelectCenterSensitivity'
  | 'onSelectRotationSensitivity'
  | 'onSetOrbitCenter'
  | 'orbitCenter'
  | 'orbitNeedsNeutral'
  | 'orbitYaw'
  | 'requestedOrbitYaw'
  | 'rotationSensitivity'
  | 'safeOrbitYaw'
>

const OrbitControlView = ({
  centerSensitivity,
  isConnected,
  onSelectCenterSensitivity,
  onSelectRotationSensitivity,
  onSetOrbitCenter,
  orbitCenter,
  orbitNeedsNeutral,
  orbitYaw,
  requestedOrbitYaw,
  rotationSensitivity,
  safeOrbitYaw,
}: OrbitControlViewProps) => {
  const [coordinateDraft, setCoordinateDraft] = useState<{
    axis: 'x' | 'y'
    value: string
  } | null>(null)
  const centerXInput =
    coordinateDraft?.axis === 'x' ? coordinateDraft.value : orbitCenter.x.toFixed(2)
  const centerYInput =
    coordinateDraft?.axis === 'y' ? coordinateDraft.value : orbitCenter.y.toFixed(2)

  const commitCoordinate = (axis: 'x' | 'y', rawValue: string) => {
    setCoordinateDraft(null)
    const value = Number(rawValue)
    if (rawValue.trim() === '' || !Number.isFinite(value)) {
      return
    }
    const next = { ...orbitCenter, [axis]: value }
    onSetOrbitCenter(next)
  }

  const centerExtent = Math.max(Math.abs(orbitCenter.x), Math.abs(orbitCenter.y), 0.75)
  const plotRanges = [1, 2, 5, 10, 20, 50, 100]
  const plotRange = plotRanges.find((range) => range >= centerExtent * 1.2) ?? centerExtent * 1.2
  const pixelsPerMeter = ORBIT_PLOT_RADIUS / plotRange
  const iccX = ORBIT_PLOT_CENTER + orbitCenter.x * pixelsPerMeter
  const iccY = ORBIT_PLOT_CENTER - orbitCenter.y * pixelsPerMeter
  const orbitRadius = Math.hypot(orbitCenter.x, orbitCenter.y)
  const centerLinearSpeed = Math.abs(orbitYaw) * orbitRadius
  const configuredYawMax = sensitivityScale[rotationSensitivity] / RADIUS
  const yawRatio = Math.min(Math.abs(orbitYaw) / configuredYawMax, 1)
  const yawSweepDegrees = orbitYaw === 0 ? 0 : 28 + yawRatio * 242
  const yawStartDegrees = -90
  const yawEndDegrees =
    orbitYaw >= 0 ? yawStartDegrees - yawSweepDegrees : yawStartDegrees + yawSweepDegrees
  const yawStartPoint = polarPoint(ORBIT_PLOT_CENTER, 35, yawStartDegrees)
  const yawEndPoint = polarPoint(ORBIT_PLOT_CENTER, 35, yawEndDegrees)
  const yawLargeArc = yawSweepDegrees > 180 ? 1 : 0
  const yawSweepFlag = orbitYaw >= 0 ? 0 : 1
  const yawPath =
    yawSweepDegrees === 0
      ? ''
      : `M ${yawStartPoint.x} ${yawStartPoint.y} A 35 35 0 ${yawLargeArc} ${yawSweepFlag} ${yawEndPoint.x} ${yawEndPoint.y}`
  const yawWasLimited = Math.abs(requestedOrbitYaw) > Math.abs(orbitYaw) + 1e-4
  const coordinateEditingDisabled = Math.abs(orbitYaw) > 1e-4
  const status = !isConnected
    ? 'No controller'
    : orbitNeedsNeutral
      ? 'Center left stick'
      : Math.abs(orbitYaw) > 1e-4
        ? 'Orbit active'
        : 'Ready'
  const direction = orbitYaw > 1e-4 ? 'CCW' : orbitYaw < -1e-4 ? 'CW' : '—'

  return (
    <>
      <div
        className={`orbit-plot ${!isConnected ? 'orbit-plot-disconnected' : ''}`}
        aria-label={`Orbit center X ${orbitCenter.x.toFixed(2)} meters, Y ${orbitCenter.y.toFixed(2)} meters`}
      >
        <svg viewBox="0 0 240 240" role="img" aria-label="Rover orbit center visualization">
          <defs>
            <marker
              id="orbit-arrow-head"
              viewBox="0 0 10 10"
              refX="8"
              refY="5"
              markerWidth="5"
              markerHeight="5"
              orient="auto"
            >
              <path d="M 0 0 L 10 5 L 0 10 z" className="orbit-arrow-head" />
            </marker>
          </defs>
          <line className="orbit-axis" x1="24" y1="120" x2="216" y2="120" />
          <line className="orbit-axis" x1="120" y1="24" x2="120" y2="216" />
          <path className="orbit-axis-arrow" d="M 116 31 L 120 24 L 124 31" />
          <path className="orbit-axis-arrow" d="M 209 116 L 216 120 L 209 124" />
          <text className="orbit-axis-label" x="126" y="31">
            +Y
          </text>
          <text className="orbit-axis-label" x="203" y="111">
            +X
          </text>
          <text className="orbit-scale-label" x="20" y="224">
            ±{Number(plotRange).toFixed(plotRange < 10 ? 1 : 0)} m
          </text>

          {orbitRadius > 0.005 && (
            <line
              className="orbit-radius-line"
              x1={ORBIT_PLOT_CENTER}
              y1={ORBIT_PLOT_CENTER}
              x2={iccX}
              y2={iccY}
            />
          )}

          <g className="rover-symbol" aria-hidden="true">
            <path d="M 120 90 L 136 103 L 136 146 L 104 146 L 104 103 Z" />
            <line x1="120" y1="96" x2="120" y2="139" />
            <rect x="98" y="103" width="6" height="15" rx="2" />
            <rect x="136" y="103" width="6" height="15" rx="2" />
            <rect x="98" y="132" width="6" height="15" rx="2" />
            <rect x="136" y="132" width="6" height="15" rx="2" />
          </g>

          <g className="orbit-center-marker" transform={`translate(${iccX} ${iccY})`}>
            <circle className="orbit-center-halo" r="9" />
            <circle className="orbit-center-point" r="3.5" />
            <line x1="-13" y1="0" x2="-7" y2="0" />
            <line x1="7" y1="0" x2="13" y2="0" />
            <line x1="0" y1="-13" x2="0" y2="-7" />
            <line x1="0" y1="7" x2="0" y2="13" />
          </g>

          <circle className="orbit-yaw-track" cx="120" cy="120" r="35" />
          {yawPath && (
            <path
              className="orbit-yaw-arrow"
              d={yawPath}
              markerEnd="url(#orbit-arrow-head)"
            />
          )}
        </svg>
        <span className={`orbit-state ${Math.abs(orbitYaw) > 1e-4 ? 'active' : ''}`}>
          {orbitRadius <= 0.005 ? 'PIVOT' : 'ORBIT'} · {status}
        </span>
      </div>

      <div className="orbit-coordinate-editor">
        <div className="orbit-coordinate-title">
          <span>Rotation center</span>
          <button
            className="orbit-reset-button"
            type="button"
            disabled={coordinateEditingDisabled}
            onClick={() => onSetOrbitCenter({ x: 0, y: 0 })}
          >
            Reset
          </button>
        </div>
        <div className="orbit-coordinate-fields">
          <label>
            <span>X</span>
            <input
              type="number"
              inputMode="decimal"
              step="0.05"
              value={centerXInput}
              disabled={coordinateEditingDisabled}
              onFocus={() => {
                setCoordinateDraft({ axis: 'x', value: centerXInput })
              }}
              onChange={(event) =>
                setCoordinateDraft({ axis: 'x', value: event.target.value })
              }
              onBlur={(event) => commitCoordinate('x', event.target.value)}
              onKeyDown={(event) => {
                if (event.key === 'Enter') event.currentTarget.blur()
              }}
              aria-label="Orbit center X in meters, positive to rover right"
            />
            <span>m</span>
          </label>
          <label>
            <span>Y</span>
            <input
              type="number"
              inputMode="decimal"
              step="0.05"
              value={centerYInput}
              disabled={coordinateEditingDisabled}
              onFocus={() => {
                setCoordinateDraft({ axis: 'y', value: centerYInput })
              }}
              onChange={(event) =>
                setCoordinateDraft({ axis: 'y', value: event.target.value })
              }
              onBlur={(event) => commitCoordinate('y', event.target.value)}
              onKeyDown={(event) => {
                if (event.key === 'Enter') event.currentTarget.blur()
              }}
              aria-label="Orbit center Y in meters, positive to rover front"
            />
            <span>m</span>
          </label>
        </div>
      </div>

      <div className="orbit-metrics">
        <div>
          <span>Angular speed</span>
          <strong>{formatSigned(orbitYaw, 2)} rad/s</strong>
        </div>
        <div>
          <span>Direction</span>
          <strong>{direction}</strong>
        </div>
        <div>
          <span>Radius</span>
          <strong>{orbitRadius.toFixed(2)} m</strong>
        </div>
        <div>
          <span>Rover speed</span>
          <strong>{centerLinearSpeed.toFixed(2)} m/s</strong>
        </div>
      </div>

      {yawWasLimited && (
        <div className="orbit-limit-note">
          Wheel-speed limit · {Math.abs(requestedOrbitYaw).toFixed(2)} →{' '}
          {safeOrbitYaw.toFixed(2)} rad/s
        </div>
      )}

      <div className="orbit-sensitivity-grid">
        <label className="gamepad-picker">
          <span className="gamepad-header">Rotation sensitivity</span>
          <select
            value={rotationSensitivity}
            onChange={(event) =>
              onSelectRotationSensitivity(event.target.value as SensitivityLevel)
            }
          >
            {(['low', 'med', 'high'] as const).map((level) => (
              <option key={level} value={level}>
                {level.toUpperCase()} {(sensitivityScale[level] / RADIUS).toFixed(2)} rad/s
              </option>
            ))}
          </select>
        </label>
        <label className="gamepad-picker">
          <span className="gamepad-header">Center sensitivity</span>
          <select
            value={centerSensitivity}
            onChange={(event) =>
              onSelectCenterSensitivity(event.target.value as SensitivityLevel)
            }
          >
            {(['low', 'med', 'high'] as const).map((level) => (
              <option key={level} value={level}>
                {level.toUpperCase()} {centerSensitivityScale[level].toFixed(2)} m/s
              </option>
            ))}
          </select>
        </label>
      </div>
    </>
  )
}

const ControlVectorPlot = ({
  cmdVel,
  controlMode,
  onSelectControlMode,
  modeSwitchProgress,
  sensitivity,
  rotationSensitivity,
  centerSensitivity,
  orbitCenter,
  orbitYaw,
  requestedOrbitYaw,
  safeOrbitYaw,
  orbitNeedsNeutral,
  isConnected,
  gamepads,
  selectedGamepadIndex,
  onSelectGamepad,
  onSelectSensitivity,
  onSelectRotationSensitivity,
  onSelectCenterSensitivity,
  onSetOrbitCenter,
}: ControlVectorPlotProps) => {
  const axisRange = sensitivityScale[sensitivity]
  const yawRange = axisRange / RADIUS
  const displayX = -cmdVel.x
  const displayY = cmdVel.y
  const displayYaw = -cmdVel.yaw
  const magnitude = Math.hypot(displayX, displayY)
  const arrowScale = Math.min(1, magnitude / axisRange)
  const arrowAngle = Math.atan2(-displayY, displayX) * (180 / Math.PI)
  const yawMagnitude = Math.min(1, Math.abs(displayYaw) / yawRange)
  const yawSweep = yawMagnitude * 0.75
  const yawRadius = 26
  const yawAngle = yawSweep * 360
  const yawStartAngle = -90
  const yawEndAngle = yawStartAngle + (displayYaw >= 0 ? yawAngle : -yawAngle)
  const yawLargeArc = yawAngle > 180 ? 1 : 0
  const yawSweepFlag = displayYaw >= 0 ? 1 : 0
  const yawStartPoint = polarPoint(ARC_CENTER, yawRadius, yawStartAngle)
  const yawEndPoint = polarPoint(ARC_CENTER, yawRadius, yawEndAngle)
  const yawPath =
    yawAngle === 0
      ? ''
      : `M ${yawStartPoint.x} ${yawStartPoint.y} A ${yawRadius} ${yawRadius} 0 ${yawLargeArc} ${yawSweepFlag} ${yawEndPoint.x} ${yawEndPoint.y}`
  const modeProgressStyle = {
    '--mode-progress': `${Math.round(modeSwitchProgress * 100)}%`,
  } as CSSProperties

  return (
    <section className="panel-section control-command-panel">
      <div className="control-visual">
        <div
          className="control-mode-switch"
          role="group"
          aria-label="Drive control mode"
          style={modeProgressStyle}
        >
          <button
            type="button"
            className={controlMode === 'manual' ? 'active' : ''}
            aria-pressed={controlMode === 'manual'}
            onClick={() => onSelectControlMode('manual')}
          >
            Manual
          </button>
          <button
            type="button"
            className={controlMode === 'orbit' ? 'active' : ''}
            aria-pressed={controlMode === 'orbit'}
            onClick={() => onSelectControlMode('orbit')}
          >
            Orbit
          </button>
          <span className="control-mode-progress" aria-hidden="true" />
        </div>

        {modeSwitchProgress > 0 && modeSwitchProgress < 1 && (
          <div className="mode-switch-hint">
            Hold L1 + R1 · {Math.ceil((1 - modeSwitchProgress) * 2)} s
          </div>
        )}

        <div className="gamepad-row">
          <GamepadPicker
            gamepads={gamepads}
            selectedGamepadIndex={selectedGamepadIndex}
            onSelectGamepad={onSelectGamepad}
          />
          {controlMode === 'manual' && (
            <label className="gamepad-picker">
              <span className="gamepad-header">Sensitivity</span>
              <select
                value={sensitivity}
                onChange={(event) =>
                  onSelectSensitivity(event.target.value as SensitivityLevel)
                }
              >
                {(['low', 'med', 'high'] as const).map((level) => (
                  <option key={level} value={level}>
                    {level.toUpperCase()} {sensitivityScale[level].toFixed(1)}
                  </option>
                ))}
              </select>
            </label>
          )}
        </div>

        {controlMode === 'manual' ? (
          <>
            <div className={`vector-plot ${isConnected ? '' : 'vector-plot-disconnected'}`}>
              <div className="axis-markers">
                <span className="axis-label top">+{axisRange.toFixed(1)}</span>
                <span className="axis-label bottom">-{axisRange.toFixed(1)}</span>
                <span className="axis-label left">+{axisRange.toFixed(1)}</span>
                <span className="axis-label right">-{axisRange.toFixed(1)}</span>
              </div>
              <div className="angvel-overlay">
                <svg viewBox="0 0 60 60" className="angvel-arc">
                  <circle
                    className="angvel-arc-track"
                    cx={ARC_CENTER}
                    cy={ARC_CENTER}
                    r={yawRadius}
                  />
                  {yawPath ? <path className="angvel-arc-sweep" d={yawPath} /> : null}
                </svg>
              </div>
              <div
                className="vector-arrow"
                style={{
                  transform: `translateY(-50%) rotate(${arrowAngle}deg) scaleX(${arrowScale})`,
                }}
              />
              <div className="center-dot" />
            </div>
            <div className="vector-labels">
              {isConnected ? (
                <>
                  <span className="vector-value">x {formatSigned(cmdVel.y)} m/s</span>
                  <span className="vector-value">y {formatSigned(cmdVel.x)} m/s</span>
                  <span className="vector-value">ω {formatSigned(cmdVel.yaw)} rad/s</span>
                </>
              ) : (
                <span className="vector-value vector-value-muted">no gamepad connected</span>
              )}
            </div>
          </>
        ) : (
          <OrbitControlView
            centerSensitivity={centerSensitivity}
            isConnected={isConnected}
            onSelectCenterSensitivity={onSelectCenterSensitivity}
            onSelectRotationSensitivity={onSelectRotationSensitivity}
            onSetOrbitCenter={onSetOrbitCenter}
            orbitCenter={orbitCenter}
            orbitNeedsNeutral={orbitNeedsNeutral}
            orbitYaw={orbitYaw}
            requestedOrbitYaw={requestedOrbitYaw}
            rotationSensitivity={rotationSensitivity}
            safeOrbitYaw={safeOrbitYaw}
          />
        )}
      </div>
    </section>
  )
}

export default ControlVectorPlot
