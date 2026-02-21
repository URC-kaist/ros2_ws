import './ControlVectorPlot.css'

export type CmdVel = {
  x: number
  y: number
  yaw: number
}

type ControlVectorPlotProps = {
  cmdVel: CmdVel
  sensitivity: 'low' | 'med' | 'high'
  isConnected: boolean
  gamepads: Array<{ index: number; id: string }>
  selectedGamepadIndex: number | null
  onSelectGamepad: (index: number | null) => void
  onSelectSensitivity: (level: 'low' | 'med' | 'high') => void
}

const sensitivityScale = {
  low: 0.4,
  med: 0.8,
  high: 1.2,
} as const

const RADIUS = 0.57725 // meters

const sensitivityValues = {
  low: 0.4,
  med: 0.8,
  high: 1.2,
} as const

const ARC_CENTER = 30

const polarPoint = (radius: number, degrees: number) => {
  const radians = (degrees * Math.PI) / 180
  return {
    x: ARC_CENTER + radius * Math.cos(radians),
    y: ARC_CENTER + radius * Math.sin(radians),
  }
}

const ControlVectorPlot = ({
  cmdVel,
  sensitivity,
  isConnected,
  gamepads,
  selectedGamepadIndex,
  onSelectGamepad,
  onSelectSensitivity,
}: ControlVectorPlotProps) => {
  const axisRange = 1 * sensitivityScale[sensitivity]
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
  const yawStartPoint = polarPoint(yawRadius, yawStartAngle)
  const yawEndPoint = polarPoint(yawRadius, yawEndAngle)
  const yawPath =
    yawAngle === 0
      ? ''
      : `M ${yawStartPoint.x} ${yawStartPoint.y} A ${yawRadius} ${yawRadius} 0 ${yawLargeArc} ${yawSweepFlag} ${yawEndPoint.x} ${yawEndPoint.y}`
  const formatSigned = (value: number) => (value >= 0 ? ` ${value.toFixed(1)}` : value.toFixed(1))

  return (
    <section className="panel-section">
      <div className="control-visual">
        <div className="gamepad-row">
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
          <label className="gamepad-picker">
            <span className="gamepad-header">Sensitivity</span>
            <select
              value={sensitivity}
              onChange={(event) =>
                onSelectSensitivity(event.target.value as 'low' | 'med' | 'high')
              }
            >
              {(['low', 'med', 'high'] as const).map((level) => (
                <option key={level} value={level}>
                  {level.toUpperCase()} {sensitivityValues[level].toFixed(1)}
                </option>
              ))}
            </select>
          </label>
        </div>
        <div className={`vector-plot ${isConnected ? '' : 'vector-plot-disconnected'}`}>
          <div className="axis-markers">
            <span className="axis-label top">+{axisRange.toFixed(1)}</span>
            <span className="axis-label bottom">-{axisRange.toFixed(1)}</span>
            <span className="axis-label left">+{axisRange.toFixed(1)}</span>
            <span className="axis-label right">-{axisRange.toFixed(1)}</span>
          </div>
          <div className="angvel-overlay">
            <svg viewBox="0 0 60 60" className="angvel-arc">
              <circle className="angvel-arc-track" cx={ARC_CENTER} cy={ARC_CENTER} r={yawRadius} />
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
      </div>
    </section>
  )
}

export default ControlVectorPlot
