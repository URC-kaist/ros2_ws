import { FiSettings } from 'react-icons/fi'
import IconButton from '../ui/IconButton'
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
  onSelectGamepad: (index: number) => void
  isSettingsOpen: boolean
  onToggleSettings: () => void
}

const sensitivityScale = {
  low: 0.6,
  med: 1.0,
  high: 1.6,
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
  isSettingsOpen,
  onToggleSettings,
}: ControlVectorPlotProps) => {
  const axisRange = 1 * sensitivityScale[sensitivity]
  const yawRange = 1 * sensitivityScale[sensitivity]
  const magnitude = Math.hypot(cmdVel.x, cmdVel.y)
  const arrowScale = Math.min(1, magnitude / axisRange)
  const arrowAngle = Math.atan2(-cmdVel.y, cmdVel.x) * (180 / Math.PI)
  const yawMagnitude = Math.min(1, Math.abs(cmdVel.yaw) / yawRange)
  const yawSweep = yawMagnitude * 0.75
  const yawRadius = 26
  const yawAngle = yawSweep * 360
  const yawStartAngle = -90
  const yawEndAngle = yawStartAngle + (cmdVel.yaw >= 0 ? yawAngle : -yawAngle)
  const yawLargeArc = yawAngle > 180 ? 1 : 0
  const yawSweepFlag = cmdVel.yaw >= 0 ? 1 : 0
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
        <label className="gamepad-picker">
          <span className="gamepad-header">
            Gamepad
            <IconButton
              className="settings-button"
              ariaLabel="Open settings"
              pressed={isSettingsOpen}
              onClick={onToggleSettings}
            >
              <FiSettings aria-hidden="true" />
            </IconButton>
          </span>
          <select
            value={selectedGamepadIndex ?? (gamepads[0]?.index ?? '')}
            onChange={(event) => onSelectGamepad(Number(event.target.value))}
            disabled={gamepads.length === 0}
          >
            {gamepads.length === 0 ? (
              <option value="">No gamepad detected</option>
            ) : (
              gamepads.map((pad) => (
                <option key={pad.index} value={pad.index}>
                  {pad.id}
                </option>
              ))
            )}
          </select>
        </label>
        <div className={`vector-plot ${isConnected ? '' : 'vector-plot-disconnected'}`}>
          <div className="axis-markers">
            <span className="axis-label top">+{axisRange.toFixed(1)}</span>
            <span className="axis-label bottom">-{axisRange.toFixed(1)}</span>
            <span className="axis-label left">-{axisRange.toFixed(1)}</span>
            <span className="axis-label right">+{axisRange.toFixed(1)}</span>
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
              <span className="vector-value">x {formatSigned(cmdVel.x)} m/s</span>
              <span className="vector-value">y {formatSigned(cmdVel.y)} m/s</span>
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
