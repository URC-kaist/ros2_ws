import './ControlVectorPlot.css'

export type CmdVel = {
  x: number
  y: number
  yaw: number
}

type ControlVectorPlotProps = {
  cmdVel: CmdVel
  sensitivity: 'low' | 'med' | 'high'
}

const sensitivityScale = {
  low: 0.6,
  med: 1.0,
  high: 1.6,
} as const

const ControlVectorPlot = ({ cmdVel, sensitivity }: ControlVectorPlotProps) => {
  const axisRange = 1 * sensitivityScale[sensitivity]
  const yawRange = 1 * sensitivityScale[sensitivity]
  const magnitude = Math.hypot(cmdVel.x, cmdVel.y)
  const arrowScale = Math.min(1, magnitude / axisRange)
  const arrowAngle = Math.atan2(-cmdVel.y, cmdVel.x) * (180 / Math.PI)
  const yawMagnitude = Math.min(1, Math.abs(cmdVel.yaw) / yawRange)
  const yawSweep = yawMagnitude * 0.75
  const yawRotation = cmdVel.yaw >= 0 ? -90 : 90
  const yawRadius = 26
  const yawCircumference = 2 * Math.PI * yawRadius

  return (
    <section className="panel-section">
      <div className="control-visual">
        <div className="vector-plot">
          <div className="axis-markers">
            <span className="axis-label top">+{axisRange.toFixed(1)}</span>
            <span className="axis-label bottom">-{axisRange.toFixed(1)}</span>
            <span className="axis-label left">-{axisRange.toFixed(1)}</span>
            <span className="axis-label right">+{axisRange.toFixed(1)}</span>
          </div>
          <div className="angvel-overlay">
            <svg viewBox="0 0 60 60" className="angvel-arc">
              <circle className="angvel-arc-track" cx="30" cy="30" r={yawRadius} />
              <circle
                className="angvel-arc-sweep"
                cx="30"
                cy="30"
                r={yawRadius}
                style={{
                  strokeDasharray: `${yawCircumference * yawSweep} ${yawCircumference}`,
                  transform: `rotate(${yawRotation}deg)`,
                }}
              />
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
          <span className="vector-value">x {cmdVel.x.toFixed(2)} m/s</span>
          <span className="vector-value">y {cmdVel.y.toFixed(2)} m/s</span>
          <span className="vector-value">ω {cmdVel.yaw.toFixed(2)} rad/s</span>
        </div>
      </div>
    </section>
  )
}

export default ControlVectorPlot
