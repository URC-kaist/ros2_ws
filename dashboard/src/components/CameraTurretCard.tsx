import { useCallback, useEffect, useRef, useState } from 'react'
import { useXbeeGateway } from '../hooks/useXbeeGateway'

const SLIDER_STEP = 0.01

type TurretCommand = { x: number; y: number; z: number }

const ZERO_COMMAND: TurretCommand = { x: 0, y: 0, z: 0 }

const clampUnit = (value: number) => Math.max(-1, Math.min(1, value))
const commandsEqual = (a: TurretCommand, b: TurretCommand) =>
  a.x === b.x && a.y === b.y && a.z === b.z

const CameraTurretCard = () => {
  const { gateway } = useXbeeGateway()
  const [command, setCommand] = useState<TurretCommand>(ZERO_COMMAND)
  const commandRef = useRef<TurretCommand>(ZERO_COMMAND)

  const publishCommand = useCallback((next: TurretCommand) => {
    if (commandsEqual(commandRef.current, next)) {
      return
    }
    commandRef.current = next
    setCommand(next)
    gateway.sendCmdCameraTurret(next)
  }, [gateway])

  useEffect(() => {
    const handleVisibility = () => {
      if (!document.hidden) {
        return
      }
      publishCommand(ZERO_COMMAND)
    }
    document.addEventListener('visibilitychange', handleVisibility)
    return () => document.removeEventListener('visibilitychange', handleVisibility)
  }, [publishCommand])

  useEffect(() => {
    return () => {
      if (!commandsEqual(commandRef.current, ZERO_COMMAND)) {
        gateway.sendCmdCameraTurret(ZERO_COMMAND)
      }
    }
  }, [gateway])

  const updateAxis = (axis: keyof TurretCommand, value: number) => {
    publishCommand({
      ...commandRef.current,
      [axis]: clampUnit(value),
    })
  }

  const center = () => {
    publishCommand(ZERO_COMMAND)
  }

  return (
    <article className="card camera-turret-card">
      <header className="camera-turret__header">
        <h3>Camera Turret</h3>
        <span className="pill pill--on">slider control</span>
      </header>

      <div className="camera-turret__sliders">
        <label className="camera-turret__slider">
          <span className="camera-turret__slider-label">
            <span>Pan</span>
            <output>{command.x.toFixed(2)}</output>
          </span>
          <input
            type="range"
            min="-1"
            max="1"
            step={SLIDER_STEP}
            value={command.x}
            onChange={(event) => updateAxis('x', Number(event.target.value))}
          />
        </label>

        <label className="camera-turret__slider">
          <span className="camera-turret__slider-label">
            <span>Tilt</span>
            <output>{command.y.toFixed(2)}</output>
          </span>
          <input
            type="range"
            min="-1"
            max="1"
            step={SLIDER_STEP}
            value={command.y}
            onChange={(event) => updateAxis('y', Number(event.target.value))}
          />
        </label>

        <button type="button" className="camera-turret__center" onClick={center}>
          Center
        </button>
      </div>
    </article>
  )
}

export default CameraTurretCard
