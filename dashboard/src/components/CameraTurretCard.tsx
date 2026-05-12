import { type PointerEvent, useCallback, useEffect, useRef, useState } from 'react'
import { useXbeeGateway } from '../hooks/useXbeeGateway'

const COMMAND_STEP = 0.01

type TurretCommand = { x: number; y: number; z: number }

const ZERO_COMMAND: TurretCommand = { x: 0, y: 0, z: 0 }

const clampUnit = (value: number) => Math.max(-1, Math.min(1, value))
const quantizeUnit = (value: number) =>
  clampUnit(Math.round(clampUnit(value) / COMMAND_STEP) * COMMAND_STEP)
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

  const updateFromPointer = (event: PointerEvent<HTMLDivElement>) => {
    const rect = event.currentTarget.getBoundingClientRect()
    const nextX = ((event.clientX - rect.left) / rect.width) * 2 - 1
    const nextY = 1 - ((event.clientY - rect.top) / rect.height) * 2
    publishCommand({
      x: quantizeUnit(nextX),
      y: quantizeUnit(nextY),
      z: 0,
    })
  }

  const center = () => {
    publishCommand(ZERO_COMMAND)
  }

  return (
    <article className="card camera-turret-card">
      <header className="camera-turret__header">
        <h3>Camera Turret</h3>
      </header>

      <div className="camera-turret__control">
        <div
          aria-label="Camera turret pan and tilt"
          className="camera-turret__pad"
          onPointerDown={(event) => {
            event.currentTarget.setPointerCapture(event.pointerId)
            updateFromPointer(event)
          }}
          onPointerMove={(event) => {
            if (event.currentTarget.hasPointerCapture(event.pointerId)) {
              updateFromPointer(event)
            }
          }}
          onPointerUp={(event) => {
            if (event.currentTarget.hasPointerCapture(event.pointerId)) {
              event.currentTarget.releasePointerCapture(event.pointerId)
            }
          }}
          onPointerCancel={(event) => {
            if (event.currentTarget.hasPointerCapture(event.pointerId)) {
              event.currentTarget.releasePointerCapture(event.pointerId)
            }
          }}
          role="application"
          tabIndex={0}
        >
          <span className="camera-turret__axis camera-turret__axis--x" />
          <span className="camera-turret__axis camera-turret__axis--y" />
          <span
            className="camera-turret__thumb"
            style={{
              left: `${((command.x + 1) / 2) * 100}%`,
              top: `${((1 - command.y) / 2) * 100}%`,
            }}
          />
        </div>
        <div className="camera-turret__readout">
          <span>Pan {command.x.toFixed(2)}</span>
          <span>Tilt {command.y.toFixed(2)}</span>
        </div>
        <button type="button" className="camera-turret__center" onClick={center}>
          Center
        </button>
      </div>
    </article>
  )
}

export default CameraTurretCard
