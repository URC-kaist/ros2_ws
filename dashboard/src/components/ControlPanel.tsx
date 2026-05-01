import { useEffect, useRef, useState } from 'react'
import ControlEstopSection from './ControlPanel/ControlEstopSection'
import ControlStatusList from './ControlPanel/ControlStatusList'
import ControlVectorPlot, { type CmdVel } from './ControlPanel/ControlVectorPlot'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import './ControlPanel/ControlPanel.css'

const sensitivityScale = {
  low: 0.4,
  med: 0.8,
  high: 1.2,
} as const

const RADIUS = 0.57725 // meters

const ControlPanel = () => {
  const { gateway } = useXbeeGateway()
  const cmdVelRef = useRef<CmdVel>({ x: 0, y: 0, yaw: 0 })
  const [sensitivity, setSensitivity] = useState<'low' | 'med' | 'high'>('med')
  const [cmdVel, setCmdVel] = useState<CmdVel>({ x: 0, y: 0, yaw: 0 })
  const [gamepadConnected, setGamepadConnected] = useState(false)
  const [gamepads, setGamepads] = useState<Array<{ index: number; id: string }>>([])
  const [gamepadIndex, setGamepadIndex] = useState<number | null>(null)
  const gamepadConnectedRef = useRef(false)
  const gamepadIndexRef = useRef<number | null>(null)
  const controlEnabledRef = useRef(true)

  useEffect(() => {
    gamepadIndexRef.current = gamepadIndex
  }, [gamepadIndex])

  useEffect(() => {
    cmdVelRef.current = cmdVel
  }, [cmdVel])

  useEffect(() => {
    const heartbeatId = window.setInterval(() => {
      gateway.sendHeartbeat()
    }, 500)
    return () => window.clearInterval(heartbeatId)
  }, [gateway])

  useEffect(() => {
    const handleVisibility = () => {
      if (!document.hidden) {
        return
      }
      const zero = { x: 0, y: 0, yaw: 0 }
      cmdVelRef.current = zero
      setCmdVel(zero)
      gamepadConnectedRef.current = false
      setGamepadConnected(false)
      gateway.sendCmdDrive({
        linear_x_m_s: 0,
        linear_y_m_s: 0,
        angular_z_rad_s: 0,
      })
    }
    document.addEventListener('visibilitychange', handleVisibility)
    return () => document.removeEventListener('visibilitychange', handleVisibility)
  }, [gateway])

  useEffect(() => {
    const cmdRateMs = 50
    const cmdId = window.setInterval(() => {
      if (!gamepadConnected) {
        return
      }
      const latest = cmdVelRef.current
      gateway.sendCmdDrive({
        linear_x_m_s: latest.y,
        linear_y_m_s: latest.x,
        angular_z_rad_s: latest.yaw,
      })
    }, cmdRateMs)
    return () => window.clearInterval(cmdId)
  }, [gamepadConnected, gateway])

  useEffect(() => {
    const updateGamepads = () => {
      const pads = navigator.getGamepads?.() ?? []
      const list = pads.reduce<Array<{ index: number; id: string }>>((acc, pad, index) => {
        if (pad) {
          acc.push({ index, id: pad.id || `Gamepad ${index + 1}` })
        }
        return acc
      }, [])
      setGamepads(list)
      if (list.length === 0) {
        setGamepadIndex(null)
        controlEnabledRef.current = false
      } else if (
        controlEnabledRef.current &&
        (gamepadIndexRef.current == null || !pads[gamepadIndexRef.current])
      ) {
        setGamepadIndex(list[0].index)
      }
    }

    updateGamepads()
    window.addEventListener('gamepadconnected', updateGamepads)
    window.addEventListener('gamepaddisconnected', updateGamepads)
    return () => {
      window.removeEventListener('gamepadconnected', updateGamepads)
      window.removeEventListener('gamepaddisconnected', updateGamepads)
    }
  }, [])

  useEffect(() => {
    let frame = 0
    const deadzone = 0.08
    const scale = sensitivityScale[sensitivity]
    const yawScale = scale / RADIUS
    const xScale = scale
    const yScale = scale

    const applyDeadzone = (value: number) => (Math.abs(value) < deadzone ? 0 : value)

    const tick = () => {
      const pads = navigator.getGamepads?.() ?? []
      const selectedIndex = gamepadIndexRef.current
      const pad =
        controlEnabledRef.current && selectedIndex != null
          ? pads[selectedIndex]
          : controlEnabledRef.current
            ? null
            : null
      if (pad) {
        if (!gamepadConnectedRef.current) {
          gamepadConnectedRef.current = true
          setGamepadConnected(true)
        }
        const leftX = applyDeadzone(pad.axes[0] ?? 0)
        const rightX = applyDeadzone(pad.axes[2] ?? 0)
        const rightY = applyDeadzone(pad.axes[3] ?? 0)
        const leftTrigger = pad.buttons[6]?.value ?? 0
        const rightTrigger = pad.buttons[7]?.value ?? 0
        const baseY = (rightTrigger - leftTrigger) * yScale
        const addY = -rightY * yScale
        const next = {
          x: -rightX * xScale,
          y: baseY + addY,
          yaw: -leftX * yawScale,
        }

        setCmdVel(next)
      } else if (gamepadConnectedRef.current) {
        gamepadConnectedRef.current = false
        setGamepadConnected(false)
        setCmdVel({ x: 0, y: 0, yaw: 0 })
      }
      frame = requestAnimationFrame(tick)
    }

    frame = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(frame)
  }, [sensitivity])

  return (
    <aside className="control-panel">
      <ControlStatusList />
      <ControlVectorPlot
        cmdVel={cmdVel}
        sensitivity={sensitivity}
        isConnected={gamepadConnected}
        gamepads={gamepads}
        selectedGamepadIndex={gamepadIndex}
        onSelectGamepad={(value) => {
          controlEnabledRef.current = value !== null
          setGamepadIndex(value)
          if (value === null) {
            setGamepadConnected(false)
            setCmdVel({ x: 0, y: 0, yaw: 0 })
          }
        }}
        onSelectSensitivity={setSensitivity}
      />
      <ControlEstopSection />
    </aside>
  )
}

export default ControlPanel
