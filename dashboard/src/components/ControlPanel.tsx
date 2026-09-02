import { useCallback, useEffect, useRef, useState } from 'react'
import ControlEstopSection from './ControlPanel/ControlEstopSection'
import ControlStatusList from './ControlPanel/ControlStatusList'
import ControlVectorPlot, {
  type CmdVel,
  type ControlMode,
  type OrbitCenter,
  type SensitivityLevel,
} from './ControlPanel/ControlVectorPlot'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import './ControlPanel/ControlPanel.css'

const sensitivityScale = {
  low: 0.2,
  med: 0.4,
  high: 0.8,
} as const

const RADIUS = 0.57725 // meters
const WHEEL_BASE = 0.95386 // meters
const TRACK_WIDTH = 0.6504 // meters
const MAX_WHEEL_LINEAR_SPEED = 0.8 // meters per second
const MODE_HOLD_MS = 2000
const GAMEPAD_DEADZONE = 0.08

const centerSensitivityScale = {
  low: 0.1,
  med: 0.25,
  high: 0.5,
} as const

const getSafeOrbitYawRate = (center: OrbitCenter) => {
  const halfLength = WHEEL_BASE * 0.5
  const halfWidth = TRACK_WIDTH * 0.5
  const wheels = [
    { x: -halfWidth, y: halfLength },
    { x: halfWidth, y: halfLength },
    { x: -halfWidth, y: -halfLength },
    { x: halfWidth, y: -halfLength },
  ]
  const farthestWheelDistance = Math.max(
    ...wheels.map((wheel) => Math.hypot(wheel.x - center.x, wheel.y - center.y))
  )
  return farthestWheelDistance > 0
    ? MAX_WHEEL_LINEAR_SPEED / farthestWheelDistance
    : MAX_WHEEL_LINEAR_SPEED / RADIUS
}

const ControlPanel = () => {
  const { gateway } = useXbeeGateway()
  const cmdVelRef = useRef<CmdVel>({ x: 0, y: 0, yaw: 0 })
  const [sensitivity, setSensitivity] = useState<SensitivityLevel>('med')
  const [rotationSensitivity, setRotationSensitivity] = useState<SensitivityLevel>('med')
  const [centerSensitivity, setCenterSensitivity] = useState<SensitivityLevel>('med')
  const [cmdVel, setCmdVel] = useState<CmdVel>({ x: 0, y: 0, yaw: 0 })
  const [controlMode, setControlMode] = useState<ControlMode>('manual')
  const [orbitCenter, setOrbitCenter] = useState<OrbitCenter>({ x: 0, y: 0 })
  const [orbitYaw, setOrbitYaw] = useState(0)
  const [requestedOrbitYaw, setRequestedOrbitYaw] = useState(0)
  const [safeOrbitYaw, setSafeOrbitYaw] = useState(MAX_WHEEL_LINEAR_SPEED / RADIUS)
  const [orbitNeedsNeutral, setOrbitNeedsNeutral] = useState(false)
  const [modeSwitchProgress, setModeSwitchProgress] = useState(0)
  const [gamepadConnected, setGamepadConnected] = useState(false)
  const [gamepads, setGamepads] = useState<Array<{ index: number; id: string }>>([])
  const [gamepadIndex, setGamepadIndex] = useState<number | null>(null)
  const gamepadConnectedRef = useRef(false)
  const gamepadIndexRef = useRef<number | null>(null)
  const controlEnabledRef = useRef(true)
  const controlModeRef = useRef<ControlMode>('manual')
  const orbitCenterRef = useRef<OrbitCenter>({ x: 0, y: 0 })
  const orbitNeedsNeutralRef = useRef(false)
  const modeHoldStartedAtRef = useRef<number | null>(null)
  const modeChordLatchedRef = useRef(false)

  useEffect(() => {
    gamepadIndexRef.current = gamepadIndex
  }, [gamepadIndex])

  const updateCommand = useCallback((next: CmdVel) => {
    cmdVelRef.current = next
    setCmdVel(next)
  }, [])

  const sendZeroCommand = useCallback(() => {
    const zero = { x: 0, y: 0, yaw: 0 }
    updateCommand(zero)
    setOrbitYaw(0)
    setRequestedOrbitYaw(0)
    gateway.sendCmdDrive({
      linear_x_m_s: 0,
      linear_y_m_s: 0,
      angular_z_rad_s: 0,
    })
  }, [gateway, updateCommand])

  const selectControlMode = useCallback(
    (nextMode: ControlMode) => {
      if (controlModeRef.current === nextMode) return
      sendZeroCommand()
      controlModeRef.current = nextMode
      setControlMode(nextMode)
      modeHoldStartedAtRef.current = null
      modeChordLatchedRef.current = true
      setModeSwitchProgress(0)

      if (nextMode === 'orbit') {
        const origin = { x: 0, y: 0 }
        orbitCenterRef.current = origin
        setOrbitCenter(origin)
        setSafeOrbitYaw(getSafeOrbitYawRate(origin))
        orbitNeedsNeutralRef.current = true
        setOrbitNeedsNeutral(true)
      } else {
        orbitNeedsNeutralRef.current = false
        setOrbitNeedsNeutral(false)
      }
    },
    [sendZeroCommand]
  )

  const updateOrbitCenter = useCallback((next: OrbitCenter) => {
    if (!Number.isFinite(next.x) || !Number.isFinite(next.y)) return
    orbitCenterRef.current = next
    setOrbitCenter(next)
    setSafeOrbitYaw(getSafeOrbitYawRate(next))
  }, [])

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
      sendZeroCommand()
      gamepadConnectedRef.current = false
      setGamepadConnected(false)
      if (controlModeRef.current === 'orbit') {
        orbitNeedsNeutralRef.current = true
        setOrbitNeedsNeutral(true)
      }
    }
    document.addEventListener('visibilitychange', handleVisibility)
    return () => document.removeEventListener('visibilitychange', handleVisibility)
  }, [sendZeroCommand])

  useEffect(() => {
    const cmdRateMs = 50
    const cmdId = window.setInterval(() => {
      if (!gamepadConnectedRef.current) {
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
  }, [gateway])

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
    const scale = sensitivityScale[sensitivity]
    const yawScale = scale / RADIUS
    const xScale = scale
    const yScale = scale
    const orbitYawScale = sensitivityScale[rotationSensitivity] / RADIUS
    const orbitCenterRate = centerSensitivityScale[centerSensitivity]
    let previousFrameAt = performance.now()

    const applyDeadzone = (value: number) =>
      Math.abs(value) < GAMEPAD_DEADZONE ? 0 : value

    const tick = (frameAt: number) => {
      const dt = Math.min(Math.max((frameAt - previousFrameAt) / 1000, 0), 0.05)
      previousFrameAt = frameAt
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
        const modeChordPressed =
          Boolean(pad.buttons[4]?.pressed || (pad.buttons[4]?.value ?? 0) > 0.5) &&
          Boolean(pad.buttons[5]?.pressed || (pad.buttons[5]?.value ?? 0) > 0.5)

        if (modeChordPressed) {
          if (!modeChordLatchedRef.current) {
            if (modeHoldStartedAtRef.current == null) {
              modeHoldStartedAtRef.current = frameAt
            }
            const progress = Math.min(
              (frameAt - modeHoldStartedAtRef.current) / MODE_HOLD_MS,
              1
            )
            setModeSwitchProgress(progress)
            if (progress >= 1) {
              const nextMode = controlModeRef.current === 'manual' ? 'orbit' : 'manual'
              selectControlMode(nextMode)
            }
          }
        } else {
          modeHoldStartedAtRef.current = null
          modeChordLatchedRef.current = false
          setModeSwitchProgress(0)
        }

        if (controlModeRef.current === 'orbit') {
          const center = orbitCenterRef.current
          const nextCenter = {
            x: center.x + rightX * orbitCenterRate * dt,
            y: center.y - rightY * orbitCenterRate * dt,
          }
          if (rightX !== 0 || rightY !== 0) {
            updateOrbitCenter(nextCenter)
          }

          if (orbitNeedsNeutralRef.current) {
            if (leftX === 0) {
              orbitNeedsNeutralRef.current = false
              setOrbitNeedsNeutral(false)
            }
            updateCommand({ x: 0, y: 0, yaw: 0 })
            setOrbitYaw(0)
            setRequestedOrbitYaw(0)
          } else {
            const requestedYaw = -leftX * orbitYawScale
            const yawLimit = getSafeOrbitYawRate(orbitCenterRef.current)
            const yaw = Math.max(-yawLimit, Math.min(yawLimit, requestedYaw))
            const activeCenter = orbitCenterRef.current
            const next = {
              // UI coordinates: +X right, +Y forward. ROS uses +X forward, +Y left.
              x: -yaw * activeCenter.y,
              y: -yaw * activeCenter.x,
              yaw,
            }
            updateCommand(next)
            setRequestedOrbitYaw(requestedYaw)
            setOrbitYaw(yaw)
            setSafeOrbitYaw(yawLimit)
          }
        } else {
          const leftTrigger = pad.buttons[6]?.value ?? 0
          const rightTrigger = pad.buttons[7]?.value ?? 0
          const baseY = (rightTrigger - leftTrigger) * yScale
          const addY = -rightY * yScale
          const next = {
            x: -rightX * xScale,
            y: baseY + addY,
            yaw: -leftX * yawScale,
          }

          updateCommand(next)
        }
      } else if (gamepadConnectedRef.current) {
        gamepadConnectedRef.current = false
        setGamepadConnected(false)
        sendZeroCommand()
        if (controlModeRef.current === 'orbit') {
          orbitNeedsNeutralRef.current = true
          setOrbitNeedsNeutral(true)
        }
      }
      frame = requestAnimationFrame(tick)
    }

    frame = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(frame)
  }, [
    centerSensitivity,
    rotationSensitivity,
    selectControlMode,
    sendZeroCommand,
    sensitivity,
    updateCommand,
    updateOrbitCenter,
  ])

  return (
    <aside className="control-panel">
      <ControlStatusList />
      <ControlVectorPlot
        cmdVel={cmdVel}
        controlMode={controlMode}
        onSelectControlMode={selectControlMode}
        modeSwitchProgress={modeSwitchProgress}
        sensitivity={sensitivity}
        rotationSensitivity={rotationSensitivity}
        centerSensitivity={centerSensitivity}
        orbitCenter={orbitCenter}
        orbitYaw={orbitYaw}
        requestedOrbitYaw={requestedOrbitYaw}
        safeOrbitYaw={safeOrbitYaw}
        orbitNeedsNeutral={orbitNeedsNeutral}
        isConnected={gamepadConnected}
        gamepads={gamepads}
        selectedGamepadIndex={gamepadIndex}
        onSelectGamepad={(value) => {
          controlEnabledRef.current = value !== null
          setGamepadIndex(value)
          if (value === null) {
            sendZeroCommand()
            setGamepadConnected(false)
            gamepadConnectedRef.current = false
            if (controlModeRef.current === 'orbit') {
              orbitNeedsNeutralRef.current = true
              setOrbitNeedsNeutral(true)
            }
          }
        }}
        onSelectSensitivity={setSensitivity}
        onSelectRotationSensitivity={setRotationSensitivity}
        onSelectCenterSensitivity={setCenterSensitivity}
        onSetOrbitCenter={updateOrbitCenter}
      />
      <ControlEstopSection />
    </aside>
  )
}

export default ControlPanel
