import { useEffect, useRef, useState } from 'react'
import { getSikGatewayClient } from '../lib/sikGateway'
import './ArmServoCard.css'

type GamepadInfo = { index: number; id: string }

const LIN_SCALE = 0.6 // m/s equivalent for servo twist
const ANG_SCALE = 1.2 // rad/s equivalent for servo twist
const DEADZONE = 0.08
const CMD_PERIOD_MS = 50
const GRIPPER_RATE_PER_SEC = 0.08

const ArmServoCard = () => {
  const gatewayRef = useRef(getSikGatewayClient())
  const [gamepads, setGamepads] = useState<GamepadInfo[]>([])
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null)
  const [connected, setConnected] = useState(false) // gamepad present
  const [lastCmdDisplay, setLastCmdDisplay] = useState({
    lin_x: 0,
    lin_y: 0,
    lin_z: 0,
    ang_x: 0,
    ang_y: 0,
    ang_z: 0,
  })
  const cmdRef = useRef(lastCmdDisplay)
  const gripperRef = useRef(0.5)
  const hasPadRef = useRef(false)
  const controlEnabledRef = useRef(false)
  const lastTickMsRef = useRef<number | null>(null)
  const [gripperDisplay, setGripperDisplay] = useState(0.5)

  // Keep refs for animation loop
  const selectedRef = useRef<number | null>(null)
  useEffect(() => {
    selectedRef.current = selectedIndex
  }, [selectedIndex])

  useEffect(() => {
    gatewayRef.current.connect()
  }, [])

  // Gamepad discovery
  useEffect(() => {
    const updatePads = () => {
      const pads = navigator.getGamepads?.() ?? []
      const list = pads.reduce<GamepadInfo[]>((acc, pad, index) => {
        if (pad) acc.push({ index, id: pad.id || `Gamepad ${index + 1}` })
        return acc
      }, [])
      setGamepads(list)
      if (list.length === 0) {
        setSelectedIndex(null)
        controlEnabledRef.current = false
      } else if (controlEnabledRef.current && (selectedRef.current == null || !pads[selectedRef.current])) {
        setSelectedIndex(list[0].index)
      }
    }
    updatePads()
    window.addEventListener('gamepadconnected', updatePads)
    window.addEventListener('gamepaddisconnected', updatePads)
    return () => {
      window.removeEventListener('gamepadconnected', updatePads)
      window.removeEventListener('gamepaddisconnected', updatePads)
    }
  }, [])

  const applyDeadzone = (v: number) => (Math.abs(v) < DEADZONE ? 0 : v)

  // Read gamepad and update UI state continuously
  useEffect(() => {
    let frame = 0
    const tick = () => {
      const nowMs = performance.now()
      const prevMs = lastTickMsRef.current
      lastTickMsRef.current = nowMs
      const dtSec = prevMs == null ? 0 : Math.max(0, Math.min((nowMs - prevMs) / 1000, 0.1))
      const pads = navigator.getGamepads?.() ?? []
      const pad =
        controlEnabledRef.current && selectedRef.current != null
          ? pads[selectedRef.current]
          : null
      if (pad) {
        if (!hasPadRef.current) {
          hasPadRef.current = true
          setConnected(true)
        }
        const lx = applyDeadzone(pad.axes[0] ?? 0)
        const ly = applyDeadzone(pad.axes[1] ?? 0)
        const rx = applyDeadzone(pad.axes[2] ?? 0)
        const ry = applyDeadzone(pad.axes[3] ?? 0)
        const lt = pad.buttons[6]?.value ?? 0
        const rt = pad.buttons[7]?.value ?? 0
        const lb = pad.buttons[4]?.value ?? 0
        const rb = pad.buttons[5]?.value ?? 0
        const dpadLeft = pad.buttons[14]?.value ?? 0
        const dpadRight = pad.buttons[15]?.value ?? 0

        const lin_x = -ly * LIN_SCALE // forward/back
        const lin_y = lx * LIN_SCALE // left/right
        const lin_z = (rt - lt) * LIN_SCALE // triggers for up/down
        const ang_y = -ry * ANG_SCALE // pitch
        const ang_x = rx * ANG_SCALE // roll
        const ang_z = (Math.max(0, dpadRight) - Math.max(0, dpadLeft)) * ANG_SCALE // yaw

        const next = { lin_x, lin_y, lin_z, ang_x, ang_y, ang_z }
        cmdRef.current = next
        setLastCmdDisplay(next)

        const gripAxis = Math.max(0, rb) - Math.max(0, lb)
        const nextGrip = Math.min(1, Math.max(0, gripperRef.current + gripAxis * GRIPPER_RATE_PER_SEC * dtSec))
        gripperRef.current = nextGrip
        setGripperDisplay(nextGrip)
      } else {
        if (hasPadRef.current) {
          hasPadRef.current = false
          setConnected(false)
        }
        const zero = { lin_x: 0, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 0 }
        cmdRef.current = zero
        setLastCmdDisplay(zero)
        lastTickMsRef.current = null
      }
      frame = requestAnimationFrame(tick)
    }
    frame = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(frame)
  }, [connected])

  // Send servo commands at fixed rate
  useEffect(() => {
    const timer = window.setInterval(() => {
      const cmd = cmdRef.current
      gatewayRef.current.sendCmdArmTwist({
        lin_x_m_s: cmd.lin_x,
        lin_y_m_s: cmd.lin_y,
        lin_z_m_s: cmd.lin_z,
        ang_x_rad_s: cmd.ang_x,
        ang_y_rad_s: cmd.ang_y,
        ang_z_rad_s: cmd.ang_z,
      })
      if (controlEnabledRef.current) {
        gatewayRef.current.sendCmdArmGripper({
          position_norm: gripperRef.current,
        })
      }
    }, CMD_PERIOD_MS)
    return () => window.clearInterval(timer)
  }, [])

  return (
    <article className="card arm-card">
      <header className="arm-card__header">
        <h3>Arm Servo</h3>
        <span className={`pill ${connected ? 'pill--on' : 'pill--off'}`}>
          {connected ? 'gamepad connected' : 'no gamepad'}
        </span>
      </header>

      <label className="gamepad-picker">
        <span className="gamepad-label">Joystick</span>
        <select
          value={selectedIndex ?? ''}
          onChange={(e) => {
            const next = e.target.value === '' ? null : Number(e.target.value)
            controlEnabledRef.current = next !== null
            setSelectedIndex(next)
            if (next === null) {
              const zero = { lin_x: 0, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 0 }
              cmdRef.current = zero
              setLastCmdDisplay(zero)
              setConnected(false)
              lastTickMsRef.current = null
            }
          }}
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

      <div className="arm-card__grid">
        <div>
          <p className="arm-card__label">Linear (m/s)</p>
          <div className="arm-card__values">
            <span>X {lastCmdDisplay.lin_x.toFixed(2)}</span>
            <span>Y {lastCmdDisplay.lin_y.toFixed(2)}</span>
            <span>Z {lastCmdDisplay.lin_z.toFixed(2)}</span>
          </div>
        </div>
        <div>
          <p className="arm-card__label">Angular (rad/s)</p>
          <div className="arm-card__values">
            <span>Roll {lastCmdDisplay.ang_x.toFixed(2)}</span>
            <span>Pitch {lastCmdDisplay.ang_y.toFixed(2)}</span>
            <span>Yaw {lastCmdDisplay.ang_z.toFixed(2)}</span>
          </div>
        </div>
      </div>

      <div>
        <p className="arm-card__label">Gripper (normalized)</p>
        <div className="arm-card__values">
          <span>Pos {gripperDisplay.toFixed(2)}</span>
        </div>
      </div>

      <p className="arm-card__hint">
        LS: XY, Triggers: Z, RS: pitch/roll, D-pad left/right: yaw, Bumpers: gripper
      </p>
    </article>
  )
}

export default ArmServoCard
