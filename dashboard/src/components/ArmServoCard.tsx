import { useEffect, useRef, useState } from 'react'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import './ArmServoCard.css'

type GamepadInfo = { index: number; id: string }
type ArmServoMode = 'cartesian' | 'joint'

const LIN_SCALE = 0.6 // m/s equivalent for servo twist
const ANG_SCALE = 1.2 // rad/s equivalent for servo twist
const JOINT_SCALE = 1.5 // rad/s at full joint command scale
const DEADZONE = 0.08
const CMD_PERIOD_MS = 50
const GRIPPER_RATE_PER_SEC = 0.08
const ZERO_TWIST = { lin_x: 0, lin_y: 0, lin_z: 0, ang_x: 0, ang_y: 0, ang_z: 0 }
const ZERO_JOINTS = [0, 0, 0, 0, 0, 0]

const ArmServoCard = () => {
  const { gateway } = useXbeeGateway()
  const [gamepads, setGamepads] = useState<GamepadInfo[]>([])
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null)
  const [connected, setConnected] = useState(false) // gamepad present
  const [mode, setMode] = useState<ArmServoMode>('cartesian')
  const modeRef = useRef<ArmServoMode>('cartesian')
  const [lastCmdDisplay, setLastCmdDisplay] = useState({
    lin_x: 0,
    lin_y: 0,
    lin_z: 0,
    ang_x: 0,
    ang_y: 0,
    ang_z: 0,
  })
  const [jointDisplay, setJointDisplay] = useState<number[]>(ZERO_JOINTS)
  const [jointSpeed, setJointSpeed] = useState(0.25)
  const cmdRef = useRef(lastCmdDisplay)
  const jointCmdRef = useRef<number[]>(ZERO_JOINTS)
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
        cmdRef.current = ZERO_TWIST
        setLastCmdDisplay(ZERO_TWIST)
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
      if (modeRef.current === 'cartesian') {
        const cmd = cmdRef.current
        gateway.sendCmdArmTwist({
          lin_x_m_s: cmd.lin_x,
          lin_y_m_s: cmd.lin_y,
          lin_z_m_s: cmd.lin_z,
          ang_x_rad_s: cmd.ang_x,
          ang_y_rad_s: cmd.ang_y,
          ang_z_rad_s: cmd.ang_z,
        })
      } else {
        gateway.sendCmdArmJoint({
          velocities_rad_s: jointCmdRef.current,
        })
      }
      if (controlEnabledRef.current) {
        gateway.sendCmdArmGripper({
          position_norm: gripperRef.current,
        })
      }
    }, CMD_PERIOD_MS)
    return () => window.clearInterval(timer)
  }, [gateway])

  const setJointJog = (jointIndex: number, direction: -1 | 1) => {
    const next = ZERO_JOINTS.map((_, index) =>
      index === jointIndex ? direction * jointSpeed * JOINT_SCALE : 0,
    )
    jointCmdRef.current = next
    setJointDisplay(next)
  }

  const stopJointJog = () => {
    jointCmdRef.current = ZERO_JOINTS
    setJointDisplay(ZERO_JOINTS)
    gateway.sendCmdArmJoint({ velocities_rad_s: ZERO_JOINTS })
  }

  const stopAllArmMotion = () => {
    cmdRef.current = ZERO_TWIST
    jointCmdRef.current = ZERO_JOINTS
    setLastCmdDisplay(ZERO_TWIST)
    setJointDisplay(ZERO_JOINTS)
    gateway.sendCmdArmTwist({
      lin_x_m_s: 0,
      lin_y_m_s: 0,
      lin_z_m_s: 0,
      ang_x_rad_s: 0,
      ang_y_rad_s: 0,
      ang_z_rad_s: 0,
    })
    gateway.sendCmdArmJoint({ velocities_rad_s: ZERO_JOINTS })
  }

  const switchArmMode = (nextMode: ArmServoMode) => {
    if (modeRef.current === nextMode) return
    stopAllArmMotion()
    modeRef.current = nextMode
    setMode(nextMode)
  }

  useEffect(() => {
    window.addEventListener('blur', stopAllArmMotion)
    return () => window.removeEventListener('blur', stopAllArmMotion)
  })

  return (
    <article className="card arm-card">
      <header className="arm-card__header">
        <h3>Arm Servo</h3>
        <span className={`pill ${connected ? 'pill--on' : 'pill--off'}`}>
          {connected ? 'gamepad connected' : 'no gamepad'}
        </span>
      </header>

      <div className="arm-card__toolbar">
        <div className="arm-card__mode" role="tablist" aria-label="Arm servo mode">
          <button
            className={mode === 'cartesian' ? 'active' : ''}
            type="button"
            onClick={() => switchArmMode('cartesian')}
          >
            Cartesian
          </button>
          <button
            className={mode === 'joint' ? 'active' : ''}
            type="button"
            onClick={() => switchArmMode('joint')}
          >
            Joint
          </button>
        </div>
        <button className="arm-card__stop" type="button" onClick={stopAllArmMotion}>
          Stop Arm
        </button>
      </div>

      <label className="gamepad-picker">
        <span className="gamepad-label">Joystick</span>
        <select
          value={selectedIndex ?? ''}
          onChange={(e) => {
            const next = e.target.value === '' ? null : Number(e.target.value)
            controlEnabledRef.current = next !== null
            setSelectedIndex(next)
            if (next === null) {
              stopAllArmMotion()
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

      {mode === 'cartesian' ? (
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
      ) : (
        <div className="arm-card__joint-panel">
          <label className="arm-card__speed">
            <span>Joint speed</span>
            <input
              max="1"
              min="0.05"
              step="0.05"
              type="range"
              value={jointSpeed}
              onChange={(event) => setJointSpeed(Number(event.target.value))}
            />
            <strong>{(jointSpeed * JOINT_SCALE).toFixed(2)} rad/s</strong>
          </label>
          <div className="arm-card__joint-grid">
            {jointDisplay.map((velocity, index) => (
              <div className="arm-card__joint-row" key={`arm-j${index + 1}`}>
                <span>J{index + 1}</span>
                <button
                  aria-label={`Jog J${index + 1} negative`}
                  type="button"
                  onPointerDown={(event) => {
                    event.currentTarget.setPointerCapture(event.pointerId)
                    setJointJog(index, -1)
                  }}
                  onPointerUp={stopJointJog}
                  onPointerCancel={stopJointJog}
                  onLostPointerCapture={stopJointJog}
                >
                  -
                </button>
                <output>{velocity.toFixed(2)}</output>
                <button
                  aria-label={`Jog J${index + 1} positive`}
                  type="button"
                  onPointerDown={(event) => {
                    event.currentTarget.setPointerCapture(event.pointerId)
                    setJointJog(index, 1)
                  }}
                  onPointerUp={stopJointJog}
                  onPointerCancel={stopJointJog}
                  onLostPointerCapture={stopJointJog}
                >
                  +
                </button>
              </div>
            ))}
          </div>
        </div>
      )}

      <div>
        <p className="arm-card__label">Gripper (normalized)</p>
        <div className="arm-card__values">
          <span>Pos {gripperDisplay.toFixed(2)}</span>
        </div>
      </div>

      <p className="arm-card__hint">
        {mode === 'cartesian'
          ? 'LS: XY, Triggers: Z, RS: pitch/roll, D-pad left/right: yaw, Bumpers: gripper'
          : 'Hold +/- to jog one joint at the selected speed. Releasing sends zero joint velocity.'}
      </p>
    </article>
  )
}

export default ArmServoCard
