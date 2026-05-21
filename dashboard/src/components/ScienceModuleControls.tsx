import { useCallback, useEffect, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'
import type {
  CarriageMotorCommandRequest,
  CarriageMotorTelemetryMsg,
  MotorPositionRequest,
  MotorVelocityRequest,
  MoveCarriageRequest,
  PumpRequest,
  ScienceServiceResponse,
  SetScienceLedRequest,
} from '../lib/rosMessages'

const PUMP_SERVICE = '/science/pump'
const PUMP_TYPE = 'mr2_science_module/srv/Pump'
const LED_SERVICE = '/science/led'
const LED_TYPE = 'mr2_science_module/srv/SetScienceLed'
const CARRIAGE_SERVICE = '/science/carriage'
const CARRIAGE_TYPE = 'mr2_science_module/srv/MoveCarriage'
const CARRIAGE_MOTOR_SERVICE = '/science/carriage_motor'
const CARRIAGE_MOTOR_TYPE = 'mr2_science_module/srv/CarriageMotorCommand'
const DRILL_VELOCITY_SERVICE = '/science/drill_velocity'
const MOTOR_VELOCITY_TYPE = 'mr2_science_module/srv/MotorVelocity'
const DRILL_POSITION_SERVICE = '/science/drill_position'
const MOTOR_POSITION_TYPE = 'mr2_science_module/srv/MotorPosition'
const CARRIAGE_TELEMETRY_TOPIC = '/science/carriage_motor/telemetry'
const CARRIAGE_TELEMETRY_TYPE = 'mr2_science_module/msg/CarriageMotorTelemetry'

type PendingCommand = string | null
type StatusState = { ok: boolean; text: string } | null
const MICROMETERS_PER_CM = 10000

const clamp = (value: number, min: number, max: number) =>
  Math.min(max, Math.max(min, value))

const numberFromInput = (value: string, fallback: number) => {
  const parsed = Number(value)
  return Number.isFinite(parsed) ? parsed : fallback
}

const formatSigned = (value?: number, suffix = '') => {
  if (!Number.isFinite(value)) return '--'
  return `${value?.toLocaleString()}${suffix}`
}

const formatCm = (valueUm?: number) => {
  if (!Number.isFinite(valueUm)) return '--'
  return `${((valueUm ?? 0) / MICROMETERS_PER_CM).toFixed(3)} cm`
}

const useScienceService = () => {
  const { ros, connected } = useRosBridge()
  const [pending, setPending] = useState<PendingCommand>(null)
  const [status, setStatus] = useState<StatusState>(null)

  const callService = useCallback(
    async <TRequest,>(
      command: string,
      service: string,
      serviceType: string,
      request: TRequest
    ) => {
      if (!ros.isConnected()) {
        setStatus({ ok: false, text: 'ROS bridge is offline' })
        return false
      }
      setPending(command)
      setStatus(null)
      try {
        const response = await ros.callService<TRequest, ScienceServiceResponse>(
          service,
          serviceType,
          request
        )
        setStatus({
          ok: response.success,
          text: response.message || (response.success ? 'Sent' : 'Command failed'),
        })
        return response.success
      } catch (error) {
        setStatus({
          ok: false,
          text: error instanceof Error ? error.message : 'Service call failed',
        })
        return false
      } finally {
        setPending(null)
      }
    },
    [ros]
  )

  return { callService, connected, pending, status }
}

const CommandStatus = ({ status }: { status: StatusState }) => {
  if (!status) return null
  return (
    <div className={`science-module__status ${status.ok ? '' : 'science-module__status--error'}`}>
      {status.text}
    </div>
  )
}

export const SciencePumpLedCard = () => {
  const { callService, connected, pending, status } = useScienceService()
  const [pumpGroup, setPumpGroup] = useState(0)
  const [pumpDurationMs, setPumpDurationMs] = useState(0)
  const [ledEnabled, setLedEnabled] = useState(false)
  const [brightness, setBrightness] = useState(128)

  const runPump = () => {
    void callService<PumpRequest>('pump', PUMP_SERVICE, PUMP_TYPE, {
      group: pumpGroup,
      duration_ms: clamp(Math.round(pumpDurationMs), 0, 65535),
    })
  }

  const applyLed = () => {
    void callService<SetScienceLedRequest>('led', LED_SERVICE, LED_TYPE, {
      enabled: ledEnabled,
      brightness: clamp(Math.round(brightness), 0, 255),
    })
  }

  return (
    <article className="card science-module-card">
      <header className="science-module__header">
        <div>
          <h3>Pumps & Light</h3>
          <p>Sample pump groups and science module illumination.</p>
        </div>
        <span className={`pill ${connected ? 'pill--on' : 'pill--off'}`}>
          {connected ? 'ROS online' : 'ROS offline'}
        </span>
      </header>

      <section className="science-module__section">
        <h4>Pump</h4>
        <div className="science-module__form">
          <label className="science-module__field">
            <span>Group</span>
            <select
              value={pumpGroup}
              onChange={(event) => setPumpGroup(Number(event.target.value))}
            >
              <option value={0}>Pumps 2 & 4</option>
              <option value={1}>Pumps 1 & 3</option>
            </select>
          </label>
          <label className="science-module__field">
            <span>Duration (ms)</span>
            <input
              type="number"
              min={0}
              max={65535}
              step={100}
              value={pumpDurationMs}
              onChange={(event) =>
                setPumpDurationMs(numberFromInput(event.target.value, pumpDurationMs))
              }
            />
          </label>
          <button
            className="science-module__button"
            type="button"
            disabled={!connected || pending === 'pump'}
            onClick={runPump}
          >
            {pending === 'pump' ? 'Sending...' : 'Run pump'}
          </button>
        </div>
      </section>

      <section className="science-module__section">
        <h4>LED</h4>
        <div className="science-module__form">
          <label className="science-module__toggle">
            <input
              type="checkbox"
              checked={ledEnabled}
              onChange={(event) => setLedEnabled(event.target.checked)}
            />
            <span>{ledEnabled ? 'Enabled' : 'Disabled'}</span>
          </label>
          <label className="science-module__field">
            <span>Brightness: {brightness}</span>
            <input
              className="science-module__range"
              type="range"
              min={0}
              max={255}
              step={1}
              value={brightness}
              disabled={!ledEnabled}
              onChange={(event) => setBrightness(numberFromInput(event.target.value, brightness))}
            />
          </label>
          <button
            className="science-module__button"
            type="button"
            disabled={!connected || pending === 'led'}
            onClick={applyLed}
          >
            {pending === 'led' ? 'Sending...' : 'Apply LED'}
          </button>
        </div>
      </section>
      <CommandStatus status={status} />
    </article>
  )
}

export const ScienceCarriageCard = () => {
  const { ros, connected } = useRosBridge()
  const { callService, pending, status } = useScienceService()
  const [positionCommand, setPositionCommand] = useState(1)
  const [vibrationMs, setVibrationMs] = useState(0)
  const [motorMode, setMotorMode] = useState(1)
  const [motorValue, setMotorValue] = useState(0)
  const [telemetry, setTelemetry] = useState<CarriageMotorTelemetryMsg | null>(null)

  useEffect(() => {
    return ros.subscribe<CarriageMotorTelemetryMsg>(
      CARRIAGE_TELEMETRY_TOPIC,
      CARRIAGE_TELEMETRY_TYPE,
      setTelemetry,
      { throttleRate: 100, queueSize: 1 }
    )
  }, [ros])

  const moveCarriage = (command = positionCommand) => {
    void callService<MoveCarriageRequest>('carriage', CARRIAGE_SERVICE, CARRIAGE_TYPE, {
      command,
      vibration_duration_ms: command === 0 ? 0 : clamp(Math.round(vibrationMs), 0, 65535),
    })
  }

  const sendMotor = (mode = motorMode) => {
    const value =
      mode === 2 ? Math.round(motorValue * MICROMETERS_PER_CM) : Math.round(motorValue)
    void callService<CarriageMotorCommandRequest>(
      'carriage-motor',
      CARRIAGE_MOTOR_SERVICE,
      CARRIAGE_MOTOR_TYPE,
      {
        mode,
        value: mode === 1 || mode === 2 ? value : 0,
      }
    )
  }

  return (
    <article className="card science-module-card">
      <header className="science-module__header">
        <div>
          <h3>Carriage</h3>
          <p>Tray position selection and carriage motor commands.</p>
        </div>
        <span className={`pill ${connected ? 'pill--on' : 'pill--off'}`}>
          {connected ? 'ROS online' : 'ROS offline'}
        </span>
      </header>

      <section className="science-module__section">
        <h4>Move carriage</h4>
        <div className="science-module__form">
          <label className="science-module__field">
            <span>Position</span>
            <select
              value={positionCommand}
              onChange={(event) => setPositionCommand(Number(event.target.value))}
            >
              {[1, 2, 3, 4, 5, 6].map((position) => (
                <option key={position} value={position}>
                  Position {position}
                </option>
              ))}
            </select>
          </label>
          <label className="science-module__field">
            <span>Vibration (ms)</span>
            <input
              type="number"
              min={0}
              max={65535}
              step={100}
              value={vibrationMs}
              onChange={(event) => setVibrationMs(numberFromInput(event.target.value, vibrationMs))}
            />
          </label>
          <div className="science-module__actions">
            <button
              className="science-module__button science-module__button--secondary"
              type="button"
              disabled={!connected || pending === 'carriage'}
              onClick={() => moveCarriage(0)}
            >
              Startup
            </button>
            <button
              className="science-module__button"
              type="button"
              disabled={!connected || pending === 'carriage'}
              onClick={() => moveCarriage()}
            >
              Move
            </button>
          </div>
        </div>
      </section>

      <section className="science-module__section">
        <h4>Motor</h4>
        <div className="science-module__form">
          <label className="science-module__field">
            <span>Mode</span>
            <select
              value={motorMode}
              onChange={(event) => setMotorMode(Number(event.target.value))}
            >
              <option value={1}>Velocity (um/s)</option>
              <option value={2}>Position (cm)</option>
            </select>
          </label>
          <label className="science-module__field">
            <span>{motorMode === 2 ? 'Position (cm)' : 'Velocity (um/s)'}</span>
            <input
              type="number"
              step={motorMode === 2 ? 0.1 : 100}
              value={motorValue}
              onChange={(event) => setMotorValue(numberFromInput(event.target.value, motorValue))}
            />
          </label>
          <div className="science-module__actions">
            <button
              className="science-module__button science-module__button--secondary"
              type="button"
              disabled={!connected || pending === 'carriage-motor'}
              onClick={() => sendMotor(3)}
            >
              Home
            </button>
            <button
              className="science-module__button science-module__button--secondary"
              type="button"
              disabled={!connected || pending === 'carriage-motor'}
              onClick={() => sendMotor(4)}
            >
              FOC calibrate
            </button>
            <button
              className="science-module__button"
              type="button"
              disabled={!connected || pending === 'carriage-motor'}
              onClick={() => sendMotor()}
            >
              Send
            </button>
          </div>
        </div>
      </section>

      <div className="science-module__stats">
        <div>
          <span>Position</span>
          <strong>{formatCm(telemetry?.position_um)}</strong>
        </div>
        <div>
          <span>Velocity</span>
          <strong>{formatSigned(telemetry?.velocity_um_s, ' um/s')}</strong>
        </div>
      </div>
      <CommandStatus status={status} />
    </article>
  )
}

export const ScienceDrillCard = () => {
  const { callService, connected, pending, status } = useScienceService()
  const [velocityRadS, setVelocityRadS] = useState(0)
  const [positionRad, setPositionRad] = useState(0)

  const setVelocity = (value = velocityRadS) => {
    void callService<MotorVelocityRequest>(
      'drill-velocity',
      DRILL_VELOCITY_SERVICE,
      MOTOR_VELOCITY_TYPE,
      { rad_s: value }
    )
  }

  const setPosition = () => {
    void callService<MotorPositionRequest>(
      'drill-position',
      DRILL_POSITION_SERVICE,
      MOTOR_POSITION_TYPE,
      { rad: positionRad }
    )
  }

  return (
    <article className="card science-module-card">
      <header className="science-module__header">
        <div>
          <h3>Drill Motor</h3>
          <p>Direct drill motor velocity and position targets.</p>
        </div>
        <span className={`pill ${connected ? 'pill--on' : 'pill--off'}`}>
          {connected ? 'ROS online' : 'ROS offline'}
        </span>
      </header>

      <div className="science-module__form">
        <label className="science-module__field">
          <span>Velocity (rad/s)</span>
          <input
            type="number"
            step={0.1}
            value={velocityRadS}
            onChange={(event) => setVelocityRadS(numberFromInput(event.target.value, velocityRadS))}
          />
        </label>
        <div className="science-module__actions">
          <button
            className="science-module__button science-module__button--secondary"
            type="button"
            disabled={!connected || pending === 'drill-velocity'}
            onClick={() => setVelocity(0)}
          >
            Stop
          </button>
          <button
            className="science-module__button"
            type="button"
            disabled={!connected || pending === 'drill-velocity'}
            onClick={() => setVelocity()}
          >
            Set velocity
          </button>
        </div>
        <label className="science-module__field">
          <span>Position (rad)</span>
          <input
            type="number"
            step={0.1}
            value={positionRad}
            onChange={(event) => setPositionRad(numberFromInput(event.target.value, positionRad))}
          />
        </label>
        <button
          className="science-module__button"
          type="button"
          disabled={!connected || pending === 'drill-position'}
          onClick={setPosition}
        >
          Set position
        </button>
      </div>
      <CommandStatus status={status} />
    </article>
  )
}
