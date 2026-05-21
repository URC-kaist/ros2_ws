import { useCallback, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'
import type {
  MotorPositionRequest,
  MotorVelocityRequest,
  ScienceServiceResponse,
  SelectCentrifugePositionRequest,
  TriggerCentrifugeRampRequest,
} from '../lib/rosMessages'

const POSITION_SERVICE = '/science/centrifuge_position'
const POSITION_TYPE = 'mr2_science_module/srv/SelectCentrifugePosition'
const RAMP_SERVICE = '/science/centrifuge_ramp'
const RAMP_TYPE = 'mr2_science_module/srv/TriggerCentrifugeRamp'
const DEBUG_VELOCITY_SERVICE = '/science/debug_centrifuge_motor_velocity'
const MOTOR_VELOCITY_TYPE = 'mr2_science_module/srv/MotorVelocity'
const DEBUG_POSITION_SERVICE = '/science/debug_centrifuge_motor_position'
const MOTOR_POSITION_TYPE = 'mr2_science_module/srv/MotorPosition'

type PendingCommand = 'position' | 'ramp' | 'velocity' | 'motor-position' | null
type StatusState = { ok: boolean; text: string } | null

const numberFromInput = (value: string, fallback: number) => {
  const parsed = Number(value)
  return Number.isFinite(parsed) ? parsed : fallback
}

const CentrifugeCard = () => {
  const { ros, connected: rosConnected } = useRosBridge()
  const [positionIndex, setPositionIndex] = useState(0)
  const [velocityRadS, setVelocityRadS] = useState(0)
  const [positionRad, setPositionRad] = useState(0)
  const [pending, setPending] = useState<PendingCommand>(null)
  const [status, setStatus] = useState<StatusState>(null)

  const callService = useCallback(
    async <TRequest,>(
      command: Exclude<PendingCommand, null>,
      service: string,
      serviceType: string,
      request: TRequest
    ) => {
      if (!ros.isConnected()) {
        setStatus({ ok: false, text: 'ROS bridge is offline' })
        return
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
      } catch (error) {
        setStatus({
          ok: false,
          text: error instanceof Error ? error.message : 'Service call failed',
        })
      } finally {
        setPending(null)
      }
    },
    [ros]
  )

  const selectPosition = () => {
    void callService<SelectCentrifugePositionRequest>(
      'position',
      POSITION_SERVICE,
      POSITION_TYPE,
      { index: positionIndex }
    )
  }

  const startRamp = () => {
    void callService<TriggerCentrifugeRampRequest>('ramp', RAMP_SERVICE, RAMP_TYPE, {
      start: true,
    })
  }

  const setVelocity = (value = velocityRadS) => {
    void callService<MotorVelocityRequest>(
      'velocity',
      DEBUG_VELOCITY_SERVICE,
      MOTOR_VELOCITY_TYPE,
      { rad_s: value }
    )
  }

  const setPosition = () => {
    void callService<MotorPositionRequest>(
      'motor-position',
      DEBUG_POSITION_SERVICE,
      MOTOR_POSITION_TYPE,
      { rad: positionRad }
    )
  }

  return (
    <article className="card centrifuge-card">
      <header className="centrifuge-card__header">
        <div>
          <h3>Centrifuge</h3>
          <p>Position selection, ramp trigger, and debug motor targets.</p>
        </div>
        <span className={`pill ${rosConnected ? 'pill--on' : 'pill--off'}`}>
          {rosConnected ? 'ROS online' : 'ROS offline'}
        </span>
      </header>

      <section className="centrifuge-section">
        <h4>Module Commands</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Position index</span>
            <select
              value={positionIndex}
              onChange={(event) => setPositionIndex(Number(event.target.value))}
            >
              {Array.from({ length: 8 }, (_, index) => (
                <option key={index} value={index}>
                  {index} ({index * 45} deg)
                </option>
              ))}
            </select>
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={!rosConnected || pending === 'position'}
              onClick={selectPosition}
            >
              {pending === 'position' ? 'Sending...' : 'Select'}
            </button>
            <button
              className="centrifuge-button"
              type="button"
              disabled={!rosConnected || pending === 'ramp'}
              onClick={startRamp}
            >
              {pending === 'ramp' ? 'Sending...' : 'Start ramp'}
            </button>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Debug Motor</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Velocity (rad/s)</span>
            <input
              type="number"
              step={0.1}
              value={velocityRadS}
              onChange={(event) => setVelocityRadS(numberFromInput(event.target.value, velocityRadS))}
            />
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={!rosConnected || pending === 'velocity'}
              onClick={() => setVelocity(0)}
            >
              Stop
            </button>
            <button
              className="centrifuge-button"
              type="button"
              disabled={!rosConnected || pending === 'velocity'}
              onClick={() => setVelocity()}
            >
              Set velocity
            </button>
          </div>
          <label className="centrifuge-field">
            <span>Position (rad)</span>
            <input
              type="number"
              step={0.1}
              value={positionRad}
              onChange={(event) => setPositionRad(numberFromInput(event.target.value, positionRad))}
            />
          </label>
          <button
            className="centrifuge-button"
            type="button"
            disabled={!rosConnected || pending === 'motor-position'}
            onClick={setPosition}
          >
            Set position
          </button>
        </div>
      </section>

      {status && (
        <div className={`science-module__status ${status.ok ? '' : 'science-module__status--error'}`}>
          {status.text}
        </div>
      )}
    </article>
  )
}

export default CentrifugeCard
