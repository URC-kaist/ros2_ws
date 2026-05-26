import { useCallback, useEffect, useState } from 'react'
import {
  FiActivity,
  FiDroplet,
  FiHome,
  FiPower,
  FiRefreshCw,
  FiSend,
  FiSliders,
  FiSun,
} from 'react-icons/fi'
import { useRosBridge } from '../hooks/useRosBridge'
import type {
  CarriageMotorCommandRequest,
  CarriageMotorTelemetryMsg,
  MotorPositionRequest,
  MotorVelocityRequest,
  MoveCarriageRequest,
  SciencePumpRequest,
  ScienceServiceResponse,
  SelectCentrifugePositionRequest,
  SetScienceLedRequest,
  TriggerCentrifugeRampRequest,
} from '../lib/rosMessages'

const SERVICE_TYPES = {
  pump: 'mr2_science_module/srv/Pump',
  led: 'mr2_science_module/srv/SetScienceLed',
  centrifugePosition: 'mr2_science_module/srv/SelectCentrifugePosition',
  centrifugeRamp: 'mr2_science_module/srv/TriggerCentrifugeRamp',
  carriage: 'mr2_science_module/srv/MoveCarriage',
  carriageMotor: 'mr2_science_module/srv/CarriageMotorCommand',
  motorVelocity: 'mr2_science_module/srv/MotorVelocity',
  motorPosition: 'mr2_science_module/srv/MotorPosition',
} as const

const SERVICES = {
  pump: '/science/pump',
  led: '/science/led',
  centrifugePosition: '/science/centrifuge_position',
  centrifugeRamp: '/science/centrifuge_ramp',
  carriage: '/science/carriage',
  carriageMotor: '/science/carriage_motor',
  drillVelocity: '/science/drill_velocity',
  drillPosition: '/science/drill_position',
  debugCentrifugeVelocity: '/science/debug_centrifuge_motor_velocity',
  debugCentrifugePosition: '/science/debug_centrifuge_motor_position',
} as const

const clamp = (value: number, min: number, max: number) => {
  if (!Number.isFinite(value)) return min
  return Math.min(max, Math.max(min, Math.round(value)))
}

const positionOptions = [
  '0',
  'pi / 4',
  'pi / 2',
  '3pi / 4',
  'pi',
  '5pi / 4',
  '3pi / 2',
  '7pi / 4',
]

const carriageCommands = [
  { value: 0, label: 'Startup' },
  { value: 1, label: 'Funnel 1' },
  { value: 2, label: 'Funnel 2' },
  { value: 3, label: 'Funnel 3' },
  { value: 4, label: 'Funnel 4' },
  { value: 5, label: 'Funnel 5' },
  { value: 6, label: 'Funnel 6' },
]

const carriageMotorModes = [
  { value: 1, label: 'Velocity um/s' },
  { value: 2, label: 'Position um' },
  { value: 3, label: 'Home' },
  { value: 4, label: 'FOC calibration' },
]

const formatTelemetry = (value?: number, unit?: string) => {
  if (typeof value !== 'number') return '--'
  return `${value.toLocaleString()}${unit ? ` ${unit}` : ''}`
}

const CentrifugeCard = () => {
  const { ros, connected: rosConnected } = useRosBridge()
  const [busyAction, setBusyAction] = useState<string | null>(null)
  const [status, setStatus] = useState('Ready')
  const [pumpDurationMs, setPumpDurationMs] = useState(0)
  const [ledBrightness, setLedBrightness] = useState(128)
  const [centrifugePosition, setCentrifugePosition] = useState(0)
  const [carriageCommand, setCarriageCommand] = useState(0)
  const [carriageVibrationMs, setCarriageVibrationMs] = useState(1000)
  const [carriageMotorMode, setCarriageMotorMode] = useState(3)
  const [carriageMotorValue, setCarriageMotorValue] = useState(0)
  const [drillVelocityRadS, setDrillVelocityRadS] = useState(30)
  const [drillPositionRad, setDrillPositionRad] = useState(0)
  const [debugCentrifugeVelocityRadS, setDebugCentrifugeVelocityRadS] = useState(0)
  const [debugCentrifugePositionRad, setDebugCentrifugePositionRad] = useState(0)
  const [telemetry, setTelemetry] = useState<CarriageMotorTelemetryMsg | null>(null)
  const isBusy = busyAction !== null
  const carriageMotorNeedsValue = carriageMotorMode === 1 || carriageMotorMode === 2

  useEffect(() => {
    return ros.subscribe<CarriageMotorTelemetryMsg>(
      '/science/carriage_motor/telemetry',
      'mr2_science_module/msg/CarriageMotorTelemetry',
      setTelemetry,
      { throttleRate: 100 }
    )
  }, [ros])

  const callScienceService = useCallback(
    async <TRequest,>(name: string, serviceType: string, request: TRequest, label: string) => {
      if (!ros.isConnected()) {
        setStatus('ROS bridge is offline')
        return
      }

      setBusyAction(label)
      setStatus(`${label}...`)
      try {
        const response = await ros.callService<TRequest, ScienceServiceResponse>(
          name,
          serviceType,
          request
        )
        if (response.success) {
          setStatus(`${label}: sent`)
        } else {
          setStatus(`${label}: ${response.message || 'failed'}`)
        }
      } catch (err) {
        setStatus(`${label}: ${err instanceof Error ? err.message : 'service call failed'}`)
      } finally {
        setBusyAction(null)
      }
    },
    [ros]
  )

  const sendPump = (pump: number) => {
    callScienceService<SciencePumpRequest>(
      SERVICES.pump,
      SERVICE_TYPES.pump,
      {
        pump,
        duration_ms: clamp(pumpDurationMs, 0, 65535),
      },
      `Pump ${pump}`
    )
  }

  const setLed = (enabled: boolean) => {
    callScienceService<SetScienceLedRequest>(
      SERVICES.led,
      SERVICE_TYPES.led,
      {
        enabled,
        brightness: clamp(ledBrightness, 0, 255),
      },
      enabled ? 'Set LED' : 'LED off'
    )
  }

  const selectCentrifugePosition = () => {
    callScienceService<SelectCentrifugePositionRequest>(
      SERVICES.centrifugePosition,
      SERVICE_TYPES.centrifugePosition,
      { index: clamp(centrifugePosition, 0, 7) },
      'Centrifuge position'
    )
  }

  const startCentrifugeRamp = () => {
    callScienceService<TriggerCentrifugeRampRequest>(
      SERVICES.centrifugeRamp,
      SERVICE_TYPES.centrifugeRamp,
      { start: true },
      'Centrifuge ramp'
    )
  }

  const moveCarriage = () => {
    callScienceService<MoveCarriageRequest>(
      SERVICES.carriage,
      SERVICE_TYPES.carriage,
      {
        command: clamp(carriageCommand, 0, 6),
        vibration_duration_ms: clamp(carriageVibrationMs, 0, 65535),
      },
      'Move carriage'
    )
  }

  const sendCarriageMotor = () => {
    callScienceService<CarriageMotorCommandRequest>(
      SERVICES.carriageMotor,
      SERVICE_TYPES.carriageMotor,
      {
        mode: clamp(carriageMotorMode, 1, 4),
        value: carriageMotorNeedsValue ? clamp(carriageMotorValue, -2147483648, 2147483647) : 0,
      },
      'Carriage motor'
    )
  }

  const sendMotorVelocity = (name: string, radS: number, label: string) => {
    callScienceService<MotorVelocityRequest>(
      name,
      SERVICE_TYPES.motorVelocity,
      { rad_s: Number.isFinite(radS) ? radS : 0 },
      label
    )
  }

  const sendMotorPosition = (name: string, rad: number, label: string) => {
    callScienceService<MotorPositionRequest>(
      name,
      SERVICE_TYPES.motorPosition,
      { rad: Number.isFinite(rad) ? rad : 0 },
      label
    )
  }

  return (
    <article className="card centrifuge-card">
      <header className="centrifuge-card__header">
        <div>
          <h3>Science Module</h3>
          <p>CAN-backed controls for pumps, lighting, carriage, centrifuge, and drill.</p>
        </div>
        <span className="pill">{rosConnected ? 'ros online' : 'ros offline'}</span>
      </header>

      <div className="centrifuge-status" role="status">
        {isBusy ? busyAction : status}
      </div>

      <section className="centrifuge-section">
        <h4>Pumps</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Duration ms</span>
            <input
              type="number"
              value={pumpDurationMs}
              min={0}
              max={65535}
              step={100}
              onChange={(event) => setPumpDurationMs(Number(event.target.value))}
            />
          </label>
          <div className="centrifuge-actions">
            {[1, 2, 3, 4].map((pump) => (
              <button
                key={pump}
                className="centrifuge-button"
                type="button"
                disabled={isBusy || !rosConnected}
                onClick={() => sendPump(pump)}
              >
                <FiDroplet aria-hidden="true" />
                Pump {pump}
              </button>
            ))}
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>LED</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Brightness</span>
            <input
              type="range"
              value={ledBrightness}
              min={0}
              max={255}
              onChange={(event) => setLedBrightness(Number(event.target.value))}
            />
          </label>
          <label className="centrifuge-field">
            <span>PWM</span>
            <input
              type="number"
              value={ledBrightness}
              min={0}
              max={255}
              onChange={(event) => setLedBrightness(Number(event.target.value))}
            />
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() => setLed(true)}
            >
              <FiSun aria-hidden="true" />
              Set
            </button>
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() => setLed(false)}
            >
              <FiPower aria-hidden="true" />
              Off
            </button>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Centrifuge</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Indexed position</span>
            <select
              value={centrifugePosition}
              onChange={(event) => setCentrifugePosition(Number(event.target.value))}
            >
              {positionOptions.map((label, index) => (
                <option key={label} value={index}>
                  {index}: {label}
                </option>
              ))}
            </select>
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={selectCentrifugePosition}
            >
              <FiSend aria-hidden="true" />
              Position
            </button>
            <button
              className="centrifuge-button"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={startCentrifugeRamp}
            >
              <FiRefreshCw aria-hidden="true" />
              Ramp
            </button>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Carriage Module</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Command</span>
            <select
              value={carriageCommand}
              onChange={(event) => setCarriageCommand(Number(event.target.value))}
            >
              {carriageCommands.map((command) => (
                <option key={command.value} value={command.value}>
                  {command.label}
                </option>
              ))}
            </select>
          </label>
          <label className="centrifuge-field">
            <span>Vibration ms</span>
            <input
              type="number"
              value={carriageVibrationMs}
              min={0}
              max={65535}
              step={100}
              disabled={carriageCommand === 0}
              onChange={(event) => setCarriageVibrationMs(Number(event.target.value))}
            />
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={moveCarriage}
            >
              <FiSend aria-hidden="true" />
              Send
            </button>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Carriage Motor</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Mode</span>
            <select
              value={carriageMotorMode}
              onChange={(event) => setCarriageMotorMode(Number(event.target.value))}
            >
              {carriageMotorModes.map((mode) => (
                <option key={mode.value} value={mode.value}>
                  {mode.label}
                </option>
              ))}
            </select>
          </label>
          <label className="centrifuge-field">
            <span>Value</span>
            <input
              type="number"
              value={carriageMotorValue}
              step={1000}
              disabled={!carriageMotorNeedsValue}
              onChange={(event) => setCarriageMotorValue(Number(event.target.value))}
            />
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={sendCarriageMotor}
            >
              {carriageMotorMode === 3 ? (
                <FiHome aria-hidden="true" />
              ) : (
                <FiSliders aria-hidden="true" />
              )}
              Send
            </button>
          </div>
        </div>
        <div className="centrifuge-stats">
          <div className="centrifuge-stat">
            <span className="centrifuge-stat__label">Position</span>
            <span className="centrifuge-stat__value">
              {formatTelemetry(telemetry?.position_um, 'um')}
            </span>
          </div>
          <div className="centrifuge-stat">
            <span className="centrifuge-stat__label">Velocity</span>
            <span className="centrifuge-stat__value">
              {formatTelemetry(telemetry?.velocity_um_s, 'um/s')}
            </span>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Drill</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Velocity rad/s</span>
            <input
              type="number"
              value={drillVelocityRadS}
              step={1}
              onChange={(event) => setDrillVelocityRadS(Number(event.target.value))}
            />
          </label>
          <label className="centrifuge-field">
            <span>Position rad</span>
            <input
              type="number"
              value={drillPositionRad}
              step={0.1}
              onChange={(event) => setDrillPositionRad(Number(event.target.value))}
            />
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() =>
                sendMotorVelocity(SERVICES.drillVelocity, drillVelocityRadS, 'Drill velocity')
              }
            >
              <FiActivity aria-hidden="true" />
              Velocity
            </button>
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() => sendMotorVelocity(SERVICES.drillVelocity, 0, 'Stop drill')}
            >
              <FiPower aria-hidden="true" />
              Stop
            </button>
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() =>
                sendMotorPosition(SERVICES.drillPosition, drillPositionRad, 'Drill position')
              }
            >
              <FiSend aria-hidden="true" />
              Position
            </button>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Centrifuge Motor Debug</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Velocity rad/s</span>
            <input
              type="number"
              value={debugCentrifugeVelocityRadS}
              step={1}
              onChange={(event) => setDebugCentrifugeVelocityRadS(Number(event.target.value))}
            />
          </label>
          <label className="centrifuge-field">
            <span>Position rad</span>
            <input
              type="number"
              value={debugCentrifugePositionRad}
              step={0.1}
              onChange={(event) => setDebugCentrifugePositionRad(Number(event.target.value))}
            />
          </label>
          <div className="centrifuge-actions">
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() =>
                sendMotorVelocity(
                  SERVICES.debugCentrifugeVelocity,
                  debugCentrifugeVelocityRadS,
                  'Debug centrifuge velocity'
                )
              }
            >
              <FiActivity aria-hidden="true" />
              Velocity
            </button>
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() =>
                sendMotorVelocity(
                  SERVICES.debugCentrifugeVelocity,
                  0,
                  'Stop debug centrifuge'
                )
              }
            >
              <FiPower aria-hidden="true" />
              Stop
            </button>
            <button
              className="centrifuge-button centrifuge-button--ghost"
              type="button"
              disabled={isBusy || !rosConnected}
              onClick={() =>
                sendMotorPosition(
                  SERVICES.debugCentrifugePosition,
                  debugCentrifugePositionRad,
                  'Debug centrifuge position'
                )
              }
            >
              <FiSend aria-hidden="true" />
              Position
            </button>
          </div>
        </div>
      </section>
    </article>
  )
}

export default CentrifugeCard
