import { useEffect, useState } from 'react'
import { getRosBridgeClient } from '../../lib/rosBridge'
import { type LinkStatus, type TelemBattery, getSikGatewayClient } from '../../lib/sikGateway'
import './ControlStatusList.css'

type MissionStatusMsg = {
  state?: number
  detail?: string
  arrival?: boolean
}

const MISSION_STATE_LABELS: Record<number, string> = {
  0: 'IDLE',
  1: 'RUNNING',
  2: 'PAUSED',
  3: 'COMPLETED',
  4: 'FAILED',
}

const ControlStatusList = () => {
  const [linkStatus, setLinkStatus] = useState<LinkStatus | null>(null)
  const [wsConnected, setWsConnected] = useState(false)
  const [rosConnected, setRosConnected] = useState(false)
  const [missionStatus, setMissionStatus] = useState<MissionStatusMsg | null>(null)
  const [missionStatusAt, setMissionStatusAt] = useState<number | null>(null)
  const [battery1, setBattery1] = useState<TelemBattery | null>(null)
  const [battery2, setBattery2] = useState<TelemBattery | null>(null)
  const [battery1UpdatedAt, setBattery1UpdatedAt] = useState(0)
  const [battery2UpdatedAt, setBattery2UpdatedAt] = useState(0)
  const [nowMs, setNowMs] = useState(() => Date.now())

  useEffect(() => {
    const gateway = getSikGatewayClient()
    gateway.connect()
    const offLink = gateway.onLinkStatus(setLinkStatus)
    const offConnection = gateway.onConnectionStatus(setWsConnected)
    const offBattery = gateway.onTelemBattery((payload) => {
      if (payload.battery_id === 2) {
        setBattery2(payload)
        setBattery2UpdatedAt(Date.now())
      } else {
        setBattery1(payload)
        setBattery1UpdatedAt(Date.now())
      }
    })
    return () => {
      offLink()
      offConnection()
      offBattery()
    }
  }, [])

  useEffect(() => {
    const interval = window.setInterval(() => {
      setNowMs(Date.now())
    }, 1000)
    return () => {
      window.clearInterval(interval)
    }
  }, [])

  // Clear stale link status whenever the websocket reconnects/disconnects
  useEffect(() => {
    setLinkStatus(null)
  }, [wsConnected])

  useEffect(() => {
    const rosBridge = getRosBridgeClient()
    rosBridge.connect()
    const offRosConnection = rosBridge.onConnectionStatus(setRosConnected)
    return () => {
      offRosConnection()
    }
  }, [])

  useEffect(() => {
    const rosBridge = getRosBridgeClient()
    rosBridge.connect()
    const offMission = rosBridge.subscribe<MissionStatusMsg>(
      '/mission_status',
      'mr2_action_interface/msg/MissionStatus',
      (msg) => {
        setMissionStatus(msg)
        setMissionStatusAt(Date.now())
      },
      { throttleRate: 500 }
    )
    return () => {
      offMission()
    }
  }, [])

  const getBatteryPercent = (battery: TelemBattery | null) => {
    if (!battery || battery.total_capacity_mah <= 0) {
      return 0
    }
    return Math.max(
      0,
      Math.min(100, (battery.available_capacity_mah / battery.total_capacity_mah) * 100)
    )
  }
  const battery1Percent = getBatteryPercent(battery1)
  const battery2Percent = getBatteryPercent(battery2)
  // Battery telemetry is ~11.4s in real mode; keep stale threshold above that.
  const batteryStaleMs = 30000
  const battery1Stale = battery1UpdatedAt === 0 || nowMs - battery1UpdatedAt > batteryStaleMs
  const battery2Stale = battery2UpdatedAt === 0 || nowMs - battery2UpdatedAt > batteryStaleMs
  let linkState = 'Down'
  let linkDotClass = 'status-dot-error'
  if (wsConnected) {
    if (linkStatus?.connected) {
      linkState = 'Up'
      linkDotClass = ''
    } else if (linkStatus && linkStatus.connected === false) {
      linkState = 'Lost'
      linkDotClass = 'status-dot-warn'
    } else {
      // WebSocket is up but no link_status message yet (e.g., node just restarted)
      linkState = 'Down'
      linkDotClass = 'status-dot-error'
    }
  }
  const rosState = rosConnected ? 'Up' : 'Down'
  const rosDotClass = rosConnected ? '' : 'status-dot-error'

  const missionStaleMs = 1000
  const missionStale = missionStatusAt == null || nowMs - missionStatusAt > missionStaleMs
  const ledMode = (() => {
    if (missionStale || !missionStatus) return 'off'
    if (missionStatus.arrival) return 'success'
    if (missionStatus.state === 1) return 'autonomous'
    if (missionStatus.state === 2) return 'manual'
    return 'off'
  })()
  const missionLabel = missionStatus?.arrival
    ? 'ARRIVAL'
    : missionStatus?.state != null
      ? MISSION_STATE_LABELS[missionStatus.state] ?? 'UNKNOWN'
      : '—'
  const missionDotClass =
    ledMode === 'success'
      ? 'status-dot-success status-dot-flash'
      : ledMode === 'autonomous'
        ? 'status-dot-autonomous'
        : ledMode === 'manual'
          ? 'status-dot-manual'
          : 'status-dot-off'

  return (
    <>
      <section className="panel-section">
        <div className="status-list">
          <div className="status-item status-link">
            <span className="status-label">
              <span className={`status-dot ${linkDotClass}`} aria-hidden="true" />
              SiK Link
            </span>
            <div className="status-pill">
              <strong>{linkState}</strong>
            </div>
          </div>
          <div className="status-item status-link">
            <span className="status-label">
              <span className={`status-dot ${rosDotClass}`} aria-hidden="true" />
              ROS Bridge
            </span>
            <div className="status-pill">
              <strong>{rosState}</strong>
            </div>
          </div>
          <div className="status-item status-softstop">
            <span className="status-label">
              <span className={`status-dot ${missionDotClass}`} aria-hidden="true" />
              Auto Mission
            </span>
            <div className="status-pill">
              <strong>{missionLabel}</strong>
            </div>
          </div>
        </div>
      </section>
      <section className="panel-section">
        <div className="status-list">
          <div className={`status-item battery${battery1Stale ? ' battery-stale' : ''}`}>
            <div className="metric-label">
              <span>Battery 1</span>
              <strong>{battery1 ? `${battery1Percent.toFixed(0)}%` : '---'}</strong>
            </div>
            <div className="meter">
              <div className="meter-fill good" style={{ width: `${battery1Percent}%` }} />
            </div>
            <div className="battery-meta">
              <span>
                {battery1 ? `${battery1.temperature_c.toFixed(1)}°C` : '--°C'} ·{' '}
                {battery1 ? `${battery1.pack_voltage_v.toFixed(1)}V` : '--V'} ·{' '}
                {battery1 ? `${(battery1.total_capacity_mah / 1000).toFixed(1)}Ah` : '--Ah'}
              </span>
            </div>
          </div>
          <div className={`status-item battery${battery2Stale ? ' battery-stale' : ''}`}>
            <div className="metric-label">
              <span>Battery 2</span>
              <strong>{battery2 ? `${battery2Percent.toFixed(0)}%` : '---'}</strong>
            </div>
            <div className="meter">
              <div className="meter-fill accent" style={{ width: `${battery2Percent}%` }} />
            </div>
            <div className="battery-meta">
              <span>
                {battery2 ? `${battery2.temperature_c.toFixed(1)}°C` : '--°C'} ·{' '}
                {battery2 ? `${battery2.pack_voltage_v.toFixed(1)}V` : '--V'} ·{' '}
                {battery2 ? `${(battery2.total_capacity_mah / 1000).toFixed(1)}Ah` : '--Ah'}
              </span>
            </div>
          </div>
        </div>
      </section>
    </>
  )
}

export default ControlStatusList
