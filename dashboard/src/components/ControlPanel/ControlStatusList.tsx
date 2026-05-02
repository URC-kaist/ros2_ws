import { useEffect, useMemo, useState } from 'react'
import { useRosBridge } from '../../hooks/useRosBridge'
import { useXbeeGateway } from '../../hooks/useXbeeGateway'
import type { MissionStatusMsg, UBXNavStatus } from '../../lib/rosMessages'
import { type LinkStatus, type TelemBattery } from '../../lib/xbeeGateway'
import './ControlStatusList.css'

type GnssSideId = 'left' | 'right'

const MISSION_STATE_LABELS: Record<number, string> = {
  0: 'IDLE',
  1: 'RUNNING',
  2: 'PAUSED',
  3: 'COMPLETED',
  4: 'FAILED',
}

const GNSS_SIDES: Array<{ id: GnssSideId; label: string; ns: string }> = [
  { id: 'left', label: 'Left GNSS', ns: '/left_gnss' },
  { id: 'right', label: 'Right GNSS', ns: '/right_gnss' },
]

const formatFixType = (fixType?: number) => {
  if (typeof fixType !== 'number' || !Number.isFinite(fixType)) return '--'
  switch (fixType) {
    case 0:
      return 'No Fix'
    case 1:
      return 'Dead Reckoning'
    case 2:
      return '2D'
    case 3:
      return '3D'
    case 4:
      return '3D + DR'
    case 5:
      return 'Time Only'
    default:
      return `${fixType}`
  }
}

const formatFixOk = (fixOk?: boolean) => {
  if (fixOk == null) return '--'
  return fixOk ? 'yes' : 'no'
}

const ControlStatusList = () => {
  const { ros: rosBridge, connected: rosConnected } = useRosBridge()
  const { gateway, connected: wsConnected } = useXbeeGateway()
  const [linkStatus, setLinkStatus] = useState<LinkStatus | null>(null)
  const [missionStatus, setMissionStatus] = useState<MissionStatusMsg | null>(null)
  const [missionStatusAt, setMissionStatusAt] = useState<number | null>(null)
  const [battery1, setBattery1] = useState<TelemBattery | null>(null)
  const [battery2, setBattery2] = useState<TelemBattery | null>(null)
  const [battery1UpdatedAt, setBattery1UpdatedAt] = useState(0)
  const [battery2UpdatedAt, setBattery2UpdatedAt] = useState(0)
  const [nowMs, setNowMs] = useState(() => Date.now())
  const [gnssState, setGnssState] = useState<Record<GnssSideId, UBXNavStatus>>({
    left: {},
    right: {},
  })

  useEffect(() => {
    const offLink = gateway.onLinkStatus(setLinkStatus)
    const offConnection = gateway.onConnectionStatus(() => {
      setLinkStatus(null)
    })
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
  }, [gateway])

  useEffect(() => {
    const interval = window.setInterval(() => {
      setNowMs(Date.now())
    }, 1000)
    return () => {
      window.clearInterval(interval)
    }
  }, [])

  useEffect(() => {
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
  }, [rosBridge])

  useEffect(() => {
    const unsubscribers: Array<() => void> = []

    for (const side of GNSS_SIDES) {
      unsubscribers.push(
        rosBridge.subscribe<UBXNavStatus>(
          `${side.ns}/ubx_nav_status`,
          'ublox_ubx_msgs/msg/UBXNavStatus',
          (msg) => {
            if (!msg) return
            setGnssState((prev) => ({
              ...prev,
              [side.id]: msg,
            }))
          },
          { throttleRate: 1000 }
        )
      )
    }

    return () => {
      for (const off of unsubscribers) {
        off()
      }
    }
  }, [rosBridge])

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
  const getBatteryFillClass = (percent: number) => {
    if (percent <= 20) return 'low'
    if (percent <= 50) return 'warn'
    return 'good'
  }
  // Battery telemetry is ~11.4s in real mode; keep stale threshold above that.
  const batteryStaleMs = 30000
  const battery1Stale = battery1UpdatedAt === 0 || nowMs - battery1UpdatedAt > batteryStaleMs
  const battery2Stale = battery2UpdatedAt === 0 || nowMs - battery2UpdatedAt > batteryStaleMs
  let linkState = 'Down'
  let linkDotClass = 'status-dot-error'
  const effectiveLinkStatus = wsConnected ? linkStatus : null
  if (wsConnected) {
    if (effectiveLinkStatus?.connected) {
      linkState = 'Up'
      linkDotClass = ''
    } else if (effectiveLinkStatus && effectiveLinkStatus.connected === false) {
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

  const gnssSummary = useMemo(
    () =>
      GNSS_SIDES.map((side) => {
        const status = gnssState[side.id]
        const fixType = formatFixType(status?.gps_fix?.fix_type)
        const fixOk = formatFixOk(status?.gps_fix_ok)
        const fixOkTone =
          status?.gps_fix_ok == null ? '' : status.gps_fix_ok ? 'status-good' : 'status-bad'
        return {
          id: side.id,
          label: side.label,
          fixType,
          fixOk,
          fixOkTone,
        }
      }),
    [gnssState]
  )

  return (
    <>
      <section className="panel-section">
        <div className="status-grid-compact">
          <div className="status-grid-label">
            <span className={`status-dot ${linkDotClass}`} aria-hidden="true" />
            XBEE Link
          </div>
          <div className="status-grid-label">
            <span className={`status-dot ${rosDotClass}`} aria-hidden="true" />
            ROS Bridge
          </div>
          <div className="status-grid-label">
            <span className={`status-dot ${missionDotClass}`} aria-hidden="true" />
            Auto Mission
          </div>
          <div className="status-grid-value">
            <strong>{linkState}</strong>
          </div>
          <div className="status-grid-value">
            <strong>{rosState}</strong>
          </div>
          <div className="status-grid-value">
            <strong>{missionLabel}</strong>
          </div>
        </div>
      </section>
      <section className="panel-section">
        <div className="sidebar-section-title">GNSS Fix</div>
        <div className="gnss-mini-grid">
          {gnssSummary.map((side) => (
            <div className="gnss-mini-card" key={side.id}>
              <div className="gnss-mini-title">{side.label}</div>
              <div className="gnss-mini-kv">
                <span>Fix</span>
                <strong>{side.fixType}</strong>
              </div>
              <div className="gnss-mini-kv">
                <span>Fix OK</span>
                <strong className={side.fixOkTone}>{side.fixOk}</strong>
              </div>
            </div>
          ))}
        </div>
      </section>
      <section className="panel-section">
        <div className="status-list status-list--batteries">
          <div className={`status-item battery${battery1Stale ? ' battery-stale' : ''}`}>
            <div className="metric-label">
              <strong>{battery1 ? `${battery1Percent.toFixed(0)}%` : '---'}</strong>
            </div>
            <div className="meter">
              <div
                className={`meter-fill ${getBatteryFillClass(battery1Percent)}`}
                style={{ width: `${battery1Percent}%` }}
              />
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
              <strong>{battery2 ? `${battery2Percent.toFixed(0)}%` : '---'}</strong>
            </div>
            <div className="meter">
              <div
                className={`meter-fill ${getBatteryFillClass(battery2Percent)}`}
                style={{ width: `${battery2Percent}%` }}
              />
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
