import { useEffect, useMemo, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'

type LastSeenKey =
  | 'odomLocal'
  | 'gpsFiltered'
  | 'traversability'
  | 'cmdVel'
  | 'missionStatus'

type HealthTone = 'good' | 'warn' | 'error'

const WARN_AFTER_MS = 1000
const ERROR_AFTER_MS = 4000

const toneForAge = (ageMs: number): HealthTone => {
  if (ageMs <= WARN_AFTER_MS) return 'good'
  if (ageMs <= ERROR_AFTER_MS) return 'warn'
  return 'error'
}

const labelForTone = (tone: HealthTone) => {
  if (tone === 'good') return 'OK'
  if (tone === 'warn') return 'Late'
  return 'Missing'
}

const AutonomyHealthCard = () => {
  const { ros } = useRosBridge()
  const [lastSeen, setLastSeen] = useState<Record<LastSeenKey, number | null>>({
    odomLocal: null,
    gpsFiltered: null,
    traversability: null,
    cmdVel: null,
    missionStatus: null,
  })
  const [nowMs, setNowMs] = useState(() => Date.now())

  useEffect(() => {
    const timer = window.setInterval(() => {
      setNowMs(Date.now())
    }, 1000)
    return () => window.clearInterval(timer)
  }, [])

  useEffect(() => {
    const markSeen = (key: LastSeenKey) => {
      setLastSeen((prev) => ({
        ...prev,
        [key]: Date.now(),
      }))
    }
    const unsubscribers = [
      ros.subscribe<Record<string, unknown>>(
        '/odometry/filtered/local',
        'nav_msgs/msg/Odometry',
        () => markSeen('odomLocal'),
        { throttleRate: 250 }
      ),
      ros.subscribe<Record<string, unknown>>(
        '/gps/filtered',
        'sensor_msgs/msg/NavSatFix',
        () => markSeen('gpsFiltered'),
        { throttleRate: 250 }
      ),
      ros.subscribe<Record<string, unknown>>(
        '/traversability_gridmap',
        'grid_map_msgs/msg/GridMap',
        () => markSeen('traversability'),
        { throttleRate: 250 }
      ),
      ros.subscribe<Record<string, unknown>>(
        '/cmd_vel',
        'geometry_msgs/msg/Twist',
        () => markSeen('cmdVel'),
        { throttleRate: 250 }
      ),
      ros.subscribe<Record<string, unknown>>(
        '/mission_status',
        'mr2_action_interface/msg/MissionStatus',
        () => markSeen('missionStatus'),
        { throttleRate: 250 }
      ),
    ]
    return () => {
      for (const off of unsubscribers) {
        off()
      }
    }
  }, [ros])

  const statuses = useMemo(() => {
    const ageMs = (ts: number | null) =>
      ts == null ? Number.POSITIVE_INFINITY : nowMs - ts

    const localizationAge = Math.max(
      ageMs(lastSeen.odomLocal),
      ageMs(lastSeen.gpsFiltered)
    )

    const items = [
      { key: 'Localization', age: localizationAge },
      { key: 'Traversability', age: ageMs(lastSeen.traversability) },
      { key: 'Controller', age: ageMs(lastSeen.cmdVel) },
      { key: 'Master Status', age: ageMs(lastSeen.missionStatus) },
    ]

    return items.map((item) => {
      const tone = toneForAge(item.age)
      return {
        label: item.key,
        tone,
        value: labelForTone(tone),
      }
    })
  }, [lastSeen, nowMs])

  return (
    <article className="card">
      <h3>Autonomy Health</h3>
      <p>Pipeline freshness against 1s / 4s thresholds.</p>
      <div className="autonomy-health-table">
        <div className="autonomy-health-row autonomy-health-header">
          {statuses.map((item) => (
            <span key={`${item.label}-header`}>{item.label}</span>
          ))}
        </div>
        <div className="autonomy-health-row">
          {statuses.map((item) => (
            <span
              key={item.label}
              className={`autonomy-health-cell autonomy-health-${item.tone}`}
            >
              {item.value}
            </span>
          ))}
        </div>
      </div>
    </article>
  )
}

export default AutonomyHealthCard
