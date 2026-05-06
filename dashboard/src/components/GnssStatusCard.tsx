import { useEffect, useMemo, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'
import type { UBXNavHPPosLLH, UBXNavStatus } from '../lib/rosMessages'
import './GnssStatusCard.css'

type GnssSideId = 'left' | 'right'

type GnssState = {
  navStatus?: UBXNavStatus
  navStatusAt?: number
  hpPos?: UBXNavHPPosLLH
  hpPosAt?: number
}

const GNSS_SIDES: Array<{ id: GnssSideId; label: string; ns: string }> = [
  { id: 'left', label: 'Left GNSS', ns: '/left_gnss' },
  { id: 'right', label: 'Right GNSS', ns: '/right_gnss' },
]

const isFiniteNumber = (value: unknown): value is number =>
  typeof value === 'number' && Number.isFinite(value)

const formatFixType = (fixType?: number) => {
  if (!isFiniteNumber(fixType)) return '--'
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

const formatCorrections = (status?: UBXNavStatus) => {
  if (!status) return '--'
  if (status.diff_soln) return 'Applied'
  if (status.diff_corr) return 'Available'
  return 'None'
}

const rtkStatus = (status?: UBXNavStatus) => {
  if (!status) return '--'
  const carr = status.carr_soln?.status
  if (status.carr_soln_valid && carr === 2) return 'RTK Fix'
  if (status.carr_soln_valid && carr === 1) return 'RTK Float'
  if (status.diff_soln) return 'DGNSS'
  if (status.gps_fix_ok) return 'Standalone'
  return 'No Fix'
}

const rtkTone = (label: string) => {
  if (label === 'RTK Fix') return 'good'
  if (label === 'RTK Float' || label === 'DGNSS') return 'warn'
  if (label === 'Standalone' || label === '--') return 'neutral'
  return 'bad'
}

const formatAccuracy = (value?: number) => {
  if (!isFiniteNumber(value)) return '--'
  const mm = value * 0.1
  if (mm >= 1000) return `${(mm / 1000).toFixed(2)} m`
  if (mm >= 100) return `${(mm / 10).toFixed(1)} cm`
  return `${mm.toFixed(1)} mm`
}

const formatAccuracyPair = (pos?: UBXNavHPPosLLH) => {
  if (!pos) return '--'
  const h = formatAccuracy(pos.h_acc)
  const v = formatAccuracy(pos.v_acc)
  if (h === '--' && v === '--') return '--'
  return `${h} / ${v}`
}

const formatUpdatedAt = (ts?: number) =>
  ts != null ? new Date(ts).toLocaleTimeString() : '—'

const GnssStatusCard = () => {
  const { ros } = useRosBridge()
  const [gnssState, setGnssState] = useState<Record<GnssSideId, GnssState>>({
    left: {},
    right: {},
  })

  useEffect(() => {
    const unsubscribers: Array<() => void> = []

    for (const side of GNSS_SIDES) {
      unsubscribers.push(
        ros.subscribe<UBXNavStatus>(
          `${side.ns}/ubx_nav_status`,
          'ublox_ubx_msgs/msg/UBXNavStatus',
          (msg) => {
            if (!msg) return
            setGnssState((prev) => ({
              ...prev,
              [side.id]: {
                ...prev[side.id],
                navStatus: msg,
                navStatusAt: Date.now(),
              },
            }))
          },
          { throttleRate: 1000 }
        )
      )
      unsubscribers.push(
        ros.subscribe<UBXNavHPPosLLH>(
          `${side.ns}/ubx_nav_hp_pos_llh`,
          'ublox_ubx_msgs/msg/UBXNavHPPosLLH',
          (msg) => {
            if (!msg) return
            setGnssState((prev) => ({
              ...prev,
              [side.id]: {
                ...prev[side.id],
                hpPos: msg,
                hpPosAt: Date.now(),
              },
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
  }, [ros])

  const sides = useMemo(
    () =>
      GNSS_SIDES.map((side) => {
        const state = gnssState[side.id] ?? {}
        const navStatus = state.navStatus
        const hpPos = state.hpPos
        const statusLabel = rtkStatus(navStatus)
        const lastUpdated =
          Math.max(state.navStatusAt ?? 0, state.hpPosAt ?? 0) || undefined

        return {
          id: side.id,
          label: side.label,
          statusLabel,
          statusTone: rtkTone(statusLabel),
          rows: [
            { key: 'Fix type', value: formatFixType(navStatus?.gps_fix?.fix_type) },
            { key: 'Fix OK', value: navStatus ? (navStatus.gps_fix_ok ? 'yes' : 'no') : '--' },
            { key: 'Corrections', value: formatCorrections(navStatus) },
            { key: 'Accuracy (H/V)', value: formatAccuracyPair(hpPos) },
          ],
          updatedAt: lastUpdated,
        }
      }),
    [gnssState]
  )

  return (
    <>
      {sides.map((side) => (
        <article className="card gnss-card" key={side.id}>
          <div className="gnss-card__header">
            <h3>{side.label}</h3>
            <span className={`gnss-pill gnss-pill--${side.statusTone}`}>
              {side.statusLabel}
            </span>
          </div>
          <div className="gnss-card__kv">
            {side.rows.map((row) => (
              <div className="gnss-card__row" key={row.key}>
                <span>{row.key}</span>
                <strong>{row.value}</strong>
              </div>
            ))}
          </div>
          <div className="gnss-updated">Updated {formatUpdatedAt(side.updatedAt)}</div>
        </article>
      ))}
    </>
  )
}

export default GnssStatusCard
