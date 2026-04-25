import { useEffect, useMemo, useState } from 'react'
import { useRosBridge } from './useRosBridge'
import { useXbeeGateway } from './useXbeeGateway'
import type { DiagnosticArray, PackTelemetry } from '../lib/rosMessages'
import type { BaseStatus } from '../lib/xbeeGateway'
import {
  BATTERY_SPECS,
  BATTERY_TOPICS,
  buildBaseValues,
  buildBatteryValues,
  SYSTEM_STATUS_TOPICS,
  type BatteryCardId,
  type StatusCard,
  type StatusSnapshot,
} from '../lib/systemStatus'

export const useSystemStatusCards = () => {
  const { ros } = useRosBridge()
  const { gateway } = useXbeeGateway()
  const [snapshots, setSnapshots] = useState<Record<string, StatusSnapshot | null>>({})
  const [baseStatus, setBaseStatus] = useState<BaseStatus | null>(null)
  const [baseStatusUpdatedAt, setBaseStatusUpdatedAt] = useState<number | null>(null)
  const [batterySnapshots, setBatterySnapshots] = useState<
    Record<BatteryCardId, StatusSnapshot | null>
  >({
    battery_1: null,
    battery_2: null,
  })
  const [baseHeadingInput, setBaseHeadingInput] = useState(() => {
    if (typeof window === 'undefined') return ''
    return window.localStorage.getItem('baseHeadingDeg') ?? ''
  })

  useEffect(() => {
    const unsubscribers = SYSTEM_STATUS_TOPICS.map((spec) =>
      ros.subscribe<DiagnosticArray>(
        spec.topic,
        'diagnostic_msgs/DiagnosticArray',
        (msg) => {
          const status = msg?.status?.[0]
          if (!status) return
          setSnapshots((prev) => ({
            ...prev,
            [spec.id]: {
              updatedAt: Date.now(),
              values: status.values ?? [],
            },
          }))
        },
        { throttleRate: 1000 }
      )
    )
    return () => {
      unsubscribers.forEach((off) => off())
    }
  }, [ros])

  useEffect(() => {
    const unsubscribers = BATTERY_TOPICS.map((spec) =>
      ros.subscribe<PackTelemetry>(
        spec.topic,
        'mr2_battery_monitor/msg/PackTelemetry',
        (msg) => {
          if (!msg) return
          setBatterySnapshots((prev) => ({
            ...prev,
            [spec.id]: {
              updatedAt: Date.now(),
              values: buildBatteryValues(msg),
            },
          }))
        },
        { throttleRate: 1000 }
      )
    )
    return () => {
      unsubscribers.forEach((off) => off())
    }
  }, [ros])

  useEffect(() => {
    const unsubscribe = gateway.onBaseStatus((status) => {
      setBaseStatus(status)
      setBaseStatusUpdatedAt(Date.now())
    })
    return () => unsubscribe()
  }, [gateway])

  useEffect(() => {
    if (typeof window === 'undefined') return
    const stored = window.localStorage.getItem('baseHeadingDeg')
    if (!stored) return
    const parsed = Number(stored)
    if (!Number.isFinite(parsed)) return
    const normalized = ((parsed % 360) + 360) % 360
    gateway.sendBaseHeading(normalized)
  }, [gateway])

  const applyBaseHeading = () => {
    const parsed = Number(baseHeadingInput)
    if (!Number.isFinite(parsed)) return
    const normalized = ((parsed % 360) + 360) % 360
    if (typeof window !== 'undefined') {
      window.localStorage.setItem('baseHeadingDeg', String(normalized))
    }
    gateway.sendBaseHeading(normalized)
  }

  const cards = useMemo<StatusCard[]>(() => {
    const baseValues = baseStatus ? buildBaseValues(baseStatus) : []
    const baseSnapshot =
      baseStatusUpdatedAt != null
        ? {
            updatedAt: baseStatusUpdatedAt,
            values: baseValues,
          }
        : null

    const baseCard: StatusCard = {
      spec: { id: 'base_station', label: 'Base Station' },
      snapshot: baseSnapshot,
      values: baseValues,
    }

    const systemCards: StatusCard[] = SYSTEM_STATUS_TOPICS.map((spec) => {
      const snapshot = snapshots[spec.id]
      const values = snapshot?.values ?? []
      return {
        spec: { id: spec.id, label: spec.label },
        snapshot,
        values: [...values].sort((a, b) => a.key.localeCompare(b.key)),
      }
    })

    const batteryCards: StatusCard[] = BATTERY_SPECS.map((spec) => {
      const snapshot = batterySnapshots[spec.id]
      return {
        spec,
        snapshot,
        values: snapshot?.values ?? [],
      }
    })

    return [baseCard, ...batteryCards, ...systemCards]
  }, [baseStatus, baseStatusUpdatedAt, batterySnapshots, snapshots])

  return {
    cards,
    baseStatus,
    baseHeadingInput,
    setBaseHeadingInput,
    applyBaseHeading,
  }
}
