import { useEffect, useMemo, useState } from 'react'
import { getRosBridgeClient } from '../lib/rosBridge'
import './SystemStatusPanel.css'

type DiagnosticKeyValue = {
  key: string
  value: string
}

type DiagnosticStatus = {
  level: number
  name: string
  message: string
  values: DiagnosticKeyValue[]
}

type DiagnosticArray = {
  status: DiagnosticStatus[]
}

type TopicSpec = {
  id: string
  label: string
  topic: string
}

type StatusSnapshot = {
  updatedAt: number
  values: DiagnosticKeyValue[]
}

type BatteryCardId = 'battery_1' | 'battery_2'

type PackTelemetry = {
  state_of_charge_pct?: number
  health_pct?: number
  temperature_c?: number
  pack_voltage_v?: number
  pack_life_cycles?: number
  firmware_cycle_count?: number
  nominal_cell_capacity_mah?: number
  parallel_group_count?: number
  cell_count?: number
  cell_voltage_mv?: number[]
  cell_voltage_valid?: boolean[]
}

const TOPICS: TopicSpec[] = [
  { id: 'cpu', label: 'CPU', topic: '/system_status/cpu' },
  { id: 'memory', label: 'Memory', topic: '/system_status/memory' },
  { id: 'disk', label: 'Disk', topic: '/system_status/disk' },
  { id: 'load', label: 'Load', topic: '/system_status/load' },
  { id: 'network', label: 'Network', topic: '/system_status/network' },
  { id: 'swap', label: 'Swap', topic: '/system_status/swap' },
  { id: 'temperatures', label: 'Temperatures', topic: '/system_status/temperatures' },
  { id: 'uptime', label: 'Uptime', topic: '/system_status/uptime' },
]

const BATTERY_SPECS: Array<{ id: BatteryCardId; label: string }> = [
  { id: 'battery_1', label: 'Battery 1' },
  { id: 'battery_2', label: 'Battery 2' },
]

const BATTERY_TOPICS: Array<{ id: BatteryCardId; topic: string }> = [
  { id: 'battery_1', topic: '/battery_1/telemetry' },
  { id: 'battery_2', topic: '/battery_2/telemetry' },
]

const BYTE_KEYS = new Set([
  'total',
  'available',
  'used',
  'free',
  'active',
  'inactive',
  'buffers',
  'cached',
  'shared',
  'slab',
  'bytes_sent',
  'bytes_recv',
])

const COUNT_KEYS = new Set([
  'count_logical',
  'count_physical',
  'packets_sent',
  'packets_recv',
  'errin',
  'errout',
  'dropin',
  'dropout',
  'sin',
  'sout',
])

const buildBatteryValues = (telem: PackTelemetry): DiagnosticKeyValue[] => {
  const soc = telem.state_of_charge_pct
  const health = telem.health_pct
  const temperature = telem.temperature_c
  const voltage = telem.pack_voltage_v
  const packCycles = telem.pack_life_cycles
  const firmwareCycle = telem.firmware_cycle_count
  const nominalCellMah = telem.nominal_cell_capacity_mah
  const parallelGroups = telem.parallel_group_count
  const cellCount = telem.cell_count

  const totalMah =
    Number.isFinite(nominalCellMah) && Number.isFinite(parallelGroups)
      ? (nominalCellMah as number) * (parallelGroups as number)
      : null
  const availableMah =
    totalMah != null && Number.isFinite(soc)
      ? totalMah * ((soc as number) / 100.0)
      : null

  const voltages = telem.cell_voltage_mv ?? []
  const valids = telem.cell_voltage_valid ?? []
  const validSamples = voltages.filter((value, idx) => {
    if (!Number.isFinite(value)) return false
    if (valids.length > 0) {
      return !!valids[idx]
    }
    return value > 0
  })
  const validCount = validSamples.length
  const minMv = validCount ? Math.min(...validSamples) : null
  const maxMv = validCount ? Math.max(...validSamples) : null
  const avgMv =
    validCount > 0 ? validSamples.reduce((sum, v) => sum + v, 0) / validCount : null

  return [
    {
      key: 'state_of_charge',
      value: Number.isFinite(soc) ? `${(soc as number).toFixed(1)} %` : '--',
    },
    {
      key: 'health',
      value: Number.isFinite(health) ? `${(health as number).toFixed(1)} %` : '--',
    },
    {
      key: 'pack_voltage',
      value: Number.isFinite(voltage) ? `${(voltage as number).toFixed(2)} V` : '--',
    },
    {
      key: 'temperature',
      value: Number.isFinite(temperature)
        ? `${(temperature as number).toFixed(1)} °C`
        : '--',
    },
    {
      key: 'pack_life_cycles',
      value: Number.isFinite(packCycles) ? `${Math.round(packCycles as number)}` : '--',
    },
    {
      key: 'firmware_cycle_count',
      value: Number.isFinite(firmwareCycle)
        ? `${Math.round(firmwareCycle as number)}`
        : '--',
    },
    {
      key: 'nominal_cell_capacity',
      value: Number.isFinite(nominalCellMah)
        ? `${Math.round(nominalCellMah as number)} mAh`
        : '--',
    },
    {
      key: 'parallel_groups',
      value: Number.isFinite(parallelGroups)
        ? `${Math.round(parallelGroups as number)}`
        : '--',
    },
    {
      key: 'cell_count',
      value: Number.isFinite(cellCount) ? `${Math.round(cellCount as number)}` : '--',
    },
    {
      key: 'available_capacity',
      value:
        availableMah != null ? `${(availableMah / 1000).toFixed(2)} Ah` : '--',
    },
    {
      key: 'total_capacity',
      value: totalMah != null ? `${(totalMah / 1000).toFixed(2)} Ah` : '--',
    },
    {
      key: 'cell_voltage_min',
      value: minMv != null ? `${(minMv / 1000).toFixed(3)} V` : '--',
    },
    {
      key: 'cell_voltage_avg',
      value: avgMv != null ? `${(avgMv / 1000).toFixed(3)} V` : '--',
    },
    {
      key: 'cell_voltage_max',
      value: maxMv != null ? `${(maxMv / 1000).toFixed(3)} V` : '--',
    },
    {
      key: 'valid_cells',
      value: Number.isFinite(cellCount)
        ? `${validCount}/${Math.round(cellCount as number)}`
        : `${validCount}`,
    },
  ]
}

const SystemStatusPanel = () => {
  const [snapshots, setSnapshots] = useState<Record<string, StatusSnapshot | null>>({})
  const [batterySnapshots, setBatterySnapshots] = useState<
    Record<BatteryCardId, StatusSnapshot | null>
  >({
    battery_1: null,
    battery_2: null,
  })

  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
    const unsubscribers = TOPICS.map((spec) =>
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
      for (const off of unsubscribers) {
        off()
      }
    }
  }, [])

  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
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
      for (const off of unsubscribers) {
        off()
      }
    }
  }, [])

  const cards = useMemo(() => {
    const systemCards = TOPICS.map((spec) => {
      const snapshot = snapshots[spec.id]
      const values = snapshot?.values ?? []
      const sortedValues = [...values].sort((a, b) => a.key.localeCompare(b.key))
      return {
        spec: { id: spec.id, label: spec.label },
        snapshot,
        values: sortedValues,
      }
    })

    const batteryCards = BATTERY_SPECS.map((spec) => {
      const snapshot = batterySnapshots[spec.id]
      const values = snapshot?.values ?? []
      return {
        spec,
        snapshot,
        values,
      }
    })

    return [...batteryCards, ...systemCards]
  }, [snapshots, batterySnapshots])

  return (
    <div className="panel-grid" role="tabpanel">
      {cards.map(({ spec, snapshot, values }) => (
        <article className="card" key={spec.id}>
          <h3>{spec.label}</h3>
          {values.length === 0 ? (
            <div className="status-empty">No data yet.</div>
          ) : (
            <div className="status-kv-grid">
              {values.map((item) => (
                <div className="status-kv-row" key={item.key}>
                  <span className="status-kv-key">{formatKey(spec.id, item.key)}</span>
                  <strong className="status-kv-value">{formatValue(spec.id, item.key, item.value)}</strong>
                </div>
              ))}
            </div>
          )}
          {snapshot ? (
            <div className="status-updated">
              Updated {new Date(snapshot.updatedAt).toLocaleTimeString()}
            </div>
          ) : null}
        </article>
      ))}
    </div>
  )
}

const formatKey = (topicId: string, key: string) => {
  if (topicId === 'cpu' && key.startsWith('percent_per_core.')) {
    const idx = key.split('.').slice(-1)[0]
    return `core ${idx}`
  }
  if (topicId === 'load' && key.startsWith('load_avg.')) {
    const idx = key.split('.').slice(-1)[0]
    return `load ${idx}`
  }
  return key.replace(/_/g, ' ').replace(/\./g, ' / ')
}

const formatValue = (topicId: string, key: string, value: string) => {
  const num = Number(value)
  const isNum = Number.isFinite(num)
  if (!isNum) {
    return value
  }

  if (topicId === 'uptime' && key === 'uptime_sec') {
    return formatDuration(num)
  }

  if (topicId === 'temperatures') {
    return `${num.toFixed(1)}°C`
  }

  if (key.includes('percent')) {
    return `${num.toFixed(1)}%`
  }

  const baseKey = key.split('.').slice(-1)[0]
  if (BYTE_KEYS.has(baseKey)) {
    return formatBytes(num)
  }

  if (COUNT_KEYS.has(baseKey)) {
    return `${Math.round(num)}`
  }

  return num % 1 === 0 ? `${num}` : num.toFixed(2)
}

const formatBytes = (value: number) => {
  const gb = value / (1024 * 1024 * 1024)
  if (gb >= 1) {
    return `${gb.toFixed(2)} GiB`
  }
  const mb = value / (1024 * 1024)
  if (mb >= 1) {
    return `${mb.toFixed(1)} MiB`
  }
  const kb = value / 1024
  return `${kb.toFixed(1)} KiB`
}

const formatDuration = (seconds: number) => {
  const total = Math.max(0, Math.floor(seconds))
  const hrs = Math.floor(total / 3600)
  const mins = Math.floor((total % 3600) / 60)
  const secs = total % 60
  return `${hrs}h ${mins}m ${secs}s`
}

export default SystemStatusPanel
