import { useEffect, useMemo, useState, type DragEvent } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'
import { useXbeeGateway } from '../hooks/useXbeeGateway'
import {
  createMissionSpec,
  csvToMissionList,
  DETECTION_METHODS,
  getInvalidMissionFields,
  missionListToCsv,
  MISSION_TYPES,
  OBJECT_TYPES,
  type MissionSpec,
} from '../lib/missions'
import type { MissionControlMsg, MissionListMsg, MissionStatusMsg } from '../lib/rosMessages'
import './MissionMasterPanel.css'

const COMMANDS = [
  { id: 1, label: 'Pause' },
  { id: 2, label: 'Resume' },
  { id: 3, label: 'Abort' },
]

const STATE_LABELS: Record<number, string> = {
  0: 'IDLE',
  1: 'RUNNING',
  2: 'PAUSED',
  3: 'COMPLETED',
  4: 'FAILED',
}

type InvalidFieldMap = Record<number, Record<string, boolean>>

const STORAGE_SLOT_COUNT = 3
const STORAGE_PREFIX = 'missionListSlot'

type MissionMasterPanelProps = {
  missionList: MissionSpec[]
  grabFromMap: boolean
  onGrabFromMapChange: (enabled: boolean) => void
  onMissionListChange: (missions: MissionSpec[]) => void
  onMissionPreview: (missions: MissionSpec[]) => void
}

const MissionMasterPanel = ({
  missionList,
  grabFromMap,
  onGrabFromMapChange,
  onMissionListChange,
  onMissionPreview,
}: MissionMasterPanelProps) => {
  const { ros, connected: rosConnected } = useRosBridge()
  const { gateway } = useXbeeGateway()
  const [clearCostmap, setClearCostmap] = useState(false)
  const [missionId, setMissionId] = useState('0')
  const [status, setStatus] = useState<MissionStatusMsg | null>(null)
  const [statusAt, setStatusAt] = useState<number | null>(null)
  const [nowMs, setNowMs] = useState(() => Date.now())
  const [invalidFields, setInvalidFields] = useState<InvalidFieldMap>({})
  const [dragIndex, setDragIndex] = useState<number | null>(null)
  const [dropIndex, setDropIndex] = useState<number | null>(null)
  const [storageSlot, setStorageSlot] = useState(1)
  const [storageNote, setStorageNote] = useState<{ message: string; isError: boolean } | null>(
    null
  )

  useEffect(() => {
    const unsubscribe = ros.subscribe<MissionStatusMsg>(
      '/mission_status',
      'mr2_action_interface/msg/MissionStatus',
      (msg) => {
        setStatus(msg)
        setStatusAt(Date.now())
      },
      { throttleRate: 500 }
    )
    return () => unsubscribe()
  }, [ros])

  useEffect(() => {
    const timer = window.setInterval(() => {
      setNowMs(Date.now())
    }, 1000)
    return () => {
      window.clearInterval(timer)
    }
  }, [])

  const validateMissionList = () => {
    const nextInvalid: InvalidFieldMap = {}
    const clampIssues = (index: number, field: string) => {
      if (!nextInvalid[index]) nextInvalid[index] = {}
      nextInvalid[index][field] = true
    }

    missionList.forEach((mission, index) => {
      getInvalidMissionFields(mission).forEach((field) => clampIssues(index, field))
    })

    setInvalidFields(nextInvalid)
    return Object.keys(nextInvalid).length === 0
  }

  const handleSendMissionList = () => {
    if (!validateMissionList()) return

    const now = Date.now()
    const msg: MissionListMsg = {
      stamp: {
        sec: Math.floor(now / 1000),
        nanosec: (now % 1000) * 1e6,
      },
      missions: missionList,
    }
    ros.publish('/mission_list', 'mr2_action_interface/msg/MissionList', msg)
  }

  const handlePreviewMissionList = () => {
    if (!validateMissionList()) return
    onMissionPreview(missionList)
  }

  const handleSendControl = (command: number) => {
    const id = Number.parseInt(missionId, 10)
    const msg: MissionControlMsg = {
      command,
      clear_costmap: clearCostmap,
      mission_id: Number.isFinite(id) ? id : 0,
    }
    gateway.sendMissionControl(msg)
  }

  const statusLabel =
    status?.state != null ? STATE_LABELS[status.state] ?? 'UNKNOWN' : '—'
  const statusDetail = status?.detail?.trim() || '—'
  const arrivalLabel = status?.arrival ? 'ARRIVAL' : '—'
  const active = status?.active_mission
  const lastStatusAge =
    statusAt != null ? `${((nowMs - statusAt) / 1000).toFixed(1)}s ago` : '—'
  const statusStale = statusAt == null || nowMs - statusAt > 1000
  const statusFlash = !statusStale && !!status?.arrival
  const ledMode = (() => {
    if (statusStale || !status) return 'off'
    if (status.arrival) return 'success'
    if (status.state === 1) return 'autonomous'
    if (status.state === 2) return 'manual'
    if (status.state === 0 || status.state === 3 || status.state === 4) return 'off'
    return 'off'
  })()

  const missionSummary = useMemo(() => {
    if (!missionList.length) return 'No missions queued'
    return missionList
      .map((mission) => `#${mission.mission_id} (${mission.mission_type})`)
      .join(', ')
  }, [missionList])

  const clearInvalid = (index: number, field: string) => {
    setInvalidFields((prev) => {
      if (!prev[index]?.[field]) return prev
      const next = { ...prev }
      const row = { ...next[index] }
      delete row[field]
      if (Object.keys(row).length === 0) {
        delete next[index]
      } else {
        next[index] = row
      }
      return next
    })
  }

  const updateMission = (index: number, patch: Partial<MissionSpec>) => {
    onMissionListChange(
      missionList.map((mission, i) => (i === index ? { ...mission, ...patch } : mission))
    )
  }

  const reorderList = <T,>(list: T[], from: number, to: number) => {
    if (from === to) return list
    const next = [...list]
    const [moved] = next.splice(from, 1)
    next.splice(to, 0, moved)
    return next
  }

  const reorderInvalidFields = (from: number, to: number) => {
    setInvalidFields((prev) => {
      if (!prev || Object.keys(prev).length === 0) return prev
      const entries = missionList.map((_, index) => prev[index] || {})
      const reordered = reorderList(entries, from, to)
      const next: InvalidFieldMap = {}
      reordered.forEach((row, index) => {
        if (Object.keys(row).length > 0) next[index] = row
      })
      return next
    })
  }

  const moveMission = (from: number, to: number) => {
    onMissionListChange(reorderList(missionList, from, to))
    reorderInvalidFields(from, to)
  }

  const handleDragStart = (index: number) => (event: DragEvent) => {
    event.dataTransfer.effectAllowed = 'move'
    event.dataTransfer.setData('text/plain', String(index))
    setDragIndex(index)
  }

  const handleDragOver = (index: number) => (event: DragEvent) => {
    event.preventDefault()
    event.dataTransfer.dropEffect = 'move'
    if (dropIndex !== index) setDropIndex(index)
  }

  const handleDrop = (index: number) => (event: DragEvent) => {
    event.preventDefault()
    const from =
      dragIndex != null
        ? dragIndex
        : Number.parseInt(event.dataTransfer.getData('text/plain') || '', 10)
    if (!Number.isFinite(from) || from < 0 || from >= missionList.length) {
      setDragIndex(null)
      setDropIndex(null)
      return
    }
    if (from !== index) {
      moveMission(from, index)
    }
    setDragIndex(null)
    setDropIndex(null)
  }

  const handleDragEnd = () => {
    setDragIndex(null)
    setDropIndex(null)
  }

  const updateField = (index: number, field: keyof MissionSpec, value: number) => {
    updateMission(index, { [field]: value } as Partial<MissionSpec>)
    clearInvalid(index, String(field))
  }

  const addMission = () => {
    const nextId =
      missionList.reduce(
        (max, mission) =>
          Number.isFinite(mission.mission_id) ? Math.max(max, mission.mission_id) : max,
        0
      ) + 1
    onMissionListChange([
      ...missionList,
      createMissionSpec(nextId, {
        target_latitude: 0,
        target_longitude: 0,
      }),
    ])
  }

  const removeMission = (index: number) => {
    onMissionListChange(missionList.filter((_, i) => i !== index))
  }

  const handleSaveTable = () => {
    if (typeof window === 'undefined') return
    const key = `${STORAGE_PREFIX}${storageSlot}`
    window.localStorage.setItem(key, missionListToCsv(missionList))
    setStorageNote({
      message: `Saved ${missionList.length} missions to slot ${storageSlot}.`,
      isError: false,
    })
  }

  const handleLoadTable = () => {
    if (typeof window === 'undefined') return
    const key = `${STORAGE_PREFIX}${storageSlot}`
    const stored = window.localStorage.getItem(key)
    if (!stored) {
      setStorageNote({ message: `Slot ${storageSlot} is empty.`, isError: true })
      return
    }
    const missions = csvToMissionList(stored)
    if (missions.length === 0) {
      setStorageNote({
        message: `Slot ${storageSlot} has no readable missions.`,
        isError: true,
      })
      return
    }
    onMissionListChange(missions)
    setInvalidFields({})
    setStorageNote({
      message: `Loaded ${missions.length} missions from slot ${storageSlot}.`,
      isError: false,
    })
  }

  return (
    <>
      <article className="card card--span-2 mission-master">
        <header className="mission-master__header">
          <h3>Mission Master</h3>
          <span className={`pill ${rosConnected ? 'pill--on' : 'pill--off'}`}>
            {rosConnected ? 'rosbridge connected' : 'rosbridge offline'}
          </span>
        </header>

        <div className="mission-master__grid">
          <section className="mission-master__status">
            <label className="mission-master__label">MissionStatus</label>
            <div
              className={`mission-master__status-shell mission-master__status-${ledMode} ${
                statusFlash ? 'mission-master__status-flash' : ''
              }`}
            >
              <div className="mission-master__status-row mission-master__status-row--primary">
                <div className="mission-master__status-item">
                  <span>State</span>
                  <strong className="mission-master__status-state">{statusLabel}</strong>
                </div>
                <div className="mission-master__status-item">
                  <span>Arrival</span>
                  <strong>{arrivalLabel}</strong>
                </div>
                <div className="mission-master__status-item">
                  <span>Last update</span>
                  <strong>{lastStatusAge}</strong>
                </div>
                <div className="mission-master__status-item">
                  <span>Active mission</span>
                  <strong>{active ? `#${active.mission_id}` : '—'}</strong>
                </div>
                <div className="mission-master__status-item">
                  <span>Waypoints</span>
                  <strong>
                    {status?.current_waypoint_index ?? '—'} / {status?.total_waypoints ?? '—'}
                  </strong>
                </div>
                <div className="mission-master__status-item">
                  <span>Distance</span>
                  <strong>
                    {Number.isFinite(status?.distance_remaining)
                      ? `${status?.distance_remaining?.toFixed(2)} m`
                      : '—'}
                  </strong>
                </div>
              </div>
              <div className="mission-master__status-row mission-master__status-row--detail">
                <div className="mission-master__status-item mission-master__detail">
                  <span>Detail</span>
                  <strong>{statusDetail}</strong>
                </div>
              </div>
            </div>
          </section>

          <section className="mission-master__controls">
            <label className="mission-master__label">MissionControl</label>
            <div className="mission-master__control-row">
              <label className="mission-master__checkbox">
                <input
                  type="checkbox"
                  checked={clearCostmap}
                  onChange={(event) => setClearCostmap(event.target.checked)}
                />
                Clear costmap
              </label>
              <input
                type="number"
                className="mission-master__input"
                value={missionId}
                onChange={(event) => setMissionId(event.target.value)}
                placeholder="mission_id"
                min={0}
              />
              <div className="mission-master__control-buttons">
                {COMMANDS.map((cmd) => (
                  <button
                    key={cmd.id}
                    type="button"
                    className={`mission-master__button ${
                      cmd.label === 'Pause'
                        ? 'mission-master__button--pause'
                        : cmd.label === 'Resume'
                        ? 'mission-master__button--resume'
                        : 'mission-master__button--abort'
                    }`}
                    onClick={() => handleSendControl(cmd.id)}
                  >
                    {cmd.label}
                  </button>
                ))}
              </div>
              <span className="mission-master__note">Send via XBEE</span>
            </div>
          </section>

          <section>
            <label className="mission-master__label">MissionList</label>
            <div className="mission-master__actions mission-master__actions--top">
              <button type="button" className="mission-master__button" onClick={addMission}>
                Add mission
              </button>
              <button
                type="button"
                className={`mission-master__button mission-master__button--toggle ${
                  grabFromMap ? 'mission-master__button--toggle-active' : ''
                }`}
                onClick={() => onGrabFromMapChange(!grabFromMap)}
                aria-pressed={grabFromMap}
              >
                Grab from map
              </button>
              <button
                type="button"
                className="mission-master__button mission-master__button--ghost"
                onClick={handlePreviewMissionList}
              >
                Preview Points
              </button>
              <button
                type="button"
                className="mission-master__button mission-master__button--send"
                onClick={handleSendMissionList}
              >
                Send MissionList
              </button>
            </div>
            <div className="mission-master__storage">
              <div className="mission-master__storage-row">
                <span className="mission-master__storage-label">Table slots</span>
                <div className="mission-master__slot-group">
                  {Array.from({ length: STORAGE_SLOT_COUNT }, (_, index) => {
                    const slot = index + 1
                    return (
                      <button
                        key={slot}
                        type="button"
                        className={`mission-master__slot-button ${
                          storageSlot === slot ? 'mission-master__slot-button--active' : ''
                        }`}
                        onClick={() => setStorageSlot(slot)}
                      >
                        Slot {slot}
                      </button>
                    )
                  })}
                </div>
                <div className="mission-master__storage-actions">
                  <button
                    type="button"
                    className="mission-master__button mission-master__button--ghost"
                    onClick={handleSaveTable}
                  >
                    Save table
                  </button>
                  <button
                    type="button"
                    className="mission-master__button mission-master__button--ghost"
                    onClick={handleLoadTable}
                  >
                    Load table
                  </button>
                </div>
              </div>
              {storageNote ? (
                <p
                  className={`mission-master__storage-note ${
                    storageNote.isError ? 'mission-master__storage-note--error' : ''
                  }`}
                >
                  {storageNote.message}
                </p>
              ) : null}
            </div>
            <div className="mission-master__table">
              <div className="mission-master__table-row mission-master__table-row--header">
                <span />
                <span>ID</span>
                <span>Type</span>
                <span>Detect</span>
                <span>Object</span>
                <span>Lat</span>
                <span>Lon</span>
                <span>Radius</span>
                <span title="Via point skips the arrival hold between missions.">Via</span>
                <span />
              </div>
              {missionList.length === 0 ? (
                <div className="mission-master__empty">No missions yet.</div>
              ) : (
                missionList.map((mission, index) => (
                  <div
                    className={`mission-master__table-row ${
                      invalidFields[index] ? 'mission-master__table-row--invalid' : ''
                    } ${dropIndex === index ? 'mission-master__table-row--drop' : ''}`}
                    key={`${mission.mission_id}-${index}`}
                    onDragOver={handleDragOver(index)}
                    onDrop={handleDrop(index)}
                  >
                    <button
                      type="button"
                      className="mission-master__drag-handle"
                      draggable
                      onDragStart={handleDragStart(index)}
                      onDragEnd={handleDragEnd}
                      aria-label={`Reorder mission ${mission.mission_id}`}
                      title="Drag to reorder"
                    >
                      ⋮⋮
                    </button>
                    <input
                      type="number"
                      value={mission.mission_id}
                      onChange={(event) =>
                        updateField(index, 'mission_id', Number(event.target.value) || 0)
                      }
                      className={
                        invalidFields[index]?.mission_id ? 'mission-master__input--invalid' : ''
                      }
                    />
                    <select
                      value={mission.mission_type}
                      onChange={(event) =>
                        updateField(index, 'mission_type', Number(event.target.value))
                      }
                      className={
                        invalidFields[index]?.mission_type ? 'mission-master__input--invalid' : ''
                      }
                    >
                      {MISSION_TYPES.map((opt) => (
                        <option key={opt.value} value={opt.value}>
                          {opt.label}
                        </option>
                      ))}
                    </select>
                    <select
                      value={mission.detection_method}
                      onChange={(event) =>
                        updateField(index, 'detection_method', Number(event.target.value))
                      }
                      className={
                        invalidFields[index]?.detection_method ? 'mission-master__input--invalid' : ''
                      }
                    >
                      {DETECTION_METHODS.map((opt) => (
                        <option key={opt.value} value={opt.value}>
                          {opt.label}
                        </option>
                      ))}
                    </select>
                    <select
                      value={mission.object_type}
                      onChange={(event) =>
                        updateField(index, 'object_type', Number(event.target.value))
                      }
                      className={
                        invalidFields[index]?.object_type ? 'mission-master__input--invalid' : ''
                      }
                    >
                      {OBJECT_TYPES.map((opt) => (
                        <option key={opt.value} value={opt.value}>
                          {opt.label}
                        </option>
                      ))}
                    </select>
                    <input
                      key={`lat-${mission.mission_id}-${mission.target_latitude}`}
                      type="text"
                      inputMode="decimal"
                      defaultValue={
                        mission.target_latitude === 0 ? '' : String(mission.target_latitude)
                      }
                      onBlur={(event) => {
                        const value = Number.parseFloat(event.target.value)
                        if (Number.isFinite(value)) {
                          updateField(index, 'target_latitude', value)
                        }
                      }}
                      className={
                        invalidFields[index]?.target_latitude ? 'mission-master__input--invalid' : ''
                      }
                    />
                    <input
                      key={`lon-${mission.mission_id}-${mission.target_longitude}`}
                      type="text"
                      inputMode="decimal"
                      defaultValue={
                        mission.target_longitude === 0 ? '' : String(mission.target_longitude)
                      }
                      onBlur={(event) => {
                        const value = Number.parseFloat(event.target.value)
                        if (Number.isFinite(value)) {
                          updateField(index, 'target_longitude', value)
                        }
                      }}
                      className={
                        invalidFields[index]?.target_longitude ? 'mission-master__input--invalid' : ''
                      }
                    />
                    <input
                      type="number"
                      value={mission.target_radius === 0 ? '' : mission.target_radius}
                      onChange={(event) =>
                        updateField(index, 'target_radius', Number(event.target.value) || 0)
                      }
                      className={
                        invalidFields[index]?.target_radius ? 'mission-master__input--invalid' : ''
                      }
                    />
                    <div className="mission-master__via-cell">
                      <input
                        type="checkbox"
                        className={`mission-master__via-checkbox ${
                          invalidFields[index]?.waypoint_count ? 'mission-master__input--invalid' : ''
                        }`}
                        checked={mission.waypoint_count === 1}
                        onChange={(event) =>
                          updateField(index, 'waypoint_count', event.target.checked ? 1 : 0)
                        }
                        aria-label="Via point (skip arrival delay)"
                        title="Via point skips the arrival hold between missions."
                      />
                    </div>
                    <button
                      type="button"
                      className="mission-master__icon-button"
                      onClick={() => removeMission(index)}
                      aria-label="Remove mission"
                    >
                      ✕
                    </button>
                  </div>
                ))
              )}
            </div>
            <p className="mission-master__meta">{missionSummary}</p>
          </section>
        </div>
      </article>
    </>
  )
}

export default MissionMasterPanel
