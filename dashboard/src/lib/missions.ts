export type MissionSpec = {
  mission_id: number
  mission_type: number
  detection_method: number
  object_type: number
  target_latitude: number
  target_longitude: number
  target_radius: number
  waypoint_count: number
}

type MissionOption = {
  value: number
  label: string
}

export const MISSION_TYPES: MissionOption[] = [
  { value: 0, label: 'UNKNOWN' },
  { value: 1, label: 'GNSS_ONLY' },
  { value: 2, label: 'COVER_VISION' },
]

export const DETECTION_METHODS: MissionOption[] = [
  { value: 0, label: 'NONE' },
  { value: 1, label: 'ARUCO' },
  { value: 2, label: 'YOLO' },
]

export const OBJECT_TYPES: MissionOption[] = [
  { value: 0, label: 'MALLET' },
  { value: 1, label: 'PICK' },
  { value: 2, label: 'BOTTLE' },
]

const MISSION_TYPE_VALUES = new Set(MISSION_TYPES.map((item) => item.value))
const DETECTION_METHOD_VALUES = new Set(DETECTION_METHODS.map((item) => item.value))
const OBJECT_TYPE_VALUES = new Set(OBJECT_TYPES.map((item) => item.value))

export const CSV_HEADERS = [
  'mission_id',
  'mission_type',
  'detection_method',
  'object_type',
  'target_latitude',
  'target_longitude',
  'target_radius',
  'waypoint_count',
] as const

export type MissionField = keyof MissionSpec

export const createMissionSpec = (
  missionId: number,
  overrides: Partial<MissionSpec> = {}
): MissionSpec => ({
  mission_id: missionId,
  mission_type: 1,
  detection_method: 0,
  object_type: 0,
  target_latitude: 0,
  target_longitude: 0,
  target_radius: 0,
  waypoint_count: 0,
  ...overrides,
})

export const missionListToCsv = (missions: MissionSpec[]) => {
  const header = CSV_HEADERS.join(',')
  const rows = missions.map((mission) => {
    const record = mission as Record<MissionField, number>
    return CSV_HEADERS.map((key) =>
      Number.isFinite(record[key]) ? String(record[key]) : ''
    ).join(',')
  })
  return [header, ...rows].join('\n')
}

export const csvToMissionList = (csv: string) => {
  const rows = csv
    .split(/\r?\n/)
    .map((row) => row.trim())
    .filter(Boolean)
  if (!rows.length) return []
  const firstCells = rows[0].split(',').map((cell) => cell.trim().toLowerCase())
  const hasHeader = CSV_HEADERS.every(
    (header, index) => firstCells[index] === header.toLowerCase()
  )
  const start = hasHeader ? 1 : 0
  const missions: MissionSpec[] = []
  for (let i = start; i < rows.length; i += 1) {
    const cells = rows[i].split(',').map((cell) => cell.trim())
    if (cells.length < CSV_HEADERS.length) continue
    const values = cells
      .slice(0, CSV_HEADERS.length)
      .map((value) => Number.parseFloat(value))
    if (values.some((value) => !Number.isFinite(value))) continue
    missions.push(
      createMissionSpec(values[0], {
        mission_type: values[1],
        detection_method: values[2],
        object_type: values[3],
        target_latitude: values[4],
        target_longitude: values[5],
        target_radius: values[6],
        waypoint_count: values[7],
      })
    )
  }
  return missions
}

export const getInvalidMissionFields = (mission: MissionSpec): MissionField[] => {
  const invalid: MissionField[] = []
  if (!MISSION_TYPE_VALUES.has(mission.mission_type)) invalid.push('mission_type')
  if (!DETECTION_METHOD_VALUES.has(mission.detection_method)) invalid.push('detection_method')
  if (!OBJECT_TYPE_VALUES.has(mission.object_type)) invalid.push('object_type')
  if (!Number.isFinite(mission.mission_id) || mission.mission_id < 0) invalid.push('mission_id')
  if (
    !Number.isFinite(mission.target_latitude) ||
    mission.target_latitude < -90 ||
    mission.target_latitude > 90
  ) {
    invalid.push('target_latitude')
  }
  if (
    !Number.isFinite(mission.target_longitude) ||
    mission.target_longitude < -180 ||
    mission.target_longitude > 180
  ) {
    invalid.push('target_longitude')
  }
  if (
    !Number.isFinite(mission.target_radius) ||
    mission.target_radius < 0 ||
    mission.target_radius > 99
  ) {
    invalid.push('target_radius')
  }
  if (!Number.isFinite(mission.waypoint_count) || mission.waypoint_count < 0) {
    invalid.push('waypoint_count')
  }
  return invalid
}
