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

export type CoordinateFormat = 'dd' | 'ddm' | 'dms'
export type CoordinateAxis = 'lat' | 'lon'

type CoordinateFormatOption = {
  value: CoordinateFormat
  label: string
  placeholder: string
}

export const COORDINATE_FORMATS: CoordinateFormatOption[] = [
  { value: 'dd', label: 'DD', placeholder: '38.406738' },
  { value: 'ddm', label: 'DDM', placeholder: '38 24.4043 N' },
  { value: 'dms', label: 'DMS', placeholder: '38 24 24.26 N' },
]

export const MISSION_TYPE_GNSS_ONLY = 1
export const MISSION_TYPE_COVER_VISION = 2

export const DETECTION_NONE = 0
export const DETECTION_ARUCO = 1
export const DETECTION_YOLO = 2

export const MISSION_TYPES: MissionOption[] = [
  { value: 1, label: 'GNSS' },
  { value: 2, label: 'Vision' },
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
const COVER_VISION_DETECTION_VALUES = new Set([DETECTION_ARUCO, DETECTION_YOLO])

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
  mission_type: MISSION_TYPE_GNSS_ONLY,
  detection_method: DETECTION_NONE,
  object_type: 0,
  target_latitude: 0,
  target_longitude: 0,
  target_radius: 0,
  waypoint_count: 0,
  ...overrides,
})

export const normalizeMissionSpec = (mission: MissionSpec): MissionSpec => {
  const missionType = MISSION_TYPE_VALUES.has(mission.mission_type)
    ? mission.mission_type
    : MISSION_TYPE_GNSS_ONLY

  if (missionType === MISSION_TYPE_GNSS_ONLY) {
    return {
      ...mission,
      mission_type: MISSION_TYPE_GNSS_ONLY,
      detection_method: DETECTION_NONE,
      object_type: 0,
      target_radius: 0,
    }
  }

  const detectionMethod = COVER_VISION_DETECTION_VALUES.has(mission.detection_method)
    ? mission.detection_method
    : DETECTION_ARUCO

  return {
    ...mission,
    mission_type: MISSION_TYPE_COVER_VISION,
    detection_method: detectionMethod,
    object_type:
      detectionMethod === DETECTION_YOLO && OBJECT_TYPE_VALUES.has(mission.object_type)
        ? mission.object_type
        : 0,
    waypoint_count: 0,
  }
}

export const getDetectionMethodsForMissionType = (missionType: number): MissionOption[] =>
  missionType === MISSION_TYPE_COVER_VISION
    ? DETECTION_METHODS.filter((item) => COVER_VISION_DETECTION_VALUES.has(item.value))
    : DETECTION_METHODS.filter((item) => item.value === DETECTION_NONE)

const getCoordinateRange = (axis: CoordinateAxis) => (axis === 'lat' ? 90 : 180)

const getCoordinateHemisphere = (value: number, axis: CoordinateAxis) => {
  if (axis === 'lat') return value < 0 ? 'S' : 'N'
  return value < 0 ? 'W' : 'E'
}

const getCoordinateSign = (input: string, firstValue: number) => {
  const hemisphere = input.match(/[NSEW]/i)?.[0]?.toUpperCase()
  if (hemisphere === 'S' || hemisphere === 'W') return -1
  if (hemisphere === 'N' || hemisphere === 'E') return 1
  return firstValue < 0 ? -1 : 1
}

const getCoordinateNumbers = (input: string) =>
  input.match(/[+-]?\d+(?:\.\d+)?/g)?.map((value) => Number.parseFloat(value)) ?? []

export const parseCoordinateInput = (
  input: string,
  format: CoordinateFormat,
  axis: CoordinateAxis
) => {
  const trimmed = input.trim()
  if (!trimmed) return null

  const numbers = getCoordinateNumbers(trimmed)
  if (numbers.length === 0 || numbers.some((value) => !Number.isFinite(value))) return null

  const sign = getCoordinateSign(trimmed, numbers[0])
  const degrees = Math.abs(numbers[0])
  let decimalDegrees: number

  if (format === 'dd') {
    decimalDegrees = degrees
  } else if (format === 'ddm') {
    if (numbers.length < 2) return null
    const minutes = Math.abs(numbers[1])
    if (minutes >= 60) return null
    decimalDegrees = degrees + minutes / 60
  } else {
    if (numbers.length < 3) return null
    const minutes = Math.abs(numbers[1])
    const seconds = Math.abs(numbers[2])
    if (minutes >= 60 || seconds >= 60) return null
    decimalDegrees = degrees + minutes / 60 + seconds / 3600
  }

  const value = sign * decimalDegrees
  return Math.abs(value) <= getCoordinateRange(axis) ? value : null
}

export const formatCoordinateValue = (
  value: number,
  format: CoordinateFormat,
  axis: CoordinateAxis
) => {
  if (!Number.isFinite(value) || value === 0) return ''

  const absValue = Math.abs(value)
  const hemisphere = getCoordinateHemisphere(value, axis)

  if (format === 'dd') return value.toFixed(7)

  const degrees = Math.floor(absValue)
  const minutesFloat = (absValue - degrees) * 60

  if (format === 'ddm') {
    return `${degrees} ${minutesFloat.toFixed(4)} ${hemisphere}`
  }

  const minutes = Math.floor(minutesFloat)
  const seconds = (minutesFloat - minutes) * 60
  return `${degrees} ${minutes} ${seconds.toFixed(2)} ${hemisphere}`
}

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
  if (
    mission.mission_type === MISSION_TYPE_GNSS_ONLY &&
    mission.detection_method !== DETECTION_NONE
  ) {
    invalid.push('detection_method')
  }
  if (
    mission.mission_type === MISSION_TYPE_COVER_VISION &&
    !COVER_VISION_DETECTION_VALUES.has(mission.detection_method)
  ) {
    invalid.push('detection_method')
  }
  if (mission.detection_method === DETECTION_ARUCO && mission.object_type !== 0) {
    invalid.push('object_type')
  }
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
  if (mission.mission_type === MISSION_TYPE_COVER_VISION && mission.target_radius <= 0) {
    invalid.push('target_radius')
  }
  if (mission.mission_type === MISSION_TYPE_GNSS_ONLY && mission.target_radius !== 0) {
    invalid.push('target_radius')
  }
  if (!Number.isFinite(mission.waypoint_count) || mission.waypoint_count < 0) {
    invalid.push('waypoint_count')
  }
  if (mission.mission_type === MISSION_TYPE_COVER_VISION && mission.waypoint_count !== 0) {
    invalid.push('waypoint_count')
  }
  return invalid
}
