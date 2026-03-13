import type { GeoPathMsg, PoseStamped } from './rosMessages'
import type { MissionSpec } from './missions'

export type MapCoordinate = [number, number]

export type NavCovariance = {
  xVar: number
  yVar: number
  yawVar: number
}

export const MAX_SMOOTHED_POINTS = 120
export const MAX_COVERAGE_POINTS = 300

const MISSION_CIRCLE_STEPS = 64
const WGS84_A = 6378137
const RAD_TO_DEG = 180 / Math.PI
const DEG_TO_RAD = Math.PI / 180

export const COVER_VISION_OBJECT_TOPICS = [
  '/cover_vision/object_pose/aruco',
  '/cover_vision/object_pose/yolo',
  '/cover_vision/object_pose',
] as const

export const convertGeoPath = (
  msg: GeoPathMsg | null,
  maxPoints: number
): MapCoordinate[] => {
  const poses = msg?.poses ?? []
  if (poses.length === 0) return []
  const step = poses.length > maxPoints ? Math.ceil(poses.length / maxPoints) : 1
  const coords: MapCoordinate[] = []
  for (let i = 0; i < poses.length; i += step) {
    const position = poses[i]?.pose?.position
    const lat = Number(position?.latitude)
    const lon = Number(position?.longitude)
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) continue
    coords.push([lon, lat])
  }
  return coords
}

export const appendTrailPoint = (
  trail: MapCoordinate[],
  nextFix: MapCoordinate
): MapCoordinate[] => {
  const last = trail[trail.length - 1]
  if (last && Math.abs(last[0] - nextFix[0]) < 1e-6 && Math.abs(last[1] - nextFix[1]) < 1e-6) {
    return trail
  }
  const next = [...trail, nextFix]
  if (next.length > 1200) next.shift()
  return next
}

const buildMissionCircle = (lon: number, lat: number, radius: number) => {
  const latRad = lat * DEG_TO_RAD
  const coords: MapCoordinate[] = []
  for (let i = 0; i <= MISSION_CIRCLE_STEPS; i += 1) {
    const theta = (2 * Math.PI * i) / MISSION_CIRCLE_STEPS
    const dLat = (radius * Math.sin(theta)) / WGS84_A
    const dLon = (radius * Math.cos(theta)) / (WGS84_A * Math.cos(latRad))
    coords.push([lon + dLon * RAD_TO_DEG, lat + dLat * RAD_TO_DEG])
  }
  return coords
}

export const buildMissionPointFeatures = (missions: MissionSpec[]) => {
  const total = missions.length
  const toColor = (index: number) => {
    if (total <= 1) return '#ff4d4d'
    const t = index / (total - 1)
    let r = 0
    let g = 0
    let b = 0
    if (t <= 0.5) {
      const local = t / 0.5
      r = Math.round(255 * (1 - local))
      g = Math.round(255 * local)
    } else {
      const local = (t - 0.5) / 0.5
      g = Math.round(255 * (1 - local))
      b = Math.round(255 * local)
    }
    return `#${r.toString(16).padStart(2, '0')}${g
      .toString(16)
      .padStart(2, '0')}${b.toString(16).padStart(2, '0')}`
  }

  return missions
    .filter(
      (mission) =>
        Number.isFinite(mission.target_longitude) &&
        Number.isFinite(mission.target_latitude)
    )
    .map((mission, index) => ({
      type: 'Feature' as const,
      geometry: {
        type: 'Point' as const,
        coordinates: [mission.target_longitude, mission.target_latitude],
      },
      properties: {
        order_color: toColor(index),
        mission_id: mission.mission_id,
        mission_type: mission.mission_type,
        detection_method: mission.detection_method,
        object_type: mission.object_type,
        target_radius: mission.target_radius,
      },
    }))
}

export const buildMissionCircleFeatures = (missions: MissionSpec[]) =>
  missions
    .filter(
      (mission) =>
        Number.isFinite(mission.target_longitude) &&
        Number.isFinite(mission.target_latitude) &&
        Number.isFinite(mission.target_radius) &&
        mission.target_radius > 0
    )
    .map((mission) => ({
      type: 'Feature' as const,
      geometry: {
        type: 'Polygon' as const,
        coordinates: [
          buildMissionCircle(
            mission.target_longitude,
            mission.target_latitude,
            mission.target_radius
          ),
        ],
      },
      properties: {
        mission_id: mission.mission_id,
        mission_type: mission.mission_type,
      },
    }))

export const getPoseStampMs = (msg: PoseStamped | null): number => {
  const stamp = msg?.header?.stamp
  const sec = Number(stamp?.sec ?? stamp?.secs)
  const nanosec = Number(stamp?.nanosec ?? stamp?.nsecs)
  if (!Number.isFinite(sec)) return 0
  const nanos = Number.isFinite(nanosec) ? nanosec : 0
  return sec * 1000 + nanos / 1e6
}

export const createRoverMarkerElement = () => {
  const el = document.createElement('div')
  el.innerHTML = `
    <svg width="34" height="34" viewBox="0 0 34 34">
    <defs>
      <filter id="glow" x="-50%" y="-50%" width="200%" height="200%">
        <feGaussianBlur stdDeviation="2" result="blur"/>
        <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
      </filter>
    </defs>
    <g filter="url(#glow)" transform="translate(17 17)">
      <path d="M 0 -12 L 8 10 L 0 6 L -8 10 Z" fill="#35d3c3" stroke="#0b1220" stroke-width="1.5"/>
      <circle cx="0" cy="0" r="2.6" fill="#0b1220" stroke="#35d3c3" stroke-width="1.2"/>
    </g>
    </svg>
  `
  return el
}

export const createBaseMarkerElement = () => {
  const el = document.createElement('div')
  el.innerHTML = `
    <svg width="30" height="30" viewBox="0 0 30 30">
    <defs>
      <filter id="baseGlow" x="-50%" y="-50%" width="200%" height="200%">
        <feGaussianBlur stdDeviation="1.5" result="blur"/>
        <feMerge><feMergeNode in="blur"/><feMergeNode in="SourceGraphic"/></feMerge>
      </filter>
    </defs>
    <g filter="url(#baseGlow)" transform="translate(15 15)">
      <circle cx="0" cy="0" r="6" fill="#f4d35e" stroke="#0b1220" stroke-width="1.5"/>
      <path d="M 0 -12 L 4 0 L 0 -2 L -4 0 Z" fill="#f4d35e" stroke="#0b1220" stroke-width="1.2"/>
    </g>
    </svg>
  `
  return el
}
