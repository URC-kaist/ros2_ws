import { useCallback, useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import type { MissionSpec } from '../lib/missions'
import { getRosBridgeClient } from '../lib/rosBridge'
import { getSikGatewayClient } from '../lib/sikGateway'

type PoseStamped = {
  header?: {
    stamp?: {
      sec?: number
      nanosec?: number
      secs?: number
      nsecs?: number
    }
  }
  pose?: {
    position?: {
      x?: number
      y?: number
      z?: number
    }
  }
}

type GeoPoseStamped = {
  pose?: {
    position?: {
      latitude?: number
      longitude?: number
      altitude?: number
    }
  }
}

type GeoPathMsg = {
  poses?: GeoPoseStamped[]
}

const MAX_SMOOTHED_POINTS = 120
const MAX_COVERAGE_POINTS = 300
const MISSION_CIRCLE_STEPS = 64
const WGS84_A = 6378137
const RAD_TO_DEG = 180 / Math.PI
const DEG_TO_RAD = Math.PI / 180
const COVER_VISION_OBJECT_TOPICS = [
  '/cover_vision/object_pose/aruco',
  '/cover_vision/object_pose/yolo',
  '/cover_vision/object_pose',
] as const

type MapPreviewProps = {
  missionList?: MissionSpec[]
  grabFromMap?: boolean
  onGrabCoordinate?: (coord: { lat: number; lon: number }) => void
}

const MapPreview = ({
  missionList = [],
  grabFromMap = false,
  onGrabCoordinate,
}: MapPreviewProps) => {
  const mapRef = useRef<HTMLDivElement | null>(null)
  const mapInstanceRef = useRef<maplibregl.Map | null>(null)
  const markerRef = useRef<maplibregl.Marker | null>(null)
  const baseMarkerRef = useRef<maplibregl.Marker | null>(null)
  const [mapReady, setMapReady] = useState(false)
  const [followRover, setFollowRover] = useState(true)
  const [fix, setFix] = useState<[number, number] | null>(null) // [lng, lat]
  const [headingDeg, setHeadingDeg] = useState<number | null>(null)
  const [cov, setCov] = useState<{ xVar: number; yVar: number; yawVar: number } | null>(null)
  const [trail, setTrail] = useState<[number, number][]>([])
  const [baseFix, setBaseFix] = useState<[number, number] | null>(null)
  const [baseHeadingDeg, setBaseHeadingDeg] = useState<number | null>(null)
  const [smoothedPath, setSmoothedPath] = useState<[number, number][]>([])
  const [coveragePath, setCoveragePath] = useState<[number, number][]>([])
  const [objectPose, setObjectPose] = useState<[number, number] | null>(null)
  const smoothedGeoRawRef = useRef<GeoPathMsg | null>(null)
  const coverageGeoRawRef = useRef<GeoPathMsg | null>(null)
  const objectRawRef = useRef<PoseStamped | null>(null)
  const objectStampMsRef = useRef(0)
  const toLLCacheRef = useRef<Map<string, [number, number]>>(new Map())
  const objectReqRef = useRef(0)

  type ToLLRequest = {
    map_point: {
      x: number
      y: number
      z: number
    }
  }

  type ToLLResponse = {
    ll_point?: {
      latitude?: number
      longitude?: number
      altitude?: number
    }
  }

  const toLL = useCallback(
    async (x: number, y: number, z = 0): Promise<[number, number] | null> => {
      const key = `${x},${y},${z}`
      const cached = toLLCacheRef.current.get(key)
      if (cached) return cached
      const ros = getRosBridgeClient()
      if (!ros.isConnected()) return null
      try {
        const res = await ros.callService<ToLLRequest, ToLLResponse>(
          '/toLL',
          'robot_localization/srv/ToLL',
          { map_point: { x, y, z } }
        )
        const lat = Number(res?.ll_point?.latitude)
        const lon = Number(res?.ll_point?.longitude)
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null
        const coord: [number, number] = [lon, lat]
        toLLCacheRef.current.set(key, coord)
        return coord
      } catch {
        return null
      }
    },
    []
  )

  const convertGeoPath = (msg: GeoPathMsg | null, maxPoints: number): [number, number][] => {
    const poses = msg?.poses ?? []
    if (poses.length === 0) return []
    const step =
      poses.length > maxPoints ? Math.ceil(poses.length / maxPoints) : 1
    const coords: [number, number][] = []
    for (let i = 0; i < poses.length; i += step) {
      const position = poses[i]?.pose?.position
      const lat = Number(position?.latitude)
      const lon = Number(position?.longitude)
      if (!Number.isFinite(lat) || !Number.isFinite(lon)) continue
      coords.push([lon, lat])
    }
    return coords
  }

  const convertPose = useCallback(async (msg: PoseStamped | null): Promise<[number, number] | null> => {
    const position = msg?.pose?.position
    const x = Number(position?.x)
    const y = Number(position?.y)
    const z = Number(position?.z ?? 0)
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null
    return toLL(x, y, Number.isFinite(z) ? z : 0)
  }, [toLL])

  const buildMissionPointFeatures = (missions: MissionSpec[]) => {
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

  const buildMissionCircle = (lon: number, lat: number, radius: number) => {
    const latRad = lat * DEG_TO_RAD
    const coords: [number, number][] = []
    for (let i = 0; i <= MISSION_CIRCLE_STEPS; i += 1) {
      const theta = (2 * Math.PI * i) / MISSION_CIRCLE_STEPS
      const dLat = (radius * Math.sin(theta)) / WGS84_A
      const dLon = (radius * Math.cos(theta)) / (WGS84_A * Math.cos(latRad))
      coords.push([lon + dLon * RAD_TO_DEG, lat + dLat * RAD_TO_DEG])
    }
    return coords
  }

  const buildMissionCircleFeatures = useCallback((missions: MissionSpec[]) =>
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
      })), [])

  useEffect(() => {
    if (!mapRef.current) return
    const map = new maplibregl.Map({
      container: mapRef.current,
      style: {
        version: 8,
        sources: {
          imagery: {
            type: 'raster',
            tiles: [
              'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            ],
            tileSize: 256,
            attribution:
              'Sources: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
          },
        },
        layers: [
          {
            id: 'imagery',
            type: 'raster',
            source: 'imagery',
          },
        ],
      },
      center: [0, 0],
      zoom: 2,
    })

    map.addControl(new maplibregl.NavigationControl({ showCompass: false }), 'top-right')

    map.once('load', () => {
      map.addSource('rover-trail', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: { type: 'LineString', coordinates: [] },
          properties: {},
        },
      })
      map.addLayer({
        id: 'rover-trail-line',
        type: 'line',
        source: 'rover-trail',
        paint: {
          'line-color': '#35d3c3',
          'line-width': 3,
          'line-opacity': 0.7,
        },
      })
      map.addSource('smoothed-path', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: { type: 'LineString', coordinates: [] },
          properties: {},
        },
      })
      map.addLayer({
        id: 'smoothed-path-line',
        type: 'line',
        source: 'smoothed-path',
        paint: {
          'line-color': '#4f8ff7',
          'line-width': 2,
          'line-opacity': 0.8,
        },
      })
      map.addSource('coverage-path', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: { type: 'LineString', coordinates: [] },
          properties: {},
        },
      })
      map.addLayer({
        id: 'coverage-path-line',
        type: 'line',
        source: 'coverage-path',
        paint: {
          'line-color': '#f4d35e',
          'line-width': 2,
          'line-opacity': 0.8,
        },
      })
      map.addSource('cover-object', {
        type: 'geojson',
        data: {
          type: 'Feature',
          geometry: { type: 'Point', coordinates: [] },
          properties: {},
        },
      })
      map.addLayer({
        id: 'cover-object-point',
        type: 'circle',
        source: 'cover-object',
        paint: {
          'circle-radius': 5,
          'circle-color': '#ff7a59',
          'circle-stroke-color': '#0b1220',
          'circle-stroke-width': 1.5,
        },
      })
      map.addSource('mission-targets', {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: [],
        },
      })
      map.addLayer({
        id: 'mission-targets-point',
        type: 'circle',
        source: 'mission-targets',
        paint: {
          'circle-radius': 5,
          'circle-color': ['get', 'order_color'],
          'circle-stroke-color': '#0b1220',
          'circle-stroke-width': 1.5,
        },
      })
      map.addSource('mission-radii', {
        type: 'geojson',
        data: {
          type: 'FeatureCollection',
          features: [],
        },
      })
      map.addLayer({
        id: 'mission-radii-fill',
        type: 'fill',
        source: 'mission-radii',
        paint: {
          'fill-color': [
            'match',
            ['get', 'mission_type'],
            1,
            'rgba(53, 211, 195, 0.15)',
            2,
            'rgba(244, 211, 94, 0.18)',
            'rgba(255, 122, 89, 0.18)',
          ],
          'fill-opacity': 0,
        },
      })
      map.addLayer({
        id: 'mission-radii-outline',
        type: 'line',
        source: 'mission-radii',
        paint: {
          'line-color': [
            'match',
            ['get', 'mission_type'],
            1,
            '#35d3c3',
            2,
            '#f4d35e',
            '#ff7a59',
          ],
          'line-width': 1.5,
          'line-opacity': 0.7,
        },
      })
      setMapReady(true)
    })

    map.on('dragstart', () => setFollowRover(false))
    map.on('zoomstart', () => setFollowRover(false))

    mapInstanceRef.current = map

    return () => {
      map.remove()
      mapInstanceRef.current = null
    }
  }, [])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const handler = (event: maplibregl.MapMouseEvent) => {
      if (!grabFromMap || !onGrabCoordinate) return
      onGrabCoordinate({ lat: event.lngLat.lat, lon: event.lngLat.lng })
    }
    map.on('click', handler)
    return () => {
      map.off('click', handler)
    }
  }, [grabFromMap, onGrabCoordinate, mapReady])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map) return
    map.getCanvas().style.cursor = grabFromMap ? 'crosshair' : ''
    return () => {
      if (!map) return
      map.getCanvas().style.cursor = ''
    }
  }, [grabFromMap])

  // Subscribe to GNSS + heading via SiK gateway
  useEffect(() => {
    const sik = getSikGatewayClient()
    sik.connect()
    const unsubscribe = sik.onTelemNav((msg) => {
      if (Number.isFinite(msg.longitude_deg) && Number.isFinite(msg.latitude_deg)) {
        const nextFix: [number, number] = [msg.longitude_deg, msg.latitude_deg]
        setFix(nextFix)
        setTrail((prev) => {
          const last = prev[prev.length - 1]
          if (
            last &&
            Math.abs(last[0] - nextFix[0]) < 1e-6 &&
            Math.abs(last[1] - nextFix[1]) < 1e-6
          ) {
            return prev
          }
          const next = [...prev, nextFix]
          if (next.length > 1200) next.shift()
          return next
        })
      }
      if (Number.isFinite(msg.heading_deg)) {
        setHeadingDeg(msg.heading_deg)
      } else {
        setHeadingDeg(null)
      }
      if (
        Number.isFinite(msg.cov_x_var) &&
        Number.isFinite(msg.cov_y_var) &&
        Number.isFinite(msg.cov_yaw_var)
      ) {
        setCov({
          xVar: msg.cov_x_var,
          yVar: msg.cov_y_var,
          yawVar: msg.cov_yaw_var,
        })
      } else {
        setCov(null)
      }
    })
    return () => unsubscribe()
  }, [])

  useEffect(() => {
    const sik = getSikGatewayClient()
    sik.connect()
    const unsubscribe = sik.onBaseStatus((status) => {
      if (Number.isFinite(status.base_lon_deg) && Number.isFinite(status.base_lat_deg)) {
        setBaseFix([status.base_lon_deg as number, status.base_lat_deg as number])
      }
      if (Number.isFinite(status.antenna_heading_deg)) {
        setBaseHeadingDeg(status.antenna_heading_deg as number)
      } else {
        setBaseHeadingDeg(null)
      }
    })
    return () => unsubscribe()
  }, [])

  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
    const unsubPlanSmoothedGeo = ros.subscribe<GeoPathMsg>(
      '/plan_smoothed/geo',
      'geographic_msgs/msg/GeoPath',
      (msg) => {
        smoothedGeoRawRef.current = msg
        const poseCount = msg?.poses?.length ?? 0
        const coords = convertGeoPath(msg, MAX_SMOOTHED_POINTS)
        if (coords.length > 0 || poseCount === 0) {
          setSmoothedPath(coords)
        }
      },
      { throttleRate: 250 }
    )
    const unsubCoverageGeo = ros.subscribe<GeoPathMsg>(
      '/cover_vision/coverage_path/geo',
      'geographic_msgs/msg/GeoPath',
      (msg) => {
        coverageGeoRawRef.current = msg
        const poseCount = msg?.poses?.length ?? 0
        const coords = convertGeoPath(msg, MAX_COVERAGE_POINTS)
        if (coords.length > 0 || poseCount === 0) {
          setCoveragePath(coords)
        }
      },
      { throttleRate: 250 }
    )

    const getStampMs = (msg: PoseStamped | null): number => {
      const stamp = msg?.header?.stamp
      const sec = Number(stamp?.sec ?? stamp?.secs)
      const nanosec = Number(stamp?.nanosec ?? stamp?.nsecs)
      if (!Number.isFinite(sec)) return 0
      const nanos = Number.isFinite(nanosec) ? nanosec : 0
      return sec * 1000 + nanos / 1e6
    }
    const onObjectPose = (msg: PoseStamped) => {
      const stampMs = getStampMs(msg)
      if (stampMs > 0 && stampMs < objectStampMsRef.current) return
      objectStampMsRef.current = stampMs > 0 ? stampMs : Date.now()
      objectRawRef.current = msg
      const reqId = ++objectReqRef.current
      void convertPose(msg).then((coord: [number, number] | null) => {
        if (reqId !== objectReqRef.current) return
        if (coord) setObjectPose(coord)
      })
    }

    const objectUnsubs = COVER_VISION_OBJECT_TOPICS.map((topic) =>
      ros.subscribe<PoseStamped>(
        topic,
        'geometry_msgs/msg/PoseStamped',
        onObjectPose,
        { throttleRate: 250 }
      )
    )
    return () => {
      unsubPlanSmoothedGeo()
      unsubCoverageGeo()
      objectUnsubs.forEach((unsub) => unsub())
    }
  }, [convertPose])

  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
    const resync = () => {
      if (smoothedGeoRawRef.current) {
        const poseCount = smoothedGeoRawRef.current.poses?.length ?? 0
        const coords = convertGeoPath(smoothedGeoRawRef.current, MAX_SMOOTHED_POINTS)
        if (coords.length > 0 || poseCount === 0) {
          setSmoothedPath(coords)
        }
      }
      if (coverageGeoRawRef.current) {
        const poseCount = coverageGeoRawRef.current.poses?.length ?? 0
        const coords = convertGeoPath(coverageGeoRawRef.current, MAX_COVERAGE_POINTS)
        if (coords.length > 0 || poseCount === 0) {
          setCoveragePath(coords)
        }
      }
      if (objectRawRef.current) {
        const reqId = ++objectReqRef.current
        void convertPose(objectRawRef.current).then((coord: [number, number] | null) => {
          if (reqId !== objectReqRef.current) return
          if (coord) setObjectPose(coord)
        })
      }
    }

    const unsubscribe = ros.onConnectionStatus((connected) => {
      if (!connected) return
      resync()
    })

    if (ros.isConnected()) {
      resync()
    }

    return () => {
      unsubscribe()
    }
  }, [convertPose])

  // Update marker + view when a fix arrives
  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady || !fix) return

    const lngLat: [number, number] = fix
    if (!markerRef.current) {
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
      markerRef.current = new maplibregl.Marker({ element: el, rotationAlignment: 'map' })
        .setLngLat(lngLat)
        .addTo(map)
    } else {
      markerRef.current.setLngLat(lngLat)
    }

    if (followRover) {
      map.easeTo({ center: lngLat, zoom: Math.max(map.getZoom(), 17), duration: 600 })
    }
  }, [fix, mapReady, followRover])

  // Update base marker when a base fix arrives
  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady || !baseFix) return

    if (!baseMarkerRef.current) {
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
      baseMarkerRef.current = new maplibregl.Marker({ element: el, rotationAlignment: 'map' })
        .setLngLat(baseFix)
        .addTo(map)
    } else {
      baseMarkerRef.current.setLngLat(baseFix)
    }
  }, [baseFix, mapReady])

  // Rotate marker when heading updates
  useEffect(() => {
    if (!markerRef.current || headingDeg == null) return
    const rotation = 90 - headingDeg // ENU yaw (0=east, CCW) -> MapLibre rotation (0=north, CW)
    markerRef.current.setRotation(rotation)
  }, [headingDeg])

  // Rotate base marker when antenna heading updates
  useEffect(() => {
    if (!baseMarkerRef.current || baseHeadingDeg == null) return
    baseMarkerRef.current.setRotation(baseHeadingDeg)
  }, [baseHeadingDeg])

  // Push trail to map source
  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const source = map.getSource('rover-trail') as maplibregl.GeoJSONSource | undefined
    if (!source) return
    source.setData({
      type: 'Feature',
      geometry: { type: 'LineString', coordinates: trail },
      properties: {},
    })
  }, [trail, mapReady])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const pointSource = map.getSource('mission-targets') as
      | maplibregl.GeoJSONSource
      | undefined
    if (pointSource) {
      pointSource.setData({
        type: 'FeatureCollection',
        features: buildMissionPointFeatures(missionList),
      })
    }
    const radiusSource = map.getSource('mission-radii') as
      | maplibregl.GeoJSONSource
      | undefined
    if (radiusSource) {
      radiusSource.setData({
        type: 'FeatureCollection',
        features: buildMissionCircleFeatures(missionList),
      })
    }
  }, [buildMissionCircleFeatures, missionList, mapReady])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const source = map.getSource('smoothed-path') as maplibregl.GeoJSONSource | undefined
    if (!source) return
    source.setData({
      type: 'Feature',
      geometry: { type: 'LineString', coordinates: smoothedPath },
      properties: {},
    })
  }, [smoothedPath, mapReady])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const source = map.getSource('coverage-path') as maplibregl.GeoJSONSource | undefined
    if (!source) return
    source.setData({
      type: 'Feature',
      geometry: { type: 'LineString', coordinates: coveragePath },
      properties: {},
    })
  }, [coveragePath, mapReady])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const source = map.getSource('cover-object') as maplibregl.GeoJSONSource | undefined
    if (!source) return
    source.setData({
      type: 'Feature',
      geometry: { type: 'Point', coordinates: objectPose ?? [] },
      properties: {},
    })
  }, [objectPose, mapReady])

  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
      <div className="map" ref={mapRef}>
        <button
          type="button"
          onClick={() => {
            setFollowRover(true)
            const map = mapInstanceRef.current
            if (map && fix) {
              map.easeTo({ center: fix, zoom: Math.max(map.getZoom(), 17), duration: 300 })
            }
          }}
          style={{
            position: 'absolute',
            bottom: 12,
            left: 12,
            zIndex: 2,
            background: followRover ? 'rgba(53, 211, 195, 0.9)' : 'rgba(11, 18, 32, 0.85)',
            color: followRover ? '#0b1220' : '#cdd6f4',
            border: '1px solid rgba(255,255,255,0.12)',
            borderRadius: 10,
            padding: '8px 12px',
            fontSize: '12px',
            cursor: 'pointer',
            boxShadow: '0 10px 25px rgba(0,0,0,0.35)',
          }}
        >
          {followRover ? 'Following rover' : 'Follow rover'}
        </button>
        {grabFromMap ? (
          <div
            style={{
              position: 'absolute',
              bottom: 54,
              left: 12,
              zIndex: 2,
              background: 'rgba(53, 211, 195, 0.9)',
              color: '#0b1220',
              border: '1px solid rgba(255,255,255,0.12)',
              borderRadius: 8,
              padding: '6px 10px',
              fontSize: '12px',
              fontWeight: 600,
              boxShadow: '0 10px 25px rgba(0,0,0,0.35)',
              pointerEvents: 'none',
            }}
          >
            Grab from map enabled. Click to add a mission.
          </div>
        ) : null}
        <div
          style={{
            position: 'absolute',
            top: 8,
            left: 8,
            background: 'rgba(11, 18, 32, 0.8)',
            border: '1px solid rgba(255,255,255,0.08)',
            borderRadius: 8,
            padding: '6px 10px',
            fontSize: '12px',
            color: '#cdd6f4',
            zIndex: 1,
            pointerEvents: 'none',
            minWidth: 170,
          }}
        >
        <div><strong>Lat/Lon:</strong> {fix ? `${fix[1].toFixed(6)}, ${fix[0].toFixed(6)}` : '—'}</div>
        <div><strong>Heading:</strong> {headingDeg != null ? `${headingDeg.toFixed(1)}°` : '—'}</div>
        <div>
          <strong>Std XY:</strong>{' '}
          {cov ? `${Math.sqrt(Math.max(cov.xVar, 0)).toFixed(2)} m, ${Math.sqrt(Math.max(cov.yVar, 0)).toFixed(2)} m` : '—'}
        </div>
        <div>
          <strong>Std yaw:</strong>{' '}
          {cov ? `${(Math.sqrt(Math.max(cov.yawVar, 0)) * (180 / Math.PI)).toFixed(1)}°` : '—'}
        </div>
        <div><strong>Trail pts:</strong> {trail.length}</div>
      </div>
      </div>
    </div>
  )
}

export default MapPreview
