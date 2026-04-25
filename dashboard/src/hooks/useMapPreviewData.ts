import { useCallback, useEffect, useRef, useState } from 'react'
import { useRosBridge } from './useRosBridge'
import { useXbeeGateway } from './useXbeeGateway'
import {
  appendTrailPoint,
  convertGeoPath,
  COVER_VISION_OBJECT_TOPICS,
  getPoseStampMs,
  MAX_COVERAGE_POINTS,
  MAX_SMOOTHED_POINTS,
  type MapCoordinate,
  type NavCovariance,
} from '../lib/mapPreview'
import type { GeoPathMsg, PoseStamped, ToLLRequest, ToLLResponse } from '../lib/rosMessages'

export const useMapPreviewData = () => {
  const { ros, connected: rosConnected } = useRosBridge()
  const { gateway } = useXbeeGateway()
  const [fix, setFix] = useState<MapCoordinate | null>(null)
  const [headingDeg, setHeadingDeg] = useState<number | null>(null)
  const [cov, setCov] = useState<NavCovariance | null>(null)
  const [trail, setTrail] = useState<MapCoordinate[]>([])
  const [baseFix, setBaseFix] = useState<MapCoordinate | null>(null)
  const [baseHeadingDeg, setBaseHeadingDeg] = useState<number | null>(null)
  const [smoothedPath, setSmoothedPath] = useState<MapCoordinate[]>([])
  const [coveragePath, setCoveragePath] = useState<MapCoordinate[]>([])
  const [objectPose, setObjectPose] = useState<MapCoordinate | null>(null)
  const smoothedGeoRawRef = useRef<GeoPathMsg | null>(null)
  const coverageGeoRawRef = useRef<GeoPathMsg | null>(null)
  const objectRawRef = useRef<PoseStamped | null>(null)
  const objectStampMsRef = useRef(0)
  const toLLCacheRef = useRef<Map<string, MapCoordinate>>(new Map())
  const objectReqRef = useRef(0)

  const toLL = useCallback(
    async (x: number, y: number, z = 0): Promise<MapCoordinate | null> => {
      const key = `${x},${y},${z}`
      const cached = toLLCacheRef.current.get(key)
      if (cached) return cached
      if (!rosConnected) return null
      try {
        const res = await ros.callService<ToLLRequest, ToLLResponse>(
          '/toLL',
          'robot_localization/srv/ToLL',
          { map_point: { x, y, z } }
        )
        const lat = Number(res?.ll_point?.latitude)
        const lon = Number(res?.ll_point?.longitude)
        if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null
        const coord: MapCoordinate = [lon, lat]
        toLLCacheRef.current.set(key, coord)
        return coord
      } catch {
        return null
      }
    },
    [ros, rosConnected]
  )

  const convertPose = useCallback(
    async (msg: PoseStamped | null): Promise<MapCoordinate | null> => {
      const position = msg?.pose?.position
      const x = Number(position?.x)
      const y = Number(position?.y)
      const z = Number(position?.z ?? 0)
      if (!Number.isFinite(x) || !Number.isFinite(y)) return null
      return toLL(x, y, Number.isFinite(z) ? z : 0)
    },
    [toLL]
  )

  useEffect(() => {
    const unsubscribe = gateway.onTelemNav((msg) => {
      if (Number.isFinite(msg.longitude_deg) && Number.isFinite(msg.latitude_deg)) {
        const nextFix: MapCoordinate = [msg.longitude_deg, msg.latitude_deg]
        setFix(nextFix)
        setTrail((prev) => appendTrailPoint(prev, nextFix))
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
  }, [gateway])

  useEffect(() => {
    const unsubscribe = gateway.onBaseStatus((status) => {
      if (Number.isFinite(status.base_lon_deg) && Number.isFinite(status.base_lat_deg)) {
        setBaseFix([status.base_lon_deg as number, status.base_lat_deg as number])
      }
      if (Number.isFinite(status.antenna_heading_deg)) {
        setBaseHeadingDeg(status.antenna_heading_deg)
      } else {
        setBaseHeadingDeg(null)
      }
    })
    return () => unsubscribe()
  }, [gateway])

  useEffect(() => {
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

    const onObjectPose = (msg: PoseStamped) => {
      const stampMs = getPoseStampMs(msg)
      if (stampMs > 0 && stampMs < objectStampMsRef.current) return
      objectStampMsRef.current = stampMs > 0 ? stampMs : Date.now()
      objectRawRef.current = msg
      const reqId = ++objectReqRef.current
      void convertPose(msg).then((coord) => {
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
  }, [convertPose, ros])

  useEffect(() => {
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
        void convertPose(objectRawRef.current).then((coord) => {
          if (reqId !== objectReqRef.current) return
          if (coord) setObjectPose(coord)
        })
      }
    }

    const unsubscribe = ros.onConnectionStatus((connected) => {
      if (!connected) return
      resync()
    })

    if (rosConnected) {
      resync()
    }

    return () => {
      unsubscribe()
    }
  }, [convertPose, ros, rosConnected])

  return {
    fix,
    headingDeg,
    cov,
    trail,
    baseFix,
    baseHeadingDeg,
    smoothedPath,
    coveragePath,
    objectPose,
  }
}
