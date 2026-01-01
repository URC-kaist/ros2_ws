import { useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import { getRosBridgeClient } from '../lib/rosBridge'

type NavSatFix = {
  latitude: number
  longitude: number
  altitude: number
}

type Quaternion = { x: number; y: number; z: number; w: number }

type Odometry = {
  pose: {
    pose: {
      position: { x: number; y: number; z: number }
      orientation: Quaternion
    }
    covariance: number[]
  }
}

const MapPreview = () => {
  const mapRef = useRef<HTMLDivElement | null>(null)
  const mapInstanceRef = useRef<maplibregl.Map | null>(null)
  const markerRef = useRef<maplibregl.Marker | null>(null)
  const [mapReady, setMapReady] = useState(false)
  const [fix, setFix] = useState<[number, number] | null>(null) // [lng, lat]
  const [headingDeg, setHeadingDeg] = useState<number | null>(null)
  const [cov, setCov] = useState<{ xVar: number; yVar: number; yawVar: number } | null>(null)
  const [trail, setTrail] = useState<[number, number][]>([])

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
      setMapReady(true)
    })

    mapInstanceRef.current = map

    return () => {
      map.remove()
      mapInstanceRef.current = null
    }
  }, [])

  // Subscribe to fused GPS
  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
    const unsubscribe = ros.subscribe<NavSatFix>(
      '/gps/filtered',
      'sensor_msgs/NavSatFix',
      (msg) => {
        if (typeof msg?.longitude === 'number' && typeof msg?.latitude === 'number') {
          setFix([msg.longitude, msg.latitude])
        }
      },
      { throttleRate: 500 }
    )

    return () => {
      unsubscribe()
    }
  }, [])

  // Subscribe to fused odometry for heading + covariance
  useEffect(() => {
    const ros = getRosBridgeClient()
    ros.connect()
    const unsubscribe = ros.subscribe<Odometry>(
      '/odometry/filtered/global',
      'nav_msgs/Odometry',
      (msg) => {
        const q = msg?.pose?.pose?.orientation
        const covArr = msg?.pose?.covariance
        if (q && typeof q.z === 'number' && typeof q.w === 'number') {
          // yaw from quaternion assuming planar motion
          const yaw = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
          setHeadingDeg(((yaw * 180) / Math.PI + 360) % 360)
        }
        if (Array.isArray(covArr) && covArr.length >= 36) {
          setCov({
            xVar: covArr[0],
            yVar: covArr[7],
            yawVar: covArr[35],
          })
        }
      },
      { throttleRate: 200 }
    )
    return () => unsubscribe()
  }, [])

  // Update marker + view when a fix arrives
  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady || !fix) return

    const lngLat: [number, number] = fix
    if (!markerRef.current) {
      const el = document.createElementNS('http://www.w3.org/2000/svg', 'svg')
      el.setAttribute('width', '34')
      el.setAttribute('height', '34')
      el.setAttribute('viewBox', '0 0 34 34')
      el.innerHTML = `
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
      `
      markerRef.current = new maplibregl.Marker({ element: el, rotationAlignment: 'map' })
        .setLngLat(lngLat)
        .addTo(map)
    } else {
      markerRef.current.setLngLat(lngLat)
    }

    map.easeTo({ center: lngLat, zoom: Math.max(map.getZoom(), 17), duration: 600 })
  }, [fix, mapReady])

  // Rotate marker when heading updates
  useEffect(() => {
    if (!markerRef.current || headingDeg == null) return
    // @ts-expect-error maplibre marker rotation typing is looser at runtime
    const rotation = 90 - headingDeg // ENU yaw (0=east, CCW) -> MapLibre rotation (0=north, CW)
    markerRef.current.setRotation(rotation)
  }, [headingDeg])

  // Build trail from successive fixes
  useEffect(() => {
    if (!fix) return
    // eslint-disable-next-line react-hooks/set-state-in-effect
    setTrail((prev) => {
      const last = prev[prev.length - 1]
      if (last && Math.abs(last[0] - fix[0]) < 1e-6 && Math.abs(last[1] - fix[1]) < 1e-6) {
        return prev
      }
      const next = [...prev, fix]
      if (next.length > 1200) next.shift()
      return next
    })
  }, [fix])

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

  return (
    <div className="map" ref={mapRef}>
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
  )
}

export default MapPreview
