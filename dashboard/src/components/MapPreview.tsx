import { useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import { getSikGatewayClient } from '../lib/sikGateway'

const MapPreview = () => {
  const mapRef = useRef<HTMLDivElement | null>(null)
  const mapInstanceRef = useRef<maplibregl.Map | null>(null)
  const markerRef = useRef<maplibregl.Marker | null>(null)
  const [mapReady, setMapReady] = useState(false)
  const [followRover, setFollowRover] = useState(true)
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

    map.on('dragstart', () => setFollowRover(false))
    map.on('zoomstart', () => setFollowRover(false))

    mapInstanceRef.current = map

    return () => {
      map.remove()
      mapInstanceRef.current = null
    }
  }, [])

  // Subscribe to GNSS + heading via SiK gateway
  useEffect(() => {
    const sik = getSikGatewayClient()
    sik.connect()
    const unsubscribe = sik.onTelemNav((msg) => {
      if (Number.isFinite(msg.longitude_deg) && Number.isFinite(msg.latitude_deg)) {
        setFix([msg.longitude_deg, msg.latitude_deg])
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

    if (followRover) {
      map.easeTo({ center: lngLat, zoom: Math.max(map.getZoom(), 17), duration: 600 })
    }
  }, [fix, mapReady, followRover])

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
          top: 10,
          right: 10,
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
