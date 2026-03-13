import { useCallback, useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'
import { useMapPreviewData } from '../hooks/useMapPreviewData'
import {
  buildMissionCircleFeatures,
  buildMissionPointFeatures,
  createBaseMarkerElement,
  createRoverMarkerElement,
  type MapCoordinate,
} from '../lib/mapPreview'
import type { MissionSpec } from '../lib/missions'

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
  const {
    fix,
    headingDeg,
    cov,
    trail,
    baseFix,
    baseHeadingDeg,
    smoothedPath,
    coveragePath,
    objectPose,
  } = useMapPreviewData()
  const mapRef = useRef<HTMLDivElement | null>(null)
  const mapInstanceRef = useRef<maplibregl.Map | null>(null)
  const markerRef = useRef<maplibregl.Marker | null>(null)
  const baseMarkerRef = useRef<maplibregl.Marker | null>(null)
  const [mapReady, setMapReady] = useState(false)
  const [followRover, setFollowRover] = useState(true)
  const missionPointFeatures = useCallback(
    () => buildMissionPointFeatures(missionList),
    [missionList]
  )
  const missionCircleFeatures = useCallback(
    () => buildMissionCircleFeatures(missionList),
    [missionList]
  )

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

  // Update marker + view when a fix arrives
  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady || !fix) return

    const lngLat: MapCoordinate = fix
    if (!markerRef.current) {
      markerRef.current = new maplibregl.Marker({
        element: createRoverMarkerElement(),
        rotationAlignment: 'map',
      })
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
      baseMarkerRef.current = new maplibregl.Marker({
        element: createBaseMarkerElement(),
        rotationAlignment: 'map',
      })
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
        features: missionPointFeatures(),
      })
    }
    const radiusSource = map.getSource('mission-radii') as
      | maplibregl.GeoJSONSource
      | undefined
    if (radiusSource) {
      radiusSource.setData({
        type: 'FeatureCollection',
        features: missionCircleFeatures(),
      })
    }
  }, [mapReady, missionCircleFeatures, missionPointFeatures])

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
