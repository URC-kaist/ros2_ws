import { useEffect, useRef, useState } from 'react'
import maplibregl from 'maplibre-gl'
import {
  buildMissionCircleFeatures,
  buildMissionPointFeatures,
  createBaseMarkerElement,
  createRoverMarkerElement,
  type MapCoordinate,
} from '../lib/mapPreview'
import type { MissionSpec } from '../lib/missions'

const LOCAL_TILE_BASE = '/tiles'

type UseMapPreviewMapOptions = {
  grabFromMap: boolean
  onGrabCoordinate?: (coord: { lat: number; lon: number }) => void
  followRover: boolean
  onFollowRoverChange: (follow: boolean) => void
  fix: MapCoordinate | null
  headingDeg: number | null
  trail: MapCoordinate[]
  baseFix: MapCoordinate | null
  baseHeadingDeg: number | null
  smoothedPath: MapCoordinate[]
  coveragePath: MapCoordinate[]
  objectPose: MapCoordinate | null
  missionList: MissionSpec[]
  basemapMode: 'local' | 'esri'
}

export const useMapPreviewMap = ({
  grabFromMap,
  onGrabCoordinate,
  followRover,
  onFollowRoverChange,
  fix,
  headingDeg,
  trail,
  baseFix,
  baseHeadingDeg,
  smoothedPath,
  coveragePath,
  objectPose,
  missionList,
  basemapMode,
}: UseMapPreviewMapOptions) => {
  const mapRef = useRef<HTMLDivElement | null>(null)
  const mapInstanceRef = useRef<maplibregl.Map | null>(null)
  const markerRef = useRef<maplibregl.Marker | null>(null)
  const baseMarkerRef = useRef<maplibregl.Marker | null>(null)
  const [mapReady, setMapReady] = useState(false)

  useEffect(() => {
    if (!mapRef.current) return
    const map = new maplibregl.Map({
      container: mapRef.current,
      style: {
        version: 8,
        sources: {
          esri: {
            type: 'raster',
            tiles: [
              'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
            ],
            tileSize: 256,
            maxzoom: 18,
            attribution:
              'Sources: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
          },
          local_kaist: {
            type: 'raster',
            tiles: [`${LOCAL_TILE_BASE}/kaist/{z}/{x}/{y}.png`],
            tileSize: 256,
            minzoom: 14,
            maxzoom: 22,
            bounds: [127.35, 36.35, 127.375, 36.375],
          },
          local_pump_track: {
            type: 'raster',
            tiles: [`${LOCAL_TILE_BASE}/pump_track/{z}/{x}/{y}.png`],
            tileSize: 256,
            minzoom: 14,
            maxzoom: 22,
            bounds: [127.35, 36.275, 127.375, 36.3],
          },
          local_naip: {
            type: 'raster',
            tiles: [`${LOCAL_TILE_BASE}/naip/{z}/{x}/{y}.png`],
            tileSize: 256,
            minzoom: 14,
            maxzoom: 22,
          },
        },
        layers: [
          {
            id: 'basemap-esri',
            type: 'raster',
            source: 'esri',
            layout: {
              visibility: 'none',
            },
          },
          {
            id: 'basemap-local-kaist',
            type: 'raster',
            source: 'local_kaist',
          },
          {
            id: 'basemap-local-pump_track',
            type: 'raster',
            source: 'local_pump_track',
          },
          {
            id: 'basemap-local-naip',
            type: 'raster',
            source: 'local_naip',
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

    map.on('dragstart', () => onFollowRoverChange(false))
    map.on('zoomstart', () => onFollowRoverChange(false))

    mapInstanceRef.current = map

    return () => {
      map.remove()
      mapInstanceRef.current = null
    }
  }, [onFollowRoverChange])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const localVisibility = basemapMode === 'local' ? 'visible' : 'none'
    const esriVisibility = basemapMode === 'esri' ? 'visible' : 'none'
    map.setLayoutProperty('basemap-esri', 'visibility', esriVisibility)
    map.setLayoutProperty('basemap-local-kaist', 'visibility', localVisibility)
    map.setLayoutProperty('basemap-local-pump_track', 'visibility', localVisibility)
    map.setLayoutProperty('basemap-local-naip', 'visibility', localVisibility)
  }, [basemapMode, mapReady])

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
  }, [grabFromMap, mapReady, onGrabCoordinate])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map) return
    map.getCanvas().style.cursor = grabFromMap ? 'crosshair' : ''
    return () => {
      map.getCanvas().style.cursor = ''
    }
  }, [grabFromMap])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady || !fix) return

    if (!markerRef.current) {
      markerRef.current = new maplibregl.Marker({
        element: createRoverMarkerElement(),
        rotationAlignment: 'map',
      })
        .setLngLat(fix)
        .addTo(map)
    } else {
      markerRef.current.setLngLat(fix)
    }

    if (followRover) {
      map.easeTo({ center: fix, zoom: Math.max(map.getZoom(), 17), duration: 600 })
    }
  }, [fix, followRover, mapReady])

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

  useEffect(() => {
    if (!markerRef.current || headingDeg == null) return
    markerRef.current.setRotation(90 - headingDeg)
  }, [headingDeg])

  useEffect(() => {
    if (!baseMarkerRef.current || baseHeadingDeg == null) return
    baseMarkerRef.current.setRotation(baseHeadingDeg)
  }, [baseHeadingDeg])

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
  }, [mapReady, trail])

  useEffect(() => {
    const map = mapInstanceRef.current
    if (!map || !mapReady) return
    const pointSource = map.getSource('mission-targets') as maplibregl.GeoJSONSource | undefined
    if (pointSource) {
      pointSource.setData({
        type: 'FeatureCollection',
        features: buildMissionPointFeatures(missionList),
      })
    }
    const radiusSource = map.getSource('mission-radii') as maplibregl.GeoJSONSource | undefined
    if (radiusSource) {
      radiusSource.setData({
        type: 'FeatureCollection',
        features: buildMissionCircleFeatures(missionList),
      })
    }
  }, [mapReady, missionList])

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
  }, [mapReady, smoothedPath])

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
  }, [mapReady, objectPose])

  return {
    mapRef,
    mapReady,
    focusOnCoordinate: (coord: MapCoordinate) => {
      const map = mapInstanceRef.current
      if (!map) return
      map.easeTo({ center: coord, zoom: Math.max(map.getZoom(), 17), duration: 300 })
    },
  }
}
