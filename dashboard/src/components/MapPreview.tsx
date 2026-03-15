import { useCallback, useState } from 'react'
import 'maplibre-gl/dist/maplibre-gl.css'
import { useMapPreviewData } from '../hooks/useMapPreviewData'
import { useMapPreviewMap } from '../hooks/useMapPreviewMap'
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
  const [followRover, setFollowRover] = useState(true)
  const [basemapMode, setBasemapMode] = useState<'local' | 'esri'>('local')
  const handleFollowRoverChange = useCallback((follow: boolean) => {
    setFollowRover(follow)
  }, [])
  const { mapRef, focusOnCoordinate } = useMapPreviewMap({
    grabFromMap,
    onGrabCoordinate,
    followRover,
    onFollowRoverChange: handleFollowRoverChange,
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
  })

  return (
    <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0 }}>
      <div className="map" ref={mapRef}>
        <button
          type="button"
          onClick={() => {
            setFollowRover(true)
            if (fix) focusOnCoordinate(fix)
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
        <button
          type="button"
          onClick={() => setBasemapMode((prev) => (prev === 'local' ? 'esri' : 'local'))}
          style={{
            position: 'absolute',
            top: 54,
            right: 10,
            zIndex: 2,
            background: 'rgba(11, 18, 32, 0.85)',
            color: '#cdd6f4',
            border: '1px solid rgba(255,255,255,0.12)',
            borderRadius: 10,
            padding: '8px 12px',
            fontSize: '12px',
            cursor: 'pointer',
            boxShadow: '0 10px 25px rgba(0,0,0,0.35)',
          }}
        >
          Basemap: {basemapMode === 'local' ? 'Local' : 'Esri'}
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
