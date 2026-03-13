import { useState } from 'react'
import DeliveryPanel from './DeliveryPanel'
import MapPreview from './MapPreview'
import MissionMasterPanel from './MissionMasterPanel'
import type { MissionTabId } from './MissionTabs'
import SystemStatusPanel from './SystemStatusPanel'
import SpectrophotometerCard from './SpectrophotometerCard'
import AutonomyHealthCard from './AutonomyHealthCard'
import TransitiveVideoCard from './TransitiveVideoCard'
import CameraTurretCard from './CameraTurretCard'
import CentrifugeCard from './CentrifugeCard'
import MicroscopeViewCard from './MicroscopeViewCard'
import { createMissionSpec, type MissionSpec } from '../lib/missions'
import './MissionPanels.css'

type MissionPanelsProps = {
  activeTab: MissionTabId
}

const MissionPanels = ({ activeTab }: MissionPanelsProps) => {
  const [missionList, setMissionList] = useState<MissionSpec[]>([])
  const [previewMissions, setPreviewMissions] = useState<MissionSpec[]>([])
  const [grabFromMap, setGrabFromMap] = useState(false)

  const handleGrabFromMap = ({ lat, lon }: { lat: number; lon: number }) => {
    setMissionList((prev) => {
      const nextId =
        prev.reduce(
          (max, mission) =>
            Number.isFinite(mission.mission_id) ? Math.max(max, mission.mission_id) : max,
          0
        ) + 1
      return [
        ...prev,
        createMissionSpec(nextId, {
          target_latitude: lat,
          target_longitude: lon,
        }),
      ]
    })
  }

  const missionsForMap = grabFromMap ? missionList : previewMissions
  return (
    <section className="tab-panels">
      {activeTab === 'status' && <SystemStatusPanel />}
      {activeTab === 'delivery' && <DeliveryPanel />}
      {activeTab === 'science' && (
        <div className="science-layout" role="tabpanel">
          <div className="science-column">
            <article className="card module-progress-card">
              <h3>Module Progress</h3>
              <div className="module-progress">
                <div className="module-step">
                  <span className="module-dot" />
                  <span>Boring</span>
                </div>
                <div className="module-step">
                  <span className="module-dot" />
                  <span>Cache</span>
                </div>
                <div className="module-step">
                  <span className="module-dot" />
                  <span>Pump</span>
                </div>
                <div className="module-step">
                  <span className="module-dot" />
                  <span>Centrifuge</span>
                </div>
                <div className="module-step">
                  <span className="module-dot" />
                  <span>VIS</span>
                </div>
              </div>
            </article>
            <CameraTurretCard />
            <div className="science-metrics">
              <article className="card metric-card">
                <h3>Temperature</h3>
                <div className="metric-value">-- °C</div>
              </article>
              <article className="card metric-card">
                <h3>Humidity</h3>
                <div className="metric-value">-- %</div>
              </article>
            </div>
            <MicroscopeViewCard />
          </div>
          <div className="science-column">
            <CentrifugeCard />
            <SpectrophotometerCard />
          </div>
          <div className="science-column">
            <article className="card video-feed-card science-camera-bank">
              <h3>Module Cameras</h3>
              <div className="video-feed-grid">
                <div className="video-feed-item">
                  <span className="video-feed-label">Boring Camera</span>
                  <div className="video-feed-placeholder video-feed-placeholder--compact">
                    Awaiting stream...
                  </div>
                </div>
                <div className="video-feed-item">
                  <span className="video-feed-label">Pump Camera</span>
                  <div className="video-feed-placeholder video-feed-placeholder--compact">
                    Awaiting stream...
                  </div>
                </div>
                <div className="video-feed-item">
                  <span className="video-feed-label">Cache Camera</span>
                  <div className="video-feed-placeholder video-feed-placeholder--compact">
                    Awaiting stream...
                  </div>
                </div>
                <div className="video-feed-item">
                  <span className="video-feed-label">Centrifuge Camera</span>
                  <div className="video-feed-placeholder video-feed-placeholder--compact">
                    Awaiting stream...
                  </div>
                </div>
              </div>
            </article>
          </div>
        </div>
      )}

      {activeTab === 'autonomous' && (
        <div className="autonomy-layout" role="tabpanel">
          <div className="autonomy-left">
            <article className="card card--map">
              <MapPreview
                missionList={missionsForMap}
                grabFromMap={grabFromMap}
                onGrabCoordinate={handleGrabFromMap}
              />
            </article>
            <article className="card video-feed-card">
              <h3>Vision</h3>
              <div className="video-feed-grid">
                <div className="video-feed-item">
                  <span className="video-feed-label">ArUco</span>
                  <TransitiveVideoCard
                    embedded
                    source="/aruco_tracker/debug"
                    videoWidth={320}
                    videoHeight={180}
                  />
                </div>
                <div className="video-feed-item">
                  <span className="video-feed-label">YOLO</span>
                  <TransitiveVideoCard
                    embedded
                    source="/yolo/annotated_image"
                    videoWidth={320}
                    videoHeight={180}
                  />
                </div>
              </div>
            </article>
          </div>
          <div className="autonomy-right">
            <AutonomyHealthCard />
            <MissionMasterPanel
              missionList={missionList}
              grabFromMap={grabFromMap}
              onGrabFromMapChange={setGrabFromMap}
              onMissionListChange={setMissionList}
              onMissionPreview={setPreviewMissions}
            />
          </div>
        </div>
      )}

    </section>
  )
}

export default MissionPanels
