import { useState } from 'react'
import DeliveryPanel from './DeliveryPanel'
import MapPreview from './MapPreview'
import MissionMasterPanel from './MissionMasterPanel'
import type { MissionSpec } from './MapPreview'
import SystemStatusPanel from './SystemStatusPanel'
import ArmServoCard from './ArmServoCard'
import RocketM2Card from './RocketM2Card'
import SpectrophotometerCard from './SpectrophotometerCard'
import AutonomyHealthCard from './AutonomyHealthCard'
import './MissionPanels.css'

type MissionPanelsProps = {
  activeTab: string
}

const MissionPanels = ({ activeTab }: MissionPanelsProps) => {
  const [missionList, setMissionList] = useState<MissionSpec[]>([])
  const [previewMissions, setPreviewMissions] = useState<MissionSpec[]>([])
  return (
    <section className="tab-panels">
      {activeTab === 'status' && <SystemStatusPanel />}
      {activeTab === 'delivery' && <DeliveryPanel />}
      {activeTab === 'science' && (
        <div className="panel-grid" role="tabpanel">
          <article className="card">
            <h3>Sample Queue</h3>
            <p>Track collection, labeling, and storage metadata.</p>
            <div className="pill-row">
              <span className="pill">SOIL-07</span>
              <span className="pill">ICE-03</span>
              <span className="pill">ROCK-11</span>
            </div>
          </article>
          <SpectrophotometerCard />
          <article className="card">
            <h3>Thermal Map</h3>
            <p>Hotspot overlay from thermal camera feed.</p>
            <div className="heatmap" />
          </article>
        </div>
      )}

      {activeTab === 'autonomous' && (
        <div className="autonomy-layout" role="tabpanel">
          <div className="autonomy-left">
            <article className="card card--map">
              <MapPreview missionList={previewMissions} />
            </article>
            <AutonomyHealthCard />
          </div>
          <div className="autonomy-right">
            <MissionMasterPanel
              missionList={missionList}
              onMissionListChange={setMissionList}
              onMissionPreview={setPreviewMissions}
            />
          </div>
        </div>
      )}

      {activeTab === 'manipulation' && (
        <div className="panel-grid" role="tabpanel">
          <ArmServoCard />
          <article className="card">
            <h3>Tool Status</h3>
            <p>Gripper, drill, and wrist telemetry.</p>
            <div className="pill-row">
              <span className="pill">Gripper: Closed</span>
              <span className="pill">Drill: Idle</span>
              <span className="pill">Wrist: 12 deg</span>
            </div>
          </article>
        </div>
      )}

      {activeTab === 'navigation' && (
        <div className="panel-grid" role="tabpanel">
          <article className="card">
            <h3>Terrain View</h3>
            <p>LiDAR elevation slices and slope alerts.</p>
            <div className="terrain" />
          </article>
          <article className="card">
            <h3>Pose Estimator</h3>
            <p>Position, heading, and drift metrics.</p>
            <div className="status-list">
              <div className="status-item">
                <span>Heading</span>
                <strong>112 deg</strong>
              </div>
              <div className="status-item">
                <span>Drift</span>
                <strong>0.09 m</strong>
              </div>
              <div className="status-item">
                <span>Sat Fix</span>
                <strong className="status-warn">3D Fix</strong>
              </div>
            </div>
          </article>
        </div>
      )}

      {activeTab === 'comms' && (
        <div className="panel-grid" role="tabpanel">
          <RocketM2Card />
          <article className="card">
            <h3>Link Budget</h3>
            <p>SiK telemetry strength and latency.</p>
            <div className="status-list">
              <div className="status-item">
                <span>RSSI</span>
                <strong>62 dBm</strong>
              </div>
              <div className="status-item">
                <span>Latency</span>
                <strong>180 ms</strong>
              </div>
              <div className="status-item">
                <span>Packet Loss</span>
                <strong className="status-warn">2.1%</strong>
              </div>
            </div>
          </article>
          <article className="card">
            <h3>Event Stream</h3>
            <p>Command acknowledgements and alerts.</p>
            <ul className="list">
              <li>Autonomy plan synced.</li>
              <li>New waypoint uploaded.</li>
              <li>Telemetry: nominal.</li>
            </ul>
          </article>
        </div>
      )}
    </section>
  )
}

export default MissionPanels
