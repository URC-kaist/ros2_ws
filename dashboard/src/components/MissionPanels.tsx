import './MissionPanels.css'

type MissionPanelsProps = {
  activeTab: string
}

const MissionPanels = ({ activeTab }: MissionPanelsProps) => {
  return (
    <section className="tab-panels">
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
          <article className="card">
            <h3>Spectrometer</h3>
            <p>Live spectrum capture and anomaly detection.</p>
            <div className="sparkline" />
          </article>
          <article className="card">
            <h3>Thermal Map</h3>
            <p>Hotspot overlay from thermal camera feed.</p>
            <div className="heatmap" />
          </article>
        </div>
      )}

      {activeTab === 'autonomous' && (
        <div className="panel-grid" role="tabpanel">
          <article className="card">
            <h3>Route Planner</h3>
            <p>Waypoint edits, hazard avoidance, and ETA.</p>
            <div className="map" />
          </article>
          <article className="card">
            <h3>Autonomy Health</h3>
            <p>Planner status, localization, and perception.</p>
            <ul className="list">
              <li>Localization: Green</li>
              <li>Planner: Green</li>
              <li>Perception: Yellow</li>
            </ul>
          </article>
        </div>
      )}

      {activeTab === 'manipulation' && (
        <div className="panel-grid" role="tabpanel">
          <article className="card">
            <h3>Arm Pose</h3>
            <p>Joint targets and end-effector orientation.</p>
            <div className="pose-grid">
              <div>
                <span>Joint 1</span>
                <strong>32 deg</strong>
              </div>
              <div>
                <span>Joint 2</span>
                <strong>18 deg</strong>
              </div>
              <div>
                <span>Joint 3</span>
                <strong>44 deg</strong>
              </div>
            </div>
          </article>
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
