import './ControlPanel.css'

const ControlPanel = () => {
  return (
    <aside className="control-panel">
      <div className="panel-header">
        <div className="badge">MR2</div>
        <p className="panel-title">Control</p>
      </div>

      <section className="panel-section">
        <div className="control-metrics">
          <div className="metric">
            <div className="metric-label">
              <span>Cmd Vel</span>
              <strong>0.42 m/s</strong>
            </div>
            <div className="meter">
              <div className="meter-fill" style={{ width: '42%' }} />
            </div>
          </div>
          <div className="metric">
            <div className="metric-label">
              <span>Cmd Ang Vel</span>
              <strong>0.12 rad/s</strong>
            </div>
            <div className="meter">
              <div className="meter-fill accent" style={{ width: '24%' }} />
            </div>
          </div>
        </div>
      </section>

      <section className="panel-section">
        <div className="status-list">
          <div className="status-item battery">
            <div className="metric-label">
              <span>Battery</span>
              <strong>78%</strong>
            </div>
            <div className="meter">
              <div className="meter-fill good" style={{ width: '78%' }} />
            </div>
          </div>
          <div className="status-item">
            <span>Link</span>
            <strong className="status-good">Stable</strong>
          </div>
        </div>
      </section>

      <section className="panel-section estop-section">
        <button className="btn danger estop">E-Stop</button>
      </section>
    </aside>
  )
}

export default ControlPanel
