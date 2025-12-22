import './ControlStatusList.css'

const ControlStatusList = () => {
  return (
    <>
      <section className="panel-section">
        <div className="status-list">
          <div className="status-item status-link">
            <span>Link</span>
            <div className="status-pill">
              <span className="status-dot" aria-hidden="true" />
              <strong className="status-good">Stable</strong>
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
            <div className="battery-meta">
              <span>Temp 32°C</span>
              <span>Voltage 24.1V</span>
              <span>Capacity 5.6Ah</span>
            </div>
          </div>
        </div>
      </section>
    </>
  )
}

export default ControlStatusList
