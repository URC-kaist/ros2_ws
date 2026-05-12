import CentrifugeCard from './CentrifugeCard'
import MicroscopeViewCard from './MicroscopeViewCard'
import SpectrophotometerCard from './SpectrophotometerCard'

const ScienceTabPanel = () => {
  return (
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
    </div>
  )
}

export default ScienceTabPanel
