import { useSystemStatusCards } from '../hooks/useSystemStatusCards'
import GnssStatusCard from './GnssStatusCard'
import RocketM2Card from './RocketM2Card'
import SystemStatusCards from './SystemStatusCards'
import './SystemStatusPanel.css'

const SystemStatusPanel = () => {
  const {
    cards,
    baseStatus,
    baseHeadingInput,
    setBaseHeadingInput,
    applyBaseHeading,
  } = useSystemStatusCards()

  return (
    <div className="panel-grid" role="tabpanel">
      <GnssStatusCard />
      <RocketM2Card />
      <article className="card">
        <h3>Link Budget</h3>
        <p>XBEE telemetry strength and latency.</p>
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
      <SystemStatusCards
        cards={cards}
        baseStatus={baseStatus}
        baseHeadingInput={baseHeadingInput}
        onBaseHeadingInputChange={setBaseHeadingInput}
        onApplyBaseHeading={applyBaseHeading}
      />
    </div>
  )
}

export default SystemStatusPanel
