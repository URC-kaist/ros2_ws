import { useEffect, useState } from 'react'
import { type LinkStatus, type TelemBattery, getSikGatewayClient } from '../../lib/sikGateway'
import './ControlStatusList.css'

const ControlStatusList = () => {
  const isNormal = true
  const [linkStatus, setLinkStatus] = useState<LinkStatus | null>(null)
  const [wsConnected, setWsConnected] = useState(false)
  const [battery, setBattery] = useState<TelemBattery | null>(null)

  useEffect(() => {
    const gateway = getSikGatewayClient()
    gateway.connect()
    const offLink = gateway.onLinkStatus(setLinkStatus)
    const offConnection = gateway.onConnectionStatus(setWsConnected)
    const offBattery = gateway.onTelemBattery(setBattery)
    return () => {
      offLink()
      offConnection()
      offBattery()
    }
  }, [])

  const batteryPercent =
    battery && battery.total_capacity_mah > 0
      ? Math.max(
          0,
          Math.min(
            100,
            (battery.available_capacity_mah / battery.total_capacity_mah) * 100
          )
        )
      : 0
  let linkState = 'Down'
  let linkDotClass = 'status-dot-error'
  if (wsConnected) {
    if (linkStatus?.connected) {
      linkState = 'Up'
      linkDotClass = ''
    } else {
      linkState = 'Lost'
      linkDotClass = 'status-dot-warn'
    }
  }

  return (
    <>
      <section className="panel-section">
        <div className="status-list">
        <div className="status-item status-link">
          <span className="status-label">
            <span className={`status-dot ${linkDotClass}`} aria-hidden="true" />
            SiK Link
          </span>
          <div className="status-pill">
            <strong>{linkState}</strong>
          </div>
        </div>
        <div className="status-item status-softstop">
          <span className="status-label">
            <span className={`status-dot ${isNormal ? '' : 'status-dot-error'}`} aria-hidden="true" />
            Status
          </span>
          <div className="status-pill">
            <strong>{isNormal ? 'Normal' : 'E-Stop'}</strong>
          </div>
        </div>
      </div>
    </section>
      <section className="panel-section">
        <div className="status-list">
          <div className="status-item battery">
            <div className="metric-label">
              <span>Battery</span>
              <strong>{battery ? `${batteryPercent.toFixed(0)}%` : '---'}</strong>
            </div>
            <div className="meter">
              <div className="meter-fill good" style={{ width: `${batteryPercent}%` }} />
            </div>
            <div className="battery-meta">
              <span>
                Temp {battery ? `${battery.temperature_c.toFixed(1)}°C` : '--'}
              </span>
              <span>Voltage --</span>
              <span>
                Capacity {battery ? `${(battery.total_capacity_mah / 1000).toFixed(1)}Ah` : '--'}
              </span>
            </div>
          </div>
        </div>
      </section>
    </>
  )
}

export default ControlStatusList
