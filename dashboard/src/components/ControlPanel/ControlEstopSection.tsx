import { useXbeeGateway } from '../../hooks/useXbeeGateway'
import './ControlEstopSection.css'

const ControlEstopSection = () => {
  const { gateway } = useXbeeGateway()

  const handleEstop = () => {
    gateway.sendMissionControl({
      command: 3,
      clear_costmap: true,
      mission_id: 0,
    })
  }

  return (
    <section className="panel-section estop-section">
      <button className="btn danger estop" type="button" onClick={handleEstop}>
        E-Stop
      </button>
    </section>
  )
}

export default ControlEstopSection
