import MapPreview from './MapPreview'
import ArmServoCard from './ArmServoCard'
import ConfiguredVideoGrid from './ConfiguredVideoGrid'

const DeliveryPanel = () => {
  return (
    <div className="delivery-layout" role="tabpanel">
      <ConfiguredVideoGrid title="Live Feed" panel="delivery" />
      <div className="card card--map delivery-map-card">
        <h3>Rover Position</h3>
        <MapPreview />
      </div>
      <ArmServoCard />
    </div>
  )
}

export default DeliveryPanel
