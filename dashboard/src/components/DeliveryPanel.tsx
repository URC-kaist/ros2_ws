import MapPreview from './MapPreview'
import ArmServoCard from './ArmServoCard'

const DeliveryPanel = () => {
  return (
    <div className="delivery-layout" role="tabpanel">
      <div className="card card--map delivery-map-card">
        <h3>Rover Position</h3>
        <MapPreview />
      </div>
      <ArmServoCard />
    </div>
  )
}

export default DeliveryPanel
