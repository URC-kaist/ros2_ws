import MapPreview from './MapPreview'
import ArmServoCard from './ArmServoCard'
import { isRoverDirectProfile } from '../lib/operatingProfile'

const DeliveryPanel = () => {
  return (
    <div className="delivery-layout" role="tabpanel">
      {!isRoverDirectProfile && (
        <div className="card card--map delivery-map-card">
          <h3>Rover Position</h3>
          <MapPreview />
        </div>
      )}
      <ArmServoCard />
    </div>
  )
}

export default DeliveryPanel
