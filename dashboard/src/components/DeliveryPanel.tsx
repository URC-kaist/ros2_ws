import TransitiveVideoCard from './TransitiveVideoCard'
import MapPreview from './MapPreview'

const DeliveryPanel = () => {
  return (
    <div className="panel-grid" role="tabpanel">
      <TransitiveVideoCard
        source="/rgbd_camera/image"
      />
      <div className="card card--span-2 card--map">
        <h3>Rover Position</h3>
        <MapPreview />
      </div>
    </div>
  )
}

export default DeliveryPanel
