import TransitiveVideoCard from './TransitiveVideoCard'
import MapPreview from './MapPreview'
import ArmServoCard from './ArmServoCard'

const DeliveryPanel = () => {
  return (
    <div className="panel-grid" role="tabpanel">
      <article className="card card--span-2 delivery-live-feed-card">
        <h3>Live Feed</h3>
        <div className="delivery-live-feed-grid">
          <div className="delivery-live-feed-item">
            <span className="delivery-live-feed-label">/dev/video0</span>
            <TransitiveVideoCard
              embedded
              source="/dev/video0"
              type="v4l2src"
              streamtype="image/jpeg"
              framerate="15/1"
              width="320"
              height="180"
              quantizer="25"
              timeout="1800"
              count="1"
              videoWidth={320}
              videoHeight={180}
            />
          </div>
          <div className="delivery-live-feed-item">
            <span className="delivery-live-feed-label">/dev/video8</span>
            <TransitiveVideoCard
              embedded
              source="/dev/video8"
              type="v4l2src"
              streamtype="image/jpeg"
              framerate="15/1"
              width="320"
              height="180"
              quantizer="25"
              timeout="1800"
              count="1"
              videoWidth={320}
              videoHeight={180}
            />
          </div>
        </div>
      </article>
      <div className="card card--span-2 card--map">
        <h3>Rover Position</h3>
        <MapPreview />
      </div>
      <ArmServoCard />
    </div>
  )
}

export default DeliveryPanel
