import TransitiveVideoCard from './TransitiveVideoCard'
import MapPreview from './MapPreview'
import ArmServoCard from './ArmServoCard'

const DeliveryPanel = () => {
  return (
    <div className="delivery-layout" role="tabpanel">
      <article className="card video-feed-card">
        <h3>Live Feed</h3>
        <div className="video-feed-grid">
          <div className="video-feed-item">
            <span className="video-feed-label">/dev/video0</span>
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
          <div className="video-feed-item">
            <span className="video-feed-label">/dev/video8</span>
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
      <div className="card card--map delivery-map-card">
        <h3>Rover Position</h3>
        <MapPreview />
      </div>
      <ArmServoCard />
    </div>
  )
}

export default DeliveryPanel
