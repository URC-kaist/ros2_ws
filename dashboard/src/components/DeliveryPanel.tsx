import TransitiveVideoCard from './TransitiveVideoCard'
import MapPreview from './MapPreview'
import ArmServoCard from './ArmServoCard'

const DeliveryPanel = () => {
  return (
    <div className="panel-grid" role="tabpanel">
      <TransitiveVideoCard
        source="/rgbd_camera/image"
        videoWidth={640}
        videoHeight={480}
      />
      <TransitiveVideoCard
        title="Top Camera"
        description="Live feed via V4L2."
        source="/dev/video6"
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
      <ArmServoCard />
      <div className="card card--span-2 card--map">
        <h3>Rover Position</h3>
        <MapPreview />
      </div>
    </div>
  )
}

export default DeliveryPanel
