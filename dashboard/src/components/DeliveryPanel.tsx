import TransitiveVideoCard from './TransitiveVideoCard'

const DeliveryPanel = () => {
  return (
    <div className="panel-grid" role="tabpanel">
      <TransitiveVideoCard
        source="/rgbd_camera/image"
      />
    </div>
  )
}

export default DeliveryPanel
