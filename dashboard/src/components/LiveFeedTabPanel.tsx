import ConfiguredVideoGrid from './ConfiguredVideoGrid'

const LiveFeedTabPanel = () => {
  return (
    <div className="live-feed-panel" role="tabpanel">
      <ConfiguredVideoGrid title="Live Feed" large showReconnect />
    </div>
  )
}

export default LiveFeedTabPanel
