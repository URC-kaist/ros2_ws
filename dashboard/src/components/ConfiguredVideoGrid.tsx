import type { ReactNode } from 'react'
import { FiRefreshCw } from 'react-icons/fi'
import VideoStreamCard from './VideoStreamCard'
import { useVideoStreams } from '../hooks/useVideoStreams'
import { getVideoGatewayClient } from '../lib/videoGateway'

type ConfiguredVideoGridProps = {
  title: string
  panel?: string
  large?: boolean
  showReconnect?: boolean
}

const ConfiguredVideoGrid = ({
  title,
  panel,
  large = false,
  showReconnect = false,
}: ConfiguredVideoGridProps) => {
  const { streams, loading, error, refresh } = useVideoStreams(panel)

  const handleReconnect = () => {
    getVideoGatewayClient().reconnect()
    refresh()
  }

  let content: ReactNode

  if (loading) {
    content = <div className="video-feed-placeholder">Loading video streams...</div>
  } else if (error) {
    content = <div className="video-feed-placeholder">{error}</div>
  } else if (streams.length === 0) {
    content = (
      <div className="video-feed-placeholder">
        No streams configured{panel ? ` for ${panel}` : ''}
      </div>
    )
  } else {
    content = (
      <div className="video-feed-grid">
        {streams.map((stream) => (
          <div className="video-feed-item" key={stream.stream_id}>
            <span className="video-feed-label">
              {stream.display.label || stream.stream_id}
            </span>
            <VideoStreamCard
              stream={stream}
              videoWidth={stream.width ?? 320}
              videoHeight={stream.height ?? 180}
            />
          </div>
        ))}
      </div>
    )
  }

  return (
    <article className={`video-feed-card ${large ? 'video-feed-card--large' : 'card'}`}>
      <header className="video-feed-header">
        <h3>{title}</h3>
        {showReconnect && (
          <button
            className="video-feed-reconnect"
            type="button"
            aria-label="Reconnect live feed"
            title="Reconnect live feed"
            onClick={handleReconnect}
          >
            <FiRefreshCw aria-hidden="true" />
            <span>Reconnect</span>
          </button>
        )}
      </header>
      {content}
    </article>
  )
}

export default ConfiguredVideoGrid
