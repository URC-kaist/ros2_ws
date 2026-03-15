import type { ReactNode } from 'react'
import VideoStreamCard from './VideoStreamCard'
import { useVideoStreams } from '../hooks/useVideoStreams'

type ConfiguredVideoGridProps = {
  title: string
  panel: string
}

const ConfiguredVideoGrid = ({ title, panel }: ConfiguredVideoGridProps) => {
  const { streams, loading, error } = useVideoStreams(panel)

  let content: ReactNode

  if (loading) {
    content = <div className="video-feed-placeholder">Loading video streams...</div>
  } else if (error) {
    content = <div className="video-feed-placeholder">{error}</div>
  } else if (streams.length === 0) {
    content = <div className="video-feed-placeholder">No streams configured for {panel}</div>
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
    <article className="card video-feed-card">
      <h3>{title}</h3>
      {content}
    </article>
  )
}

export default ConfiguredVideoGrid
