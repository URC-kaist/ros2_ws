import type { ReactNode } from 'react'
import { useCallback, useState } from 'react'
import { FiRefreshCw } from 'react-icons/fi'
import VideoStreamCard from './VideoStreamCard'
import { useRosBridge } from '../hooks/useRosBridge'
import { useVideoStreams } from '../hooks/useVideoStreams'
import { getVideoGatewayClient, type VideoStreamInfo } from '../lib/videoGateway'

type ConfiguredVideoGridProps = {
  title: string
  panel?: string
  large?: boolean
  showReconnect?: boolean
  showStreamControls?: boolean
}

type SetBoolRequest = {
  data: boolean
}

type SetBoolResponse = {
  success: boolean
  message?: string
}

const ConfiguredVideoGrid = ({
  title,
  panel,
  large = false,
  showReconnect = false,
  showStreamControls = false,
}: ConfiguredVideoGridProps) => {
  const { ros, connected: rosConnected } = useRosBridge()
  const { streams, loading, error, refresh } = useVideoStreams(panel)
  const [enabledOverrides, setEnabledOverrides] = useState<Record<string, boolean>>({})
  const [pendingStreams, setPendingStreams] = useState<Set<string>>(() => new Set())
  const [toggleError, setToggleError] = useState<string | null>(null)

  const handleReconnect = () => {
    getVideoGatewayClient().reconnect()
    refresh()
  }

  const setStreamPending = useCallback((streamId: string, pending: boolean) => {
    setPendingStreams((current) => {
      const next = new Set(current)
      if (pending) {
        next.add(streamId)
      } else {
        next.delete(streamId)
      }
      return next
    })
  }, [])

  const handleToggleStream = useCallback(
    async (streamId: string) => {
      const currentlyEnabled = enabledOverrides[streamId] ?? true
      const nextEnabled = !currentlyEnabled
      setStreamPending(streamId, true)
      setToggleError(null)

      try {
        const response = await ros.callService<SetBoolRequest, SetBoolResponse>(
          `/video_streaming/streams/${streamId}/set_enabled`,
          'std_srvs/srv/SetBool',
          { data: nextEnabled }
        )
        if (!response.success) {
          throw new Error(response.message || 'Stream toggle failed')
        }
        setEnabledOverrides((current) => ({
          ...current,
          [streamId]: nextEnabled,
        }))
        getVideoGatewayClient().reconnect()
        refresh()
      } catch (err) {
        setToggleError(err instanceof Error ? err.message : 'Stream toggle failed')
      } finally {
        setStreamPending(streamId, false)
      }
    },
    [enabledOverrides, refresh, ros, setStreamPending]
  )

  const renderStream = (stream: VideoStreamInfo) => {
    const isEnabled = enabledOverrides[stream.stream_id] !== false
    const pending = pendingStreams.has(stream.stream_id)

    return (
      <div className="video-feed-item" key={stream.stream_id}>
        <div className="video-feed-item-header">
          <span className="video-feed-label">
            {stream.display.label || stream.stream_id}
          </span>
          {showStreamControls && (
            <button
              className="video-feed-stream-toggle"
              type="button"
              disabled={!rosConnected || pending}
              onClick={() => void handleToggleStream(stream.stream_id)}
            >
              {isEnabled ? 'Disable' : 'Enable'}
            </button>
          )}
        </div>
        <VideoStreamCard
          stream={stream}
          videoWidth={stream.width ?? 320}
          videoHeight={stream.height ?? 180}
        />
      </div>
    )
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
        {streams.map(renderStream)}
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
      {showStreamControls && toggleError && (
        <div className="video-feed-toggle-error">{toggleError}</div>
      )}
      {content}
    </article>
  )
}

export default ConfiguredVideoGrid
