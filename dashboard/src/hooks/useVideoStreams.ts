import { useEffect, useState } from 'react'
import { fetchVideoStreams, type VideoStreamInfo } from '../lib/videoGateway'

export function useVideoStreams(panel?: string) {
  const [streams, setStreams] = useState<VideoStreamInfo[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let active = true

    setLoading(true)
    setError(null)
    fetchVideoStreams()
      .then((nextStreams) => {
        if (!active) return
        setStreams(nextStreams)
        setLoading(false)
      })
      .catch((err: unknown) => {
        if (!active) return
        setLoading(false)
        setError(err instanceof Error ? err.message : 'Failed to load video streams')
      })

    return () => {
      active = false
    }
  }, [])

  const filteredStreams =
    panel == null ? streams : streams.filter((stream) => stream.display.panel === panel)

  return {
    streams: filteredStreams,
    loading,
    error,
  }
}
