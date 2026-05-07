import { useCallback, useEffect, useState } from 'react'
import { fetchVideoStreams, type VideoStreamInfo } from '../lib/videoGateway'

export function useVideoStreams(panel?: string) {
  const [streams, setStreams] = useState<VideoStreamInfo[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [refreshIndex, setRefreshIndex] = useState(0)

  const refresh = useCallback(() => {
    setRefreshIndex((current) => current + 1)
  }, [])

  useEffect(() => {
    let active = true
    let intervalId: number | null = null

    const loadStreams = async (showLoading: boolean) => {
      if (showLoading) {
        setLoading(true)
      }

      try {
        const nextStreams = await fetchVideoStreams()
        if (!active) return
        setStreams(nextStreams)
        setError(null)
      } catch (err: unknown) {
        if (!active) return
        setError(err instanceof Error ? err.message : 'Failed to load video streams')
      } finally {
        if (active && showLoading) {
          setLoading(false)
        }
      }
    }

    void loadStreams(true)
    intervalId = window.setInterval(() => {
      void loadStreams(false)
    }, 2000)

    return () => {
      active = false
      if (intervalId != null) {
        window.clearInterval(intervalId)
      }
    }
  }, [refreshIndex])

  const filteredStreams =
    panel == null ? streams : streams.filter((stream) => stream.display.panel === panel)

  return {
    streams: filteredStreams,
    loading,
    error,
    refresh,
  }
}
