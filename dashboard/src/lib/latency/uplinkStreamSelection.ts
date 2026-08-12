import type { VideoStreamInfo } from '../videoGateway'

export function selectOnlineUplinkStreams(
  streams: VideoStreamInfo[],
  excludedStreamIds: ReadonlySet<string>
) {
  return streams.filter(
    (stream) => stream.available && !excludedStreamIds.has(stream.stream_id)
  )
}
