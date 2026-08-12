import assert from 'node:assert/strict'

import { selectOnlineUplinkStreams } from '../src/lib/latency/uplinkStreamSelection'
import type { VideoStreamInfo } from '../src/lib/videoGateway'

function stream(streamId: string, available: boolean): VideoStreamInfo {
  return {
    stream_id: streamId,
    source_type: 'v4l2',
    ros_topic: null,
    udp_port: 5000,
    ros_encoding: null,
    v4l2_device: `/dev/video-${streamId}`,
    v4l2_pixel_format: 'YUYV',
    width: 640,
    height: 480,
    framerate: 30,
    available,
    display: { label: streamId },
  }
}

const noExclusions = new Set<string>()
assert.deepEqual(
  selectOnlineUplinkStreams(
    [stream('front', true), stream('rear', false)],
    noExclusions
  ).map((candidate) => candidate.stream_id),
  ['front']
)

// Streams become available one at a time during rover lease preparation. Newly
// available streams must remain selected unless the operator explicitly excluded them.
const allOnline = [
  stream('front', true),
  stream('rear', true),
  stream('left', true),
  stream('right', true),
]
assert.deepEqual(
  selectOnlineUplinkStreams(allOnline, noExclusions).map(
    (candidate) => candidate.stream_id
  ),
  ['front', 'rear', 'left', 'right']
)
assert.deepEqual(
  selectOnlineUplinkStreams(allOnline, new Set(['rear'])).map(
    (candidate) => candidate.stream_id
  ),
  ['front', 'left', 'right']
)

console.log('uplink staggered stream selection tests passed')
