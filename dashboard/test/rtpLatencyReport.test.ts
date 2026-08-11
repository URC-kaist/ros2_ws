import assert from 'node:assert/strict'

import {
  parseAutomatedUplinkReport,
  parseRtpLatencyReport,
} from '../src/lib/rtpLatencyReport'

const valid = {
  schema_version: 1,
  kind: 'mr2_rtp_link_latency_report',
  generated_at: '2026-08-10T00:00:00+00:00',
  clock_offset_us: 250,
  clock_offset_definition: 'base_clock_minus_rover_clock',
  clock_offset_assumed: false,
  warnings: [],
  streams: [
    {
      stream_id: 'front_test_cam',
      udp_port: 5000,
      ssrcs: [1234],
      rover_packets: 100,
      base_packets: 99,
      matched_packets: 99,
      missing_at_base_packets: 1,
      unmatched_at_base_packets: 0,
      negative_latency_packets: 0,
      loss_percent: 1,
      rover_offered_bitrate_bps: 2_000_000,
      base_delivered_bitrate_bps: 1_980_000,
      link_latency_us: { min: 1000, mean: 2000, p50: 1800, p95: 4000, p99: 5000, max: 6000 },
      packet_delay_variation_us: { min: 0, mean: 100, p50: 80, p95: 250, p99: 400, max: 500 },
    },
  ],
}

const parsed = parseRtpLatencyReport(JSON.stringify(valid))
assert.equal(parsed.streams[0].link_latency_us.p95, 4000)
assert.equal(parsed.streams[0].loss_percent, 1)

assert.throws(
  () => parseRtpLatencyReport({ ...valid, kind: 'wrong' }),
  /Unsupported RTP latency report schema/
)

const segment = {
  count: 2,
  min: 100,
  mean: 150,
  p50: 150,
  p95: 195,
  p99: 199,
  max: 200,
}
const automated = parseAutomatedUplinkReport({
  schema_version: 2,
  kind: 'mr2_automated_uplink_latency_report',
  generated_at: '2026-08-11T00:00:00+00:00',
  trial_id: 'trial-1',
  feed_count: 1,
  stream_ids: ['front_test_cam'],
  clock: { base_minus_rover_us: 0, base_minus_browser_us: 25 },
  aggregate: {
    rocket_m2: segment,
    base_to_browser: segment,
    decode_render: segment,
    total: segment,
  },
  matched_frames: 2,
  browser_frames: 2,
  warnings: [],
  streams: [
    {
      stream_id: 'front_test_cam',
      udp_port: 5000,
      segments: {
        rocket_m2: segment,
        base_to_browser: segment,
        decode_render: segment,
        total: segment,
      },
      matched_frames: 2,
      browser_frames: 2,
      matched_packets: 10,
      packet_loss_percent: 0,
      rover_offered_bitrate_bps: 1000,
      base_delivered_bitrate_bps: 1000,
      warnings: [],
    },
  ],
})
assert.equal(automated.aggregate.total.p95, 195)
assert.throws(
  () => parseAutomatedUplinkReport({ ...automated, aggregate: { total: segment } }),
  /aggregate.rocket_m2/
)
assert.throws(
  () =>
    parseRtpLatencyReport({
      ...valid,
      streams: [{ ...valid.streams[0], rover_packets: -1 }],
    }),
  /nonnegative integer/
)
assert.throws(
  () =>
    parseRtpLatencyReport({
      ...valid,
      streams: [{ ...valid.streams[0], loss_percent: 101 }],
    }),
  /at most 100/
)

console.log('RTP latency report parser test passed')
