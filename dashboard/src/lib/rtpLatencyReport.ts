export type RtpLatencyDistribution = {
  count?: number
  min: number | null
  mean: number | null
  p50: number | null
  p95: number | null
  p99: number | null
  max: number | null
}

export type RtpLatencyStreamReport = {
  stream_id: string
  udp_port: number
  ssrcs: number[]
  rover_packets: number
  base_packets: number
  matched_packets: number
  missing_at_base_packets: number
  unmatched_at_base_packets: number
  negative_latency_packets: number
  loss_percent: number | null
  rover_offered_bitrate_bps: number | null
  base_delivered_bitrate_bps: number | null
  link_latency_us: RtpLatencyDistribution
  packet_delay_variation_us: RtpLatencyDistribution
}

export type RtpLatencyReport = {
  schema_version: 1
  kind: 'mr2_rtp_link_latency_report'
  generated_at: string
  clock_offset_us: number
  clock_offset_definition: 'base_clock_minus_rover_clock'
  clock_offset_assumed: boolean
  aggregate_link_latency_us: RtpLatencyDistribution
  warnings: string[]
  streams: RtpLatencyStreamReport[]
}

export type AutomatedUplinkStreamReport = {
  stream_id: string
  udp_port: number
  segments: AutomatedUplinkSegments
  matched_frames: number
  browser_frames: number
  matched_packets: number
  packet_loss_percent: number | null
  rover_offered_bitrate_bps: number | null
  base_delivered_bitrate_bps: number | null
  warnings: string[]
}

export type AutomatedLatencyDistribution = RtpLatencyDistribution & { count: number }

export type AutomatedUplinkSegments = {
  rocket_m2: AutomatedLatencyDistribution
  base_to_browser: AutomatedLatencyDistribution
  decode_render: AutomatedLatencyDistribution
  total: AutomatedLatencyDistribution
}

export type AutomatedUplinkLatencyReport = {
  schema_version: 2
  kind: 'mr2_automated_uplink_latency_report'
  generated_at: string
  trial_id: string
  feed_count: number
  stream_ids: string[]
  clock: {
    base_minus_rover_us: number
    base_minus_browser_us: number
  }
  aggregate: AutomatedUplinkSegments
  matched_frames: number
  browser_frames: number
  warnings: string[]
  streams: AutomatedUplinkStreamReport[]
}

const isObject = (value: unknown): value is Record<string, unknown> =>
  typeof value === 'object' && value !== null && !Array.isArray(value)

function finiteNumber(value: unknown, field: string) {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    throw new Error(`${field} must be a finite number`)
  }
  return value
}

function nullableNumber(value: unknown, field: string, nonnegative = false) {
  if (value === null) return null
  const parsed = finiteNumber(value, field)
  if (nonnegative && parsed < 0) throw new Error(`${field} must not be negative`)
  return parsed
}

function packetCount(value: unknown, field: string) {
  const parsed = finiteNumber(value, field)
  if (!Number.isInteger(parsed) || parsed < 0) {
    throw new Error(`${field} must be a nonnegative integer`)
  }
  return parsed
}

function distribution(value: unknown, field: string): RtpLatencyDistribution {
  if (!isObject(value)) throw new Error(`${field} must be an object`)
  return {
    count: value.count == null ? undefined : packetCount(value.count, `${field}.count`),
    min: nullableNumber(value.min, `${field}.min`),
    mean: nullableNumber(value.mean, `${field}.mean`),
    p50: nullableNumber(value.p50, `${field}.p50`),
    p95: nullableNumber(value.p95, `${field}.p95`),
    p99: nullableNumber(value.p99, `${field}.p99`),
    max: nullableNumber(value.max, `${field}.max`),
  }
}

function streamReport(value: unknown, index: number): RtpLatencyStreamReport {
  const field = `streams[${index}]`
  if (!isObject(value)) throw new Error(`${field} must be an object`)
  if (typeof value.stream_id !== 'string' || value.stream_id.length === 0) {
    throw new Error(`${field}.stream_id must be a non-empty string`)
  }
  const udpPort = packetCount(value.udp_port, `${field}.udp_port`)
  if (udpPort < 1 || udpPort > 65535) throw new Error(`${field}.udp_port is invalid`)
  if (!Array.isArray(value.ssrcs)) throw new Error(`${field}.ssrcs must be an array`)
  const lossPercent = nullableNumber(value.loss_percent, `${field}.loss_percent`, true)
  if (lossPercent != null && lossPercent > 100) {
    throw new Error(`${field}.loss_percent must be at most 100`)
  }
  return {
    stream_id: value.stream_id,
    udp_port: udpPort,
    ssrcs: value.ssrcs.map((ssrc, ssrcIndex) =>
      packetCount(ssrc, `${field}.ssrcs[${ssrcIndex}]`)
    ),
    rover_packets: packetCount(value.rover_packets, `${field}.rover_packets`),
    base_packets: packetCount(value.base_packets, `${field}.base_packets`),
    matched_packets: packetCount(value.matched_packets, `${field}.matched_packets`),
    missing_at_base_packets: packetCount(
      value.missing_at_base_packets,
      `${field}.missing_at_base_packets`
    ),
    unmatched_at_base_packets: packetCount(
      value.unmatched_at_base_packets,
      `${field}.unmatched_at_base_packets`
    ),
    negative_latency_packets: packetCount(
      value.negative_latency_packets,
      `${field}.negative_latency_packets`
    ),
    loss_percent: lossPercent,
    rover_offered_bitrate_bps: nullableNumber(
      value.rover_offered_bitrate_bps,
      `${field}.rover_offered_bitrate_bps`,
      true
    ),
    base_delivered_bitrate_bps: nullableNumber(
      value.base_delivered_bitrate_bps,
      `${field}.base_delivered_bitrate_bps`,
      true
    ),
    link_latency_us: distribution(value.link_latency_us, `${field}.link_latency_us`),
    packet_delay_variation_us: distribution(
      value.packet_delay_variation_us,
      `${field}.packet_delay_variation_us`
    ),
  }
}

export function parseRtpLatencyReport(input: string | unknown): RtpLatencyReport {
  const value = typeof input === 'string' ? (JSON.parse(input) as unknown) : input
  if (!isObject(value)) throw new Error('RTP latency report must be an object')
  if (value.schema_version !== 1 || value.kind !== 'mr2_rtp_link_latency_report') {
    throw new Error('Unsupported RTP latency report schema')
  }
  if (typeof value.generated_at !== 'string' || !value.generated_at) {
    throw new Error('generated_at must be a string')
  }
  if (value.clock_offset_definition !== 'base_clock_minus_rover_clock') {
    throw new Error('clock_offset_definition must be base_clock_minus_rover_clock')
  }
  if (typeof value.clock_offset_assumed !== 'boolean') {
    throw new Error('clock_offset_assumed must be a boolean')
  }
  if (!Array.isArray(value.warnings) || !value.warnings.every((item) => typeof item === 'string')) {
    throw new Error('warnings must be a string array')
  }
  if (!Array.isArray(value.streams) || value.streams.length === 0) {
    throw new Error('streams must be a non-empty array')
  }
  return {
    schema_version: 1,
    kind: 'mr2_rtp_link_latency_report',
    generated_at: value.generated_at,
    clock_offset_us: finiteNumber(value.clock_offset_us, 'clock_offset_us'),
    clock_offset_definition: 'base_clock_minus_rover_clock',
    clock_offset_assumed: value.clock_offset_assumed,
    aggregate_link_latency_us: isObject(value.aggregate_link_latency_us)
      ? distribution(value.aggregate_link_latency_us, 'aggregate_link_latency_us')
      : {
          min: null,
          mean: null,
          p50: null,
          p95: null,
          p99: null,
          max: null,
        },
    warnings: [...value.warnings],
    streams: value.streams.map(streamReport),
  }
}

function stringArray(value: unknown, field: string) {
  if (!Array.isArray(value) || !value.every((item) => typeof item === 'string')) {
    throw new Error(`${field} must be a string array`)
  }
  return [...value]
}

function automatedSegments(value: unknown, field: string): AutomatedUplinkSegments {
  if (!isObject(value)) throw new Error(`${field} must be an object`)
  const requiredDistribution = (input: unknown, name: string) => {
    const parsed = distribution(input, name)
    if (parsed.count == null) throw new Error(`${name}.count is required`)
    return parsed as AutomatedLatencyDistribution
  }
  return {
    rocket_m2: requiredDistribution(value.rocket_m2, `${field}.rocket_m2`),
    base_to_browser: requiredDistribution(value.base_to_browser, `${field}.base_to_browser`),
    decode_render: requiredDistribution(value.decode_render, `${field}.decode_render`),
    total: requiredDistribution(value.total, `${field}.total`),
  }
}

export function parseAutomatedUplinkReport(
  input: string | unknown
): AutomatedUplinkLatencyReport {
  const value = typeof input === 'string' ? (JSON.parse(input) as unknown) : input
  if (!isObject(value) || value.schema_version !== 2 ||
      value.kind !== 'mr2_automated_uplink_latency_report') {
    throw new Error('Unsupported automated Uplink report schema')
  }
  if (typeof value.generated_at !== 'string' || typeof value.trial_id !== 'string') {
    throw new Error('Automated Uplink report identity is invalid')
  }
  const feedCount = packetCount(value.feed_count, 'feed_count')
  const streamIds = stringArray(value.stream_ids, 'stream_ids')
  if (feedCount < 1 || feedCount !== streamIds.length) {
    throw new Error('feed_count must match stream_ids')
  }
  if (!isObject(value.clock)) throw new Error('clock must be an object')
  if (!Array.isArray(value.streams) || value.streams.length !== feedCount) {
    throw new Error('streams must match feed_count')
  }
  const streams = value.streams.map((item, index) => {
    const field = `streams[${index}]`
    if (!isObject(item) || typeof item.stream_id !== 'string') {
      throw new Error(`${field} is invalid`)
    }
    return {
      stream_id: item.stream_id,
      udp_port: packetCount(item.udp_port, `${field}.udp_port`),
      segments: automatedSegments(item.segments, `${field}.segments`),
      matched_frames: packetCount(item.matched_frames, `${field}.matched_frames`),
      browser_frames: packetCount(item.browser_frames, `${field}.browser_frames`),
      matched_packets: packetCount(item.matched_packets, `${field}.matched_packets`),
      packet_loss_percent: nullableNumber(
        item.packet_loss_percent,
        `${field}.packet_loss_percent`,
        true
      ),
      rover_offered_bitrate_bps: nullableNumber(
        item.rover_offered_bitrate_bps,
        `${field}.rover_offered_bitrate_bps`,
        true
      ),
      base_delivered_bitrate_bps: nullableNumber(
        item.base_delivered_bitrate_bps,
        `${field}.base_delivered_bitrate_bps`,
        true
      ),
      warnings: stringArray(item.warnings, `${field}.warnings`),
    }
  })
  if (streams.some((stream, index) => stream.stream_id !== streamIds[index])) {
    throw new Error('stream report order must match stream_ids')
  }
  return {
    schema_version: 2,
    kind: 'mr2_automated_uplink_latency_report',
    generated_at: value.generated_at,
    trial_id: value.trial_id,
    feed_count: feedCount,
    stream_ids: streamIds,
    clock: {
      base_minus_rover_us: finiteNumber(
        value.clock.base_minus_rover_us,
        'clock.base_minus_rover_us'
      ),
      base_minus_browser_us: finiteNumber(
        value.clock.base_minus_browser_us,
        'clock.base_minus_browser_us'
      ),
    },
    aggregate: automatedSegments(value.aggregate, 'aggregate'),
    matched_frames: packetCount(value.matched_frames, 'matched_frames'),
    browser_frames: packetCount(value.browser_frames, 'browser_frames'),
    warnings: stringArray(value.warnings, 'warnings'),
    streams,
  }
}
