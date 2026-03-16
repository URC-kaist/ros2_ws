'use strict'

const fs = require('fs')

const SUPPORTED_ROS_ENCODINGS = new Set(['rgb8', 'bgr8'])
const SUPPORTED_SOURCE_TYPES = new Set(['ros_topic', 'v4l2'])

function assertNonEmptyString(value, label) {
  if (typeof value !== 'string' || value.trim() === '') {
    throw new Error(`${label} must be a non-empty string`)
  }
  return value.trim()
}

function parseOptionalPositiveInt(value, label) {
  if (value == null) return null
  const parsed = Number.parseInt(String(value), 10)
  if (!Number.isFinite(parsed) || parsed <= 0) {
    throw new Error(`${label} must be a positive integer when provided`)
  }
  return parsed
}

function parseOptionalString(value, label) {
  if (value == null) return null
  return assertNonEmptyString(value, label)
}

function normalizeEncoder(raw = {}, index = 0) {
  if (raw == null) return {}
  if (typeof raw !== 'object' || Array.isArray(raw)) {
    throw new Error(`streams[${index}].encoder must be an object when provided`)
  }

  const bitrateKbps = parseOptionalPositiveInt(
    raw.bitrate_kbps,
    `streams[${index}].encoder.bitrate_kbps`
  )
  const keyframeInterval = parseOptionalPositiveInt(
    raw.keyframe_interval,
    `streams[${index}].encoder.keyframe_interval`
  )

  const encoder = {}
  if (bitrateKbps != null) encoder.bitrate_kbps = bitrateKbps
  if (keyframeInterval != null) encoder.keyframe_interval = keyframeInterval

  if (raw.speed_preset != null) {
    encoder.speed_preset = assertNonEmptyString(
      raw.speed_preset,
      `streams[${index}].encoder.speed_preset`
    )
  }

  if (raw.tune != null) {
    encoder.tune = assertNonEmptyString(raw.tune, `streams[${index}].encoder.tune`)
  }

  return encoder
}

function normalizeDisplay(raw = {}, index = 0) {
  if (raw == null) return {}
  if (typeof raw !== 'object' || Array.isArray(raw)) {
    throw new Error(`streams[${index}].display must be an object when provided`)
  }

  const display = {}

  if (raw.label != null) {
    display.label = assertNonEmptyString(raw.label, `streams[${index}].display.label`)
  }

  if (raw.panel != null) {
    display.panel = assertNonEmptyString(raw.panel, `streams[${index}].display.panel`)
  }

  const order = parseOptionalPositiveInt(raw.order, `streams[${index}].display.order`)
  if (order != null) {
    display.order = order
  }

  return display
}

function normalizeTaggedSource(rawSource, index) {
  if (typeof rawSource !== 'object' || rawSource == null || Array.isArray(rawSource)) {
    throw new Error(`streams[${index}].source must be an object`)
  }

  const sourceType = assertNonEmptyString(rawSource.type, `streams[${index}].source.type`)
  if (!SUPPORTED_SOURCE_TYPES.has(sourceType)) {
    throw new Error(
      `streams[${index}].source.type must be one of ${Array.from(SUPPORTED_SOURCE_TYPES).join(', ')}`
    )
  }

  if (sourceType === 'ros_topic') {
    const rosTopic = assertNonEmptyString(
      rawSource.ros_topic,
      `streams[${index}].source.ros_topic`
    )
    const rosEncoding = assertNonEmptyString(
      rawSource.ros_encoding,
      `streams[${index}].source.ros_encoding`
    )

    if (!SUPPORTED_ROS_ENCODINGS.has(rosEncoding)) {
      throw new Error(
        `streams[${index}].source.ros_encoding must be one of ${Array.from(
          SUPPORTED_ROS_ENCODINGS
        ).join(', ')}`
      )
    }

    return {
      source_type: sourceType,
      ros_topic: rosTopic,
      ros_encoding: rosEncoding,
      v4l2_device: null,
      v4l2_pixel_format: null,
    }
  }

  return {
    source_type: sourceType,
    ros_topic: null,
    ros_encoding: null,
    v4l2_device: assertNonEmptyString(rawSource.device, `streams[${index}].source.device`),
    v4l2_pixel_format: parseOptionalString(
      rawSource.pixel_format,
      `streams[${index}].source.pixel_format`
    ),
  }
}

function normalizeSource(raw, index) {
  if (raw.source == null) {
    throw new Error(`streams[${index}].source is required`)
  }
  return normalizeTaggedSource(raw.source, index)
}

function normalizeStream(raw, index) {
  if (typeof raw !== 'object' || raw == null || Array.isArray(raw)) {
    throw new Error(`streams[${index}] must be an object`)
  }

  const streamId = assertNonEmptyString(raw.stream_id, `streams[${index}].stream_id`)
  const udpPort = parseOptionalPositiveInt(raw.udp_port, `streams[${index}].udp_port`)

  if (udpPort == null || udpPort > 65535) {
    throw new Error(`streams[${index}].udp_port must be a valid UDP port`)
  }

  return {
    stream_id: streamId,
    udp_port: udpPort,
    width: parseOptionalPositiveInt(raw.width, `streams[${index}].width`),
    height: parseOptionalPositiveInt(raw.height, `streams[${index}].height`),
    framerate: parseOptionalPositiveInt(raw.framerate, `streams[${index}].framerate`),
    encoder: normalizeEncoder(raw.encoder, index),
    display: normalizeDisplay(raw.display, index),
    ...normalizeSource(raw, index),
  }
}

function normalizeVideoConfig(raw) {
  if (typeof raw !== 'object' || raw == null || Array.isArray(raw)) {
    throw new Error('video stream config must be a JSON object')
  }

  if (!Array.isArray(raw.streams) || raw.streams.length === 0) {
    throw new Error('video stream config must contain a non-empty streams array')
  }

  const streams = raw.streams.map(normalizeStream)
  const streamIds = new Set()
  const udpPorts = new Set()
  const rosTopics = new Set()
  const v4l2Devices = new Set()

  for (const stream of streams) {
    if (streamIds.has(stream.stream_id)) {
      throw new Error(`duplicate stream_id: ${stream.stream_id}`)
    }
    if (udpPorts.has(stream.udp_port)) {
      throw new Error(`duplicate udp_port: ${stream.udp_port}`)
    }
    if (stream.ros_topic && rosTopics.has(stream.ros_topic)) {
      throw new Error(`duplicate ros_topic: ${stream.ros_topic}`)
    }
    if (stream.v4l2_device && v4l2Devices.has(stream.v4l2_device)) {
      throw new Error(`duplicate v4l2 device: ${stream.v4l2_device}`)
    }
    streamIds.add(stream.stream_id)
    udpPorts.add(stream.udp_port)
    if (stream.ros_topic) rosTopics.add(stream.ros_topic)
    if (stream.v4l2_device) v4l2Devices.add(stream.v4l2_device)
  }

  return {
    version: raw.version == null ? 1 : Number(raw.version),
    streams,
  }
}

function loadVideoConfig(configPath) {
  const text = fs.readFileSync(configPath, 'utf8')
  const parsed = JSON.parse(text)
  return normalizeVideoConfig(parsed)
}

function listBrowserStreams(videoConfig) {
  return videoConfig.streams
    .map((stream) => ({
      stream_id: stream.stream_id,
      source_type: stream.source_type,
      ros_topic: stream.ros_topic,
      ros_encoding: stream.ros_encoding,
      v4l2_device: stream.v4l2_device,
      v4l2_pixel_format: stream.v4l2_pixel_format,
      udp_port: stream.udp_port,
      width: stream.width,
      height: stream.height,
      framerate: stream.framerate,
      display: stream.display,
    }))
    .sort((left, right) => {
      const leftOrder = left.display.order || Number.MAX_SAFE_INTEGER
      const rightOrder = right.display.order || Number.MAX_SAFE_INTEGER
      if (leftOrder !== rightOrder) {
        return leftOrder - rightOrder
      }
      return left.stream_id.localeCompare(right.stream_id)
    })
}

module.exports = {
  SUPPORTED_ROS_ENCODINGS,
  SUPPORTED_SOURCE_TYPES,
  listBrowserStreams,
  loadVideoConfig,
  normalizeVideoConfig,
}
