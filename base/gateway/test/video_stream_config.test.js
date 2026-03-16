'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')
const path = require('path')

const {
  SUPPORTED_ROS_ENCODINGS,
  SUPPORTED_SOURCE_TYPES,
  listBrowserStreams,
  loadVideoConfig,
  normalizeVideoConfig,
} = require('../src/video/stream_config')

test('normalizeVideoConfig accepts legacy ROS streams and preserves metadata', () => {
  const config = normalizeVideoConfig({
    version: 1,
    streams: [
      {
        stream_id: 'front_nav_cam',
        ros_topic: '/front_camera/image_raw',
        udp_port: 5000,
        ros_encoding: 'rgb8',
        width: 640,
        height: 480,
        framerate: 15,
        encoder: {
          bitrate_kbps: 1800,
          keyframe_interval: 30,
          speed_preset: 'ultrafast',
          tune: 'zerolatency',
        },
        display: {
          label: 'Front Nav',
          panel: 'delivery',
          order: 1,
        },
      },
    ],
  })

  assert.deepEqual(Array.from(SUPPORTED_ROS_ENCODINGS), ['rgb8', 'bgr8'])
  assert.deepEqual(Array.from(SUPPORTED_SOURCE_TYPES), ['ros_topic', 'v4l2'])
  assert.equal(config.streams.length, 1)
  assert.equal(config.streams[0].source_type, 'ros_topic')
  assert.equal(config.streams[0].encoder.keyframe_interval, 30)
  assert.equal(config.streams[0].display.panel, 'delivery')
})

test('normalizeVideoConfig accepts tagged V4L2 sources', () => {
  const config = normalizeVideoConfig({
    streams: [
      {
        stream_id: 'front_usb_cam',
        source: {
          type: 'v4l2',
          device: '/dev/video0',
          pixel_format: 'YUY2',
        },
        udp_port: 5000,
        width: 1280,
        height: 720,
        framerate: 30,
      },
    ],
  })

  assert.equal(config.streams[0].source_type, 'v4l2')
  assert.equal(config.streams[0].v4l2_device, '/dev/video0')
  assert.equal(config.streams[0].v4l2_pixel_format, 'YUY2')
  assert.equal(config.streams[0].ros_topic, null)
})

test('normalizeVideoConfig rejects duplicate identifiers and unsupported encodings', () => {
  assert.throws(
    () =>
      normalizeVideoConfig({
        streams: [
          {
            stream_id: 'front_nav_cam',
            ros_topic: '/front_camera/image_raw',
            udp_port: 5000,
            ros_encoding: 'rgb8',
          },
          {
            stream_id: 'front_nav_cam',
            ros_topic: '/rgbd_camera/color/image_raw',
            udp_port: 5002,
            ros_encoding: 'rgb8',
          },
        ],
      }),
    /duplicate stream_id/
  )

  assert.throws(
    () =>
      normalizeVideoConfig({
        streams: [
          {
            stream_id: 'depth_cam',
            ros_topic: '/depth/image_raw',
            udp_port: 5004,
            ros_encoding: 'mono16',
          },
        ],
      }),
    /ros_encoding/
  )

  assert.throws(
    () =>
      normalizeVideoConfig({
        streams: [
          {
            stream_id: 'cam_a',
            source: {
              type: 'v4l2',
              device: '/dev/video0',
            },
            udp_port: 5000,
          },
          {
            stream_id: 'cam_b',
            source: {
              type: 'v4l2',
              device: '/dev/video0',
            },
            udp_port: 5002,
          },
        ],
      }),
    /duplicate v4l2 device/
  )
})

test('listBrowserStreams sorts by display order and exposes source metadata', () => {
  const browserStreams = listBrowserStreams(
    normalizeVideoConfig({
      streams: [
        {
          stream_id: 'second',
          source: {
            type: 'v4l2',
            device: '/dev/video2',
          },
          udp_port: 5002,
          display: { label: 'Second', panel: 'delivery', order: 2 },
          encoder: { bitrate_kbps: 1500 },
        },
        {
          stream_id: 'first',
          ros_topic: '/first',
          udp_port: 5000,
          ros_encoding: 'rgb8',
          display: { label: 'First', panel: 'delivery', order: 1 },
          encoder: { bitrate_kbps: 1400 },
        },
      ],
    })
  )

  assert.deepEqual(
    browserStreams.map((stream) => stream.stream_id),
    ['first', 'second']
  )
  assert.equal(browserStreams[0].source_type, 'ros_topic')
  assert.equal(browserStreams[1].source_type, 'v4l2')
  assert.equal(browserStreams[1].v4l2_device, '/dev/video2')
  assert.equal('encoder' in browserStreams[0], false)
})

test('loadVideoConfig accepts the checked-in central video config', () => {
  const configPath = path.resolve(
    __dirname,
    '../../../rover/ros2_ws/src/mr2_launch/config/video_streams.json'
  )
  const config = loadVideoConfig(configPath)

  assert.ok(Array.isArray(config.streams))
  assert.ok(config.streams.length > 0)
  assert.ok(config.streams.every((stream) => typeof stream.stream_id === 'string'))
})
