'use strict'

const test = require('node:test')
const assert = require('node:assert/strict')

const {
  SUPPORTED_ROS_ENCODINGS,
  listBrowserStreams,
  normalizeVideoConfig,
} = require('../src/video/stream_config')

test('normalizeVideoConfig accepts the supported encodings and preserves metadata', () => {
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
      {
        stream_id: 'aruco_debug',
        ros_topic: '/aruco_tracker/debug',
        udp_port: 5002,
        ros_encoding: 'bgr8',
      },
    ],
  })

  assert.deepEqual(Array.from(SUPPORTED_ROS_ENCODINGS), ['rgb8', 'bgr8'])
  assert.equal(config.streams.length, 2)
  assert.equal(config.streams[0].encoder.keyframe_interval, 30)
  assert.equal(config.streams[0].display.panel, 'delivery')
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
})

test('listBrowserStreams sorts by display order and strips encoder internals', () => {
  const browserStreams = listBrowserStreams(
    normalizeVideoConfig({
      streams: [
        {
          stream_id: 'second',
          ros_topic: '/second',
          udp_port: 5002,
          ros_encoding: 'rgb8',
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
  assert.equal('encoder' in browserStreams[0], false)
})
