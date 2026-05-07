import { useEffect, useState } from 'react'
import { useRosBridge } from '../hooks/useRosBridge'

type RosImageMessage = {
  height: number
  width: number
  encoding: string
  is_bigendian?: number
  step: number
  data: string | number[]
}

type PoseStampedMessage = {
  pose?: {
    position?: {
      x?: number
      y?: number
      z?: number
    }
  }
}

type ObjectState = {
  label: string
  topic: string
  updatedAt: number | null
  position: {
    x: number
    y: number
    z: number
  } | null
}

const OBJECT_TOPICS = [
  { id: 'class_0', label: 'MALLET', topic: '/yolo/object_pose/class_0' },
  { id: 'class_1', label: 'PICK', topic: '/yolo/object_pose/class_1' },
  { id: 'class_2', label: 'BOTTLE', topic: '/yolo/object_pose/class_2' },
] as const

const ANNOTATED_IMAGE_TOPIC = '/yolo/annotated_image'
const IMAGE_MESSAGE_TYPES = ['sensor_msgs/msg/Image', 'sensor_msgs/Image'] as const

const bytesFromRosData = (data: string | number[]) => {
  if (typeof data !== 'string') {
    return Uint8Array.from(data)
  }
  const binary = window.atob(data)
  const bytes = new Uint8Array(binary.length)
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index)
  }
  return bytes
}

const imageMessageToDataUrl = (message: RosImageMessage) => {
  const width = Math.max(0, Math.floor(message.width))
  const height = Math.max(0, Math.floor(message.height))
  if (width === 0 || height === 0) {
    throw new Error('Empty image')
  }

  const encoding = message.encoding.toLowerCase()
  const source = bytesFromRosData(message.data)
  const rgba = new Uint8ClampedArray(width * height * 4)
  const channels =
    encoding === 'mono8' || encoding === '8uc1'
      ? 1
      : encoding === 'rgba8' || encoding === 'bgra8'
        ? 4
        : 3
  const step = message.step || width * channels

  if (
    !['rgb8', 'bgr8', 'rgba8', 'bgra8', 'mono8', '8uc1'].includes(encoding)
  ) {
    throw new Error(`Unsupported encoding ${message.encoding}`)
  }

  for (let y = 0; y < height; y += 1) {
    const rowOffset = y * step
    for (let x = 0; x < width; x += 1) {
      const sourceOffset = rowOffset + x * channels
      const targetOffset = (y * width + x) * 4

      if (encoding === 'mono8' || encoding === '8uc1') {
        const value = source[sourceOffset] ?? 0
        rgba[targetOffset] = value
        rgba[targetOffset + 1] = value
        rgba[targetOffset + 2] = value
        rgba[targetOffset + 3] = 255
      } else if (encoding === 'bgr8' || encoding === 'bgra8') {
        rgba[targetOffset] = source[sourceOffset + 2] ?? 0
        rgba[targetOffset + 1] = source[sourceOffset + 1] ?? 0
        rgba[targetOffset + 2] = source[sourceOffset] ?? 0
        rgba[targetOffset + 3] =
          encoding === 'bgra8' ? (source[sourceOffset + 3] ?? 255) : 255
      } else {
        rgba[targetOffset] = source[sourceOffset] ?? 0
        rgba[targetOffset + 1] = source[sourceOffset + 1] ?? 0
        rgba[targetOffset + 2] = source[sourceOffset + 2] ?? 0
        rgba[targetOffset + 3] =
          encoding === 'rgba8' ? (source[sourceOffset + 3] ?? 255) : 255
      }
    }
  }

  const canvas = document.createElement('canvas')
  canvas.width = width
  canvas.height = height
  const context = canvas.getContext('2d')
  if (!context) {
    throw new Error('Canvas unavailable')
  }
  context.putImageData(new ImageData(rgba, width, height), 0, 0)
  return canvas.toDataURL('image/png')
}

const RecentObjectsCard = () => {
  const { ros, connected } = useRosBridge()
  const [objects, setObjects] = useState<Record<string, ObjectState>>(() =>
    Object.fromEntries(
      OBJECT_TOPICS.map((object) => [
        object.id,
        {
          label: object.label,
          topic: object.topic,
          updatedAt: null,
          position: null,
        },
      ])
    )
  )
  const [imageUrl, setImageUrl] = useState<string | null>(null)
  const [imageUpdatedAt, setImageUpdatedAt] = useState<number | null>(null)
  const [imageError, setImageError] = useState<string | null>(null)
  const [imageSourceType, setImageSourceType] = useState<string | null>(null)

  useEffect(() => {
    const poseUnsubscribers = OBJECT_TOPICS.map((object) =>
      ros.subscribe<PoseStampedMessage>(
        object.topic,
        'geometry_msgs/msg/PoseStamped',
        (message) => {
          const position = message.pose?.position
          setObjects((current) => ({
            ...current,
            [object.id]: {
              label: object.label,
              topic: object.topic,
              updatedAt: Date.now(),
              position:
                position &&
                Number.isFinite(position.x) &&
                Number.isFinite(position.y) &&
                Number.isFinite(position.z)
                  ? {
                      x: position.x ?? 0,
                      y: position.y ?? 0,
                      z: position.z ?? 0,
                    }
                  : null,
            },
          }))
        },
        { throttleRate: 500, queueSize: 1 }
      )
    )

    const imageUnsubscribers = IMAGE_MESSAGE_TYPES.map((messageType) =>
      ros.subscribe<RosImageMessage>(
        ANNOTATED_IMAGE_TOPIC,
        messageType,
        (message) => {
          try {
            setImageUrl(imageMessageToDataUrl(message))
            setImageUpdatedAt(Date.now())
            setImageSourceType(messageType)
            setImageError(null)
          } catch (error) {
            setImageError(error instanceof Error ? error.message : 'Image decode failed')
          }
        },
        { throttleRate: 500, queueSize: 1, compression: 'cbor' }
      )
    )

    return () => {
      for (const unsubscribe of poseUnsubscribers) {
        unsubscribe()
      }
      for (const unsubscribe of imageUnsubscribers) {
        unsubscribe()
      }
    }
  }, [ros])

  return (
    <article className="card recent-objects-card">
      <header className="recent-objects-header">
        <h3>Recent Discovered Objects</h3>
        <span className={`recent-objects-status ${connected ? 'connected' : ''}`}>
          {connected ? 'ROS connected' : 'ROS offline'}
        </span>
      </header>
      <div className="recent-objects-image">
        {imageUrl ? (
          <img src={imageUrl} alt="YOLO annotated detections" />
        ) : (
          <span>{imageError ?? `Awaiting ${ANNOTATED_IMAGE_TOPIC}...`}</span>
        )}
      </div>
      <span className="recent-objects-image-meta">
        {ANNOTATED_IMAGE_TOPIC} ·{' '}
        {imageUpdatedAt ? new Date(imageUpdatedAt).toLocaleTimeString() : '--'}
        {imageSourceType ? ` · ${imageSourceType}` : ''}
      </span>
      <div className="recent-objects-grid">
        {OBJECT_TOPICS.map((object) => {
          const state = objects[object.id]
          return (
            <div className="recent-object" key={object.id}>
              <div className="recent-object__meta">
                <span className="recent-object__label">{state.label}</span>
                <span className="recent-object__topic">{state.topic}</span>
              </div>
              <div className="recent-object__pose">
                {state.position ? (
                  <>
                    <span>x {state.position.x.toFixed(2)}</span>
                    <span>y {state.position.y.toFixed(2)}</span>
                    <span>z {state.position.z.toFixed(2)}</span>
                  </>
                ) : (
                  <span>Awaiting pose...</span>
                )}
              </div>
              <span className="recent-object__age">
                Pose {state.updatedAt ? new Date(state.updatedAt).toLocaleTimeString() : '--'}
                {' · '}
                Image {imageUpdatedAt ? new Date(imageUpdatedAt).toLocaleTimeString() : '--'}
              </span>
            </div>
          )
        })}
      </div>
    </article>
  )
}

export default RecentObjectsCard
