import { useEffect, useMemo, useRef, useState, type CSSProperties } from 'react'
import { TransitiveCapability } from '@transitive-sdk/utils-web'
import './TransitiveVideoCard.css'

type TransitiveVideoCardProps = {
  title?: string
  description?: string
  source?: string
  jwt?: string
  embedded?: boolean
  videoWidth?: number
  videoHeight?: number
  type?: string
  streamtype?: string
  framerate?: string
  width?: number | string
  height?: number | string
  count?: string
  quantizer?: string
  timeout?: string
  rosversion?: string
}

const TransitiveVideoCard = ({
  title = 'Video',
  description = 'Live feed via Transitive WebRTC.',
  source = '/rgbd_camera/color/image_raw',
  jwt,
  embedded = false,
  videoWidth,
  videoHeight,
  type = 'rostopic',
  streamtype,
  framerate,
  width,
  height,
  count = '1',
  quantizer = '25',
  timeout = '1800',
  rosversion = '2',
}: TransitiveVideoCardProps) => {
  const [fetchedToken, setFetchedToken] = useState<string | null>(null)
  const [tokenStatus, setTokenStatus] = useState<'idle' | 'loading' | 'ready' | 'error'>(
    'idle'
  )
  const frameRef = useRef<HTMLDivElement | null>(null)
  const rawToken = jwt ?? fetchedToken ?? ''
  const token = rawToken.trim().replace(/^['"]|['"]$/g, '')
  const capabilityProps = useMemo(() => {
    if (type === 'v4l2src') {
      return {
        count,
        quantizer,
        source,
        timeout,
        type,
        streamtype,
        framerate,
        width: width ? String(width) : undefined,
        height: height ? String(height) : undefined,
      }
    }

    return {
      count,
      quantizer,
      source,
      timeout,
      type,
      rosversion,
    }
  }, [
    count,
    framerate,
    height,
    quantizer,
    rosversion,
    source,
    streamtype,
    timeout,
    type,
    width,
  ])

  const tokenEndpoint = useMemo(() => {
    const explicit = import.meta.env.VITE_TRANSITIVE_TOKEN_URL as string | undefined
    const base =
      explicit ??
      (typeof window === 'undefined'
        ? '/transitive/token'
        : `${window.location.origin}/transitive/token`)
    const params = new URLSearchParams()
    const id = import.meta.env.VITE_TRANSITIVE_ID as string | undefined
    const device = import.meta.env.VITE_TRANSITIVE_DEVICE as string | undefined
    const capability = import.meta.env.VITE_TRANSITIVE_CAPABILITY as string | undefined
    const userId = import.meta.env.VITE_TRANSITIVE_USER_ID as string | undefined
    const validity = import.meta.env.VITE_TRANSITIVE_VALIDITY as string | undefined
    if (id) params.set('id', id)
    if (device) params.set('device', device)
    if (capability) params.set('capability', capability)
    if (userId) params.set('userId', userId)
    if (validity) params.set('validity', validity)
    const query = params.toString()
    return query ? `${base}?${query}` : base
  }, [])

  useEffect(() => {
    if (jwt) {
      setTokenStatus(jwt.trim() ? 'ready' : 'error')
      return
    }
    if (fetchedToken) {
      setTokenStatus('ready')
      return
    }

    let active = true
    setTokenStatus('loading')
    fetch(tokenEndpoint)
      .then((res) => (res.ok ? res.json() : null))
      .then((data) => {
        if (!active) return
        if (data?.token && typeof data.token === 'string') {
          setFetchedToken(data.token)
          setTokenStatus('ready')
        } else {
          setTokenStatus('error')
        }
      })
      .catch(() => {
        if (!active) return
        setTokenStatus('error')
      })
    return () => {
      active = false
    }
  }, [jwt, fetchedToken, tokenEndpoint])

  useEffect(() => {
    if (!token) return
    const frame = frameRef.current
    if (!frame) return

    const target = frame.querySelector('*') as HTMLElement | null
    if (!target) return

    const getRoot = () => (target.shadowRoot ? target.shadowRoot : target)
    const root = getRoot()
    const shouldHide = (text: string) =>
      text.toLowerCase().includes('performs best on google chrome')

    const hideBanner = () => {
      const alerts = Array.from(root.querySelectorAll('[role="alert"], .alert'))
      for (const alert of alerts) {
        const text = alert.textContent?.trim() ?? ''
        if (!text) continue
        if (shouldHide(text)) {
          const closeBtn = alert.querySelector('.btn-close') as HTMLElement | null
          if (closeBtn) {
            closeBtn.click()
          } else {
            ;(alert as HTMLElement).style.display = 'none'
          }
        }
      }
    }

    hideBanner()

    const observer = new MutationObserver(() => hideBanner())
    observer.observe(root, { childList: true, subtree: true })
    return () => observer.disconnect()
  }, [token])

  const videoStyle = {
    ...(videoWidth ? { '--video-width': `${videoWidth}px` } : {}),
    ...(videoWidth && videoHeight
      ? { '--video-aspect': `${videoWidth} / ${videoHeight}` }
      : {}),
  } as CSSProperties

  const media = token ? (
    <div className="transitive-frame" ref={frameRef}>
      <TransitiveCapability jwt={token} {...capabilityProps} />
    </div>
  ) : (
    <div className="transitive-placeholder">
      {tokenStatus === 'loading' ? 'Waiting for token...' : 'Missing Transitive JWT'}
    </div>
  )

  if (embedded) {
    return (
      <div className="transitive-embedded" style={videoStyle}>
        {media}
      </div>
    )
  }

  return (
    <article
      className="card card--span-2"
      style={videoStyle}
    >
      <h3>{title}</h3>
      {description ? <p>{description}</p> : null}
      {media}
    </article>
  )
}

export default TransitiveVideoCard
