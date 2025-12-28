import { useEffect, useMemo, useRef, useState, type CSSProperties } from 'react'
import { TransitiveCapability } from '@transitive-sdk/utils-web'
import './TransitiveVideoCard.css'

type TransitiveVideoCardProps = {
  title?: string
  description?: string
  source?: string
  jwt?: string
  videoWidth?: number
  videoHeight?: number
}

const TransitiveVideoCard = ({
  title = 'Video',
  description = 'Live feed via Transitive WebRTC.',
  source = '/rgbd_camera/image',
  jwt,
  videoWidth = 320,
  videoHeight = 240,
}: TransitiveVideoCardProps) => {
  const [fetchedToken, setFetchedToken] = useState<string | null>(null)
  const frameRef = useRef<HTMLDivElement | null>(null)
  const rawToken = jwt ?? fetchedToken ?? ''
  const token = rawToken.trim().replace(/^['"]|['"]$/g, '')
  const missingToken = !token

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
    if (jwt) return
    if (fetchedToken) return
    fetch(tokenEndpoint)
      .then((res) => (res.ok ? res.json() : null))
      .then((data) => {
        if (data?.token && typeof data.token === 'string') {
          setFetchedToken(data.token)
        }
      })
      .catch(() => null)
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

  return (
    <article
      className="card card--span-2"
      style={
        {
          '--video-width': `${videoWidth}px`,
          '--video-aspect': `${videoWidth} / ${videoHeight}`,
        } as CSSProperties
      }
    >
      <h3>{title}</h3>
      <p>{description}</p>
      {token ? (
        <div className="transitive-frame" ref={frameRef}>
          <TransitiveCapability
            jwt={token}
            count="1"
            quantizer="25"
            rosversion="2"
            source={source}
            timeout="1800"
            type="rostopic"
          />
        </div>
      ) : (
        <div className="transitive-placeholder">
          {missingToken ? 'Missing Transitive JWT' : 'Waiting for token...'}
        </div>
      )}
    </article>
  )
}

export default TransitiveVideoCard
