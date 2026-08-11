import type { AutomatedUplinkLatencyReport } from '../rtpLatencyReport'
import type { UplinkBrowserSample } from '../latencyDiagnostics'

export type UplinkGatewayPhase =
  | 'created'
  | 'base_capturing'
  | 'waiting_for_rover_artifacts'
  | 'stopping_base_capture'
  | 'analyzing'
  | 'completed'
  | 'failed'
  | 'cancelled'

export type UplinkGatewayTrial = {
  schema_version: 1
  trial_id: string
  phase: UplinkGatewayPhase
  progress: number
  feed_count: number
  stream_ids: string[]
  duration_s: number
  rover_metadata_received: boolean
  rover_capture_received: boolean
  browser_samples_received: boolean
  error: { code: string; message: string } | null
  report: AutomatedUplinkLatencyReport | null
  upload_base_url?: string
  upload_token?: string
}

function resolveGatewayHttpUrl(pathname: string) {
  const explicit = import.meta.env.VITE_XBEE_WS_URL as string | undefined
  if (explicit) {
    const url = new URL(explicit, window.location.href)
    url.protocol = url.protocol === 'wss:' ? 'https:' : 'http:'
    url.pathname = pathname
    url.search = ''
    url.hash = ''
    return url.toString()
  }
  return `${window.location.origin}${pathname}`
}

type JsonRequestInit = {
  method?: string
  body?: string
  headers?: Record<string, string>
}

async function requestJson<T>(pathname: string, init: JsonRequestInit = {}): Promise<T> {
  const response = await fetch(resolveGatewayHttpUrl(pathname), {
    cache: 'no-store',
    ...init,
    headers: {
      ...(init.body ? { 'Content-Type': 'application/json' } : {}),
      ...init.headers,
    },
  })
  const contentType = response.headers.get('content-type') || ''
  const body = contentType.includes('application/json')
    ? ((await response.json()) as Record<string, unknown>)
    : null
  if (!response.ok) {
    throw new Error(
      typeof body?.error === 'string'
        ? body.error
        : `Uplink endpoint returned ${response.status}`
    )
  }
  return body as T
}

export function createUplinkTrial(feedCount: number, streamIds: string[], durationS: number) {
  return requestJson<UplinkGatewayTrial>('/latency/uplink/trials', {
    method: 'POST',
    body: JSON.stringify({
      feed_count: feedCount,
      stream_ids: streamIds,
      duration_s: durationS,
    }),
  })
}

export function startUplinkTrial(
  trialId: string,
  browserClock: { offset_us: number; rtt_us: number; sampled_at_epoch_us: number },
  chronySnapshot: Record<string, unknown>
) {
  return requestJson<UplinkGatewayTrial>(`/latency/uplink/trials/${trialId}/start`, {
    method: 'POST',
    body: JSON.stringify({
      browser_clock: browserClock,
      chrony_snapshot: chronySnapshot,
    }),
  })
}

export function getUplinkTrial(trialId: string) {
  return requestJson<UplinkGatewayTrial>(`/latency/uplink/trials/${trialId}`)
}

export function uploadUplinkBrowserSamples(trialId: string, samples: UplinkBrowserSample[]) {
  return requestJson<UplinkGatewayTrial>(
    `/latency/uplink/trials/${trialId}/browser-samples`,
    {
      method: 'POST',
      body: JSON.stringify({ samples }),
    }
  )
}

export function cancelUplinkTrial(trialId: string) {
  return requestJson<UplinkGatewayTrial>(`/latency/uplink/trials/${trialId}`, {
    method: 'DELETE',
  })
}
