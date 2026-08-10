import { useSyncExternalStore } from 'react'
import { latencyDiagnostics } from '../lib/latencyDiagnostics'

export const useLatencyDiagnostics = () =>
  useSyncExternalStore(
    latencyDiagnostics.subscribe,
    latencyDiagnostics.getSnapshot,
    latencyDiagnostics.getSnapshot
  )
