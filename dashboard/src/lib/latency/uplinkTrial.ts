import type { UplinkGatewayPhase } from './latencyApi'
import type { UplinkPhase } from './types'

export const UPLINK_ACTIVE_PHASES = new Set<UplinkPhase>([
  'checking_clocks',
  'preparing',
  'capturing',
  'uploading',
  'analyzing',
])

export function isUplinkActive(phase: UplinkPhase) {
  return UPLINK_ACTIVE_PHASES.has(phase)
}

export function uplinkPhaseLabel(phase: UplinkPhase) {
  switch (phase) {
    case 'checking_clocks':
      return 'Checking clocks'
    case 'preparing':
      return 'Configuring video feeds'
    case 'capturing':
      return 'Capturing RTP'
    case 'uploading':
      return 'Uploading rover capture'
    case 'analyzing':
      return 'Analyzing RTP captures'
    case 'completed':
      return 'Measurement complete'
    case 'failed':
      return 'Measurement failed'
    case 'cancelled':
      return 'Measurement cancelled'
    default:
      return 'Ready'
  }
}

export function mapGatewayUplinkPhase(phase: UplinkGatewayPhase): UplinkPhase {
  switch (phase) {
    case 'created':
    case 'base_capturing':
      return 'preparing'
    case 'waiting_for_rover_artifacts':
      return 'capturing'
    case 'stopping_base_capture':
      return 'uploading'
    case 'analyzing':
      return 'analyzing'
    case 'completed':
      return 'completed'
    case 'failed':
      return 'failed'
    case 'cancelled':
      return 'cancelled'
  }
}
