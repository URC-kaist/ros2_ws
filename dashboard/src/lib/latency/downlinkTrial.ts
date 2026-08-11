import { distribution } from './statistics'
import type {
  DownlinkMetrics,
  DownlinkPhase,
  DownlinkStages,
  LatencySegmentResult,
} from './types'

export const DOWNLINK_ACTIVE_PHASES = new Set<DownlinkPhase>([
  'checking_clocks',
  'waiting_for_neutral',
  'waiting_for_input',
  'waiting_for_trace',
  'verifying_clocks',
])

export function isDownlinkActive(phase: DownlinkPhase) {
  return DOWNLINK_ACTIVE_PHASES.has(phase)
}

export function downlinkPhaseLabel(phase: DownlinkPhase) {
  switch (phase) {
    case 'checking_clocks':
      return 'Checking clocks'
    case 'waiting_for_neutral':
      return 'Return controller to neutral'
    case 'waiting_for_input':
      return 'Send one command'
    case 'waiting_for_trace':
      return 'Waiting for rover trace'
    case 'verifying_clocks':
      return 'Verifying clocks'
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

function deltaMs(end: number | undefined, start: number | undefined) {
  return end != null && start != null ? (end - start) / 1000 : null
}

export function calculateDownlinkMetrics(
  stages: DownlinkStages,
  browserToBaseOffsetUs: number | null,
  browserClockDriftUs: number | null = null
): DownlinkMetrics {
  const t0Base =
    stages.input_t0 != null && browserToBaseOffsetUs != null
      ? stages.input_t0 + browserToBaseOffsetUs
      : undefined
  const browserToGatewayMs = deltaMs(stages.gateway_command_received_t1, t0Base)
  const gatewayToRoverMs = deltaMs(
    stages.rover_command_received_t3,
    stages.gateway_command_received_t1
  )
  const roverReceiveToPublishMs = deltaMs(
    stages.rover_cmd_vel_published_t4,
    stages.rover_command_received_t3
  )
  const totalMs = deltaMs(stages.rover_cmd_vel_published_t4, t0Base)
  const complete = [
    browserToGatewayMs,
    gatewayToRoverMs,
    roverReceiveToPublishMs,
    totalMs,
  ].every((value) => value != null && Number.isFinite(value))
  const hasNegativeSegment = [
    browserToGatewayMs,
    gatewayToRoverMs,
    roverReceiveToPublishMs,
  ].some((value) => value != null && value < 0)
  const hasExcessiveClockDrift =
    browserClockDriftUs != null && Math.abs(browserClockDriftUs) > 2_000

  return {
    browserToGatewayMs,
    gatewayToRoverMs,
    roverReceiveToPublishMs,
    totalMs,
    browserClockDriftUs,
    valid: complete && !hasNegativeSegment && !hasExcessiveClockDrift,
    warning: hasNegativeSegment
      ? 'A segment is negative; verify browser/base offset and chrony synchronization.'
      : hasExcessiveClockDrift
        ? 'Browser/base clock offset drift exceeded 2 ms during the trial.'
      : null,
  }
}

export function buildDownlinkSegments(metrics: DownlinkMetrics): LatencySegmentResult[] {
  const segment = (
    id: string,
    label: string,
    boundaryStart: string,
    boundaryEnd: string,
    value: number | null
  ): LatencySegmentResult => ({
    id,
    label,
    boundaryStart,
    boundaryEnd,
    distribution: distribution(value == null ? [] : [value]),
  })

  return [
    segment(
      'browser_to_gateway',
      'Browser input -> Base gateway',
      'browser controller input',
      'base gateway command receipt',
      metrics.browserToGatewayMs
    ),
    segment(
      'gateway_to_rover',
      'Base gateway -> Rover receive',
      'base gateway command receipt',
      'rover XBEE command receipt',
      metrics.gatewayToRoverMs
    ),
    segment(
      'rover_receive_to_publish',
      'Rover receive -> Command publish',
      'rover XBEE command receipt',
      'rover /base/cmd_vel publish',
      metrics.roverReceiveToPublishMs
    ),
    segment(
      'downlink_total',
      'Total',
      'browser controller input',
      'rover /base/cmd_vel publish',
      metrics.totalMs
    ),
  ]
}
