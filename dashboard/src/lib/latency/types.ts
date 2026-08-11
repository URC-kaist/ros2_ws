export type LatencyDistribution = {
  count: number
  minMs: number | null
  meanMs: number | null
  p50Ms: number | null
  p95Ms: number | null
  p99Ms: number | null
  maxMs: number | null
}

export type LatencySegmentResult = {
  id: string
  label: string
  boundaryStart: string
  boundaryEnd: string
  distribution: LatencyDistribution
}

export type DownlinkPhase =
  | 'idle'
  | 'checking_clocks'
  | 'waiting_for_neutral'
  | 'waiting_for_input'
  | 'waiting_for_trace'
  | 'verifying_clocks'
  | 'completed'
  | 'failed'
  | 'cancelled'

export type DownlinkError = {
  code: string
  message: string
}

export type UplinkPhase =
  | 'idle'
  | 'checking_clocks'
  | 'preparing'
  | 'capturing'
  | 'uploading'
  | 'analyzing'
  | 'completed'
  | 'failed'
  | 'cancelled'

export type DownlinkStage =
  | 'input_t0'
  | 'gateway_command_received_t1'
  | 'rover_command_received_t3'
  | 'rover_cmd_vel_published_t4'

export type DownlinkStages = Partial<Record<DownlinkStage, number>>

export type DownlinkMetrics = {
  browserToGatewayMs: number | null
  gatewayToRoverMs: number | null
  roverReceiveToPublishMs: number | null
  totalMs: number | null
  browserClockDriftUs: number | null
  valid: boolean
  warning: string | null
}

export type DownlinkTrialSnapshot = {
  id: string
  phase: DownlinkPhase
  stages: DownlinkStages
  metrics: DownlinkMetrics
  segments: LatencySegmentResult[]
  startedAtEpochUs: number
  completedAtEpochUs: number | null
  clock: {
    startBrowserToBaseOffsetUs: number | null
    startBrowserRttUs: number | null
    endBrowserToBaseOffsetUs: number | null
    endBrowserRttUs: number | null
    browserClockDriftUs: number | null
  }
  error: DownlinkError | null
}
