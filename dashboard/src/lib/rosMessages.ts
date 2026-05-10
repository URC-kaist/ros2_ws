import type { MissionSpec } from './missions'

export type RosStamp = {
  sec?: number
  nanosec?: number
  secs?: number
  nsecs?: number
}

export type RosHeader = {
  stamp?: RosStamp
  frame_id?: string
}

export type JointStateMsg = {
  header?: RosHeader
  name?: string[]
  position?: number[]
  velocity?: number[]
  effort?: number[]
}

export type TransformStampedMsg = {
  header?: RosHeader
  child_frame_id?: string
  transform?: {
    translation?: {
      x?: number
      y?: number
      z?: number
    }
    rotation?: {
      x?: number
      y?: number
      z?: number
      w?: number
    }
  }
}

export type TFMessageMsg = {
  transforms?: TransformStampedMsg[]
}

export type PoseStamped = {
  header?: {
    stamp?: RosStamp
  }
  pose?: {
    position?: {
      x?: number
      y?: number
      z?: number
    }
  }
}

export type GeoPoseStamped = {
  pose?: {
    position?: {
      latitude?: number
      longitude?: number
      altitude?: number
    }
  }
}

export type GeoPathMsg = {
  poses?: GeoPoseStamped[]
}

export type GpsFix = {
  fix_type?: number
}

export type CarrSoln = {
  status?: number
}

export type UBXNavStatus = {
  gps_fix?: GpsFix
  gps_fix_ok?: boolean
  diff_soln?: boolean
  diff_corr?: boolean
  carr_soln_valid?: boolean
  carr_soln?: CarrSoln
}

export type UBXNavHPPosLLH = {
  h_acc?: number
  v_acc?: number
}

export type MissionListMsg = {
  stamp?: { sec?: number; nanosec?: number }
  missions?: MissionSpec[]
}

export type MissionControlMsg = {
  command: number
  clear_costmap: boolean
  mission_id: number
}

export type MissionStatusMsg = {
  stamp?: { sec?: number; nanosec?: number }
  active_mission?: MissionSpec
  state?: number
  arrival?: boolean
  current_waypoint_index?: number
  total_waypoints?: number
  distance_remaining?: number
  detail?: string
}

export type DiagnosticKeyValue = {
  key: string
  value: string
}

export type DiagnosticStatus = {
  level: number
  name: string
  message: string
  values: DiagnosticKeyValue[]
}

export type DiagnosticArray = {
  status: DiagnosticStatus[]
}

export type PackTelemetry = {
  state_of_charge_pct?: number
  health_pct?: number
  temperature_c?: number
  pack_voltage_v?: number
  pack_life_cycles?: number
  firmware_cycle_count?: number
  nominal_cell_capacity_mah?: number
  parallel_group_count?: number
  cell_count?: number
  cell_voltage_mv?: number[]
  cell_voltage_valid?: boolean[]
}

export type ToLLRequest = {
  map_point: {
    x: number
    y: number
    z: number
  }
}

export type ToLLResponse = {
  ll_point?: {
    latitude?: number
    longitude?: number
    altitude?: number
  }
}

export type SpectrumMsg = {
  header?: { stamp?: { sec?: number; nanosec?: number }; frame_id?: string }
  wavelength_nm: number[]
  intensity: number[]
  mode: number
}

export type GetSpectrumRequest = {
  use_absorbance: boolean
  publish_topic: boolean
}

export type GetSpectrumResponse = {
  success: boolean
  message: string
  spectrum: SpectrumMsg
}
