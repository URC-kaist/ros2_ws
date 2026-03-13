export type CmdDrive = {
  linear_x_m_s: number
  linear_y_m_s: number
  angular_z_rad_s: number
}

export type CmdArmTwist = {
  lin_x_m_s: number
  lin_y_m_s: number
  lin_z_m_s: number
  ang_x_rad_s: number
  ang_y_rad_s: number
  ang_z_rad_s: number
}

export type CmdArmGripper = {
  position_norm: number
}

export type MissionControl = {
  command: number
  clear_costmap: boolean
  mission_id: number
}

export type BatteryId = 1 | 2

export type TelemBattery = {
  battery_id: BatteryId
  total_capacity_mah: number
  available_capacity_mah: number
  temperature_c: number
  pack_voltage_v: number
}

export type TelemNav = {
  timestamp_ms: number
  latitude_deg: number
  longitude_deg: number
  altitude_m: number
  heading_deg: number
  cov_x_var: number
  cov_y_var: number
  cov_yaw_var: number
}

export type LinkStatus = {
  connected: boolean
  last_rx_ms: number
  last_tx_ms: number
}

export type BaseStatus = {
  enabled: boolean
  antenna_ready: boolean
  auto_home: boolean
  heading_offset_deg: number
  base_lat_deg: number | null
  base_lon_deg: number | null
  base_alt_m: number | null
  antenna_heading_deg: number | null
  last_cmd_heading_deg: number | null
  last_cmd_age_ms: number | null
  base_fix_age_ms: number | null
  rover_nav_age_ms: number | null
  base_fix_valid: boolean
  rover_nav_valid: boolean
  idle_reason: string
}

export type RocketM2Status = {
  connected: boolean
  updated_at_ms: number
  last_success_ms: number | null
  signal: number | null
  rssi: number | null
  noisef: number | null
  chwidth: number | null
  rx_chainmask: number | null
  chainrssi: number[]
  chainrssimgmt: number[]
  chainrssiext: number[]
  error: string | null
}

type MessageHandler<T> = (payload: T) => void

type RawTelemBattery = {
  type: 'telem_battery'
  battery_id?: number
  total_capacity_mah: number
  available_capacity_mah: number
  temperature_c: number
  pack_voltage_v: number
}

type RawTelemNav = {
  type: 'telem_nav'
  timestamp_ms: number
  latitude_deg: number
  longitude_deg: number
  altitude_m: number
  heading_deg: number
  cov_x_var: number
  cov_y_var: number
  cov_yaw_var: number
}

type RawBaseStatus = {
  type: 'base_status'
} & BaseStatus

type RawRocketM2Status = {
  type: 'rocket_m2_status'
} & RocketM2Status

type GatewayMessage =
  | RawTelemBattery
  | RawTelemNav
  | RawBaseStatus
  | RawRocketM2Status
  | ({ type: 'link_status' } & LinkStatus)

const DEFAULT_PATH = '/sik-ws'
const HEARTBEAT_TIMEOUT_MS = 2000
const HEARTBEAT_POLL_MS = 500
const RECONNECT_BASE_MS = 500
const RECONNECT_MAX_MS = 5000

const resolveGatewayUrl = () => {
  const explicit = import.meta.env.VITE_SIK_WS_URL as string | undefined
  if (explicit) {
    return explicit
  }
  if (typeof window === 'undefined') {
    return `ws://localhost:8081${DEFAULT_PATH}`
  }
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  return `${protocol}//${window.location.host}${DEFAULT_PATH}`
}

class SikGatewayClient {
  private ws: WebSocket | null = null
  private connected = false
  private lastRxAt = 0
  private lastTxAt = 0
  private heartbeatTimer: number | null = null
  private reconnectTimer: number | null = null
  private reconnectDelayMs = RECONNECT_BASE_MS
  private lastLinkStatus: LinkStatus | null = null
  private linkConnected = false
  private linkListeners = new Set<MessageHandler<LinkStatus>>()
  private connectionListeners = new Set<MessageHandler<boolean>>()
  private batteryListeners = new Set<MessageHandler<TelemBattery>>()
  private navListeners = new Set<MessageHandler<TelemNav>>()
  private baseStatusListeners = new Set<MessageHandler<BaseStatus>>()
  private rocketM2Listeners = new Set<MessageHandler<RocketM2Status>>()
  private pendingBaseHeading: number | null = null
  private url: string

  constructor(url: string) {
    this.url = url
  }

  connect() {
    if (this.ws || this.connected) return
    this.clearReconnectTimer()
    this.ws = new WebSocket(this.url)
    this.ws.addEventListener('open', () => {
      this.connected = true
      this.reconnectDelayMs = RECONNECT_BASE_MS
      this.startHeartbeatMonitor()
      this.emitConnectionStatus(true)
      if (this.pendingBaseHeading != null) {
        this.sendBaseHeading(this.pendingBaseHeading)
        this.pendingBaseHeading = null
      }
    })
    this.ws.addEventListener('close', () => {
      this.connected = false
      this.ws = null
      this.stopHeartbeatMonitor()
      this.emitConnectionStatus(false)
      this.scheduleReconnect()
      if (this.linkConnected) {
        this.linkConnected = false
        this.emitLinkStatus({ connected: false, last_rx_ms: this.lastRxAt, last_tx_ms: this.lastTxAt })
      }
    })
    this.ws.addEventListener('error', () => {
      this.connected = false
    })
    this.ws.addEventListener('message', (event) => {
      this.lastRxAt = Date.now()
      const message = this.safeParse(event.data)
      if (!message) return
      if (message.type === 'telem_battery') {
        const batteryId = message.battery_id === 2 ? 2 : 1
        const payload: TelemBattery = {
          battery_id: batteryId,
          total_capacity_mah: message.total_capacity_mah,
          available_capacity_mah: message.available_capacity_mah,
          temperature_c: message.temperature_c,
          pack_voltage_v: message.pack_voltage_v,
        }
        for (const listener of this.batteryListeners) {
          listener(payload)
        }
      } else if (message.type === 'telem_nav') {
        const payload: TelemNav = {
          timestamp_ms: message.timestamp_ms,
          latitude_deg: message.latitude_deg,
          longitude_deg: message.longitude_deg,
          altitude_m: message.altitude_m,
          heading_deg: message.heading_deg,
          cov_x_var: message.cov_x_var,
          cov_y_var: message.cov_y_var,
          cov_yaw_var: message.cov_yaw_var,
        }
        for (const listener of this.navListeners) {
          listener(payload)
        }
      } else if (message.type === 'base_status') {
        for (const listener of this.baseStatusListeners) {
          listener(message)
        }
      } else if (message.type === 'rocket_m2_status') {
        for (const listener of this.rocketM2Listeners) {
          listener(message)
        }
      } else if (message.type === 'link_status') {
        this.lastLinkStatus = message
        this.linkConnected = message.connected
        for (const listener of this.linkListeners) {
          listener(message)
        }
      }
    })
  }

  onLinkStatus(handler: MessageHandler<LinkStatus>) {
    this.linkListeners.add(handler)
    return () => {
      this.linkListeners.delete(handler)
    }
  }

  onConnectionStatus(handler: MessageHandler<boolean>) {
    this.connectionListeners.add(handler)
    return () => {
      this.connectionListeners.delete(handler)
    }
  }

  onTelemBattery(handler: MessageHandler<TelemBattery>) {
    this.batteryListeners.add(handler)
    return () => {
      this.batteryListeners.delete(handler)
    }
  }

  onTelemNav(handler: MessageHandler<TelemNav>) {
    this.navListeners.add(handler)
    return () => {
      this.navListeners.delete(handler)
    }
  }

  onBaseStatus(handler: MessageHandler<BaseStatus>) {
    this.baseStatusListeners.add(handler)
    return () => {
      this.baseStatusListeners.delete(handler)
    }
  }

  onRocketM2Status(handler: MessageHandler<RocketM2Status>) {
    this.rocketM2Listeners.add(handler)
    return () => {
      this.rocketM2Listeners.delete(handler)
    }
  }

  sendCmdDrive(cmd: CmdDrive) {
    this.send({
      type: 'cmd_drive',
      linear_x_m_s: cmd.linear_x_m_s,
      linear_y_m_s: cmd.linear_y_m_s,
      angular_z_rad_s: cmd.angular_z_rad_s,
    })
  }

  sendCmdArmTwist(cmd: CmdArmTwist) {
    this.send({
      type: 'cmd_arm_twist',
      ...cmd,
    })
  }

  sendCmdArmGripper(cmd: CmdArmGripper) {
    this.send({
      type: 'cmd_arm_gripper',
      position_norm: cmd.position_norm,
    })
  }

  sendHeartbeat() {
    this.send({ type: 'heartbeat' })
  }

  sendMissionControl(control: MissionControl) {
    this.send({
      type: 'mission_control',
      command: control.command,
      clear_costmap: control.clear_costmap,
      mission_id: control.mission_id,
    })
  }

  sendBaseHeading(headingDeg: number) {
    if (!Number.isFinite(headingDeg)) return
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) {
      this.pendingBaseHeading = headingDeg
      this.connect()
      return
    }
    this.send({
      type: 'base_heading',
      heading_deg: headingDeg,
    })
  }

  private send(payload: Record<string, unknown>) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return
    this.lastTxAt = Date.now()
    this.ws.send(JSON.stringify(payload))
  }

  private safeParse(raw: unknown): GatewayMessage | null {
    if (typeof raw !== 'string') return null
    try {
      return JSON.parse(raw) as GatewayMessage
    } catch {
      return null
    }
  }

  private startHeartbeatMonitor() {
    if (this.heartbeatTimer != null) return
    this.heartbeatTimer = window.setInterval(() => {
      if (!this.connected) return
      const now = Date.now()
      if (this.lastRxAt === 0) return
      if (now - this.lastRxAt <= HEARTBEAT_TIMEOUT_MS) return
      if (!this.linkConnected) return
      this.linkConnected = false
      this.emitLinkStatus({
        connected: false,
        last_rx_ms: this.lastRxAt,
        last_tx_ms: this.lastTxAt,
      })
    }, HEARTBEAT_POLL_MS)
  }

  private stopHeartbeatMonitor() {
    if (this.heartbeatTimer == null) return
    window.clearInterval(this.heartbeatTimer)
    this.heartbeatTimer = null
  }

  private emitLinkStatus(status: LinkStatus) {
    for (const listener of this.linkListeners) {
      listener(status)
    }
  }

  private emitConnectionStatus(connected: boolean) {
    for (const listener of this.connectionListeners) {
      listener(connected)
    }
  }

  private scheduleReconnect() {
    if (this.reconnectTimer != null) return
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null
      this.connect()
      this.reconnectDelayMs = Math.min(this.reconnectDelayMs * 2, RECONNECT_MAX_MS)
    }, this.reconnectDelayMs)
  }

  private clearReconnectTimer() {
    if (this.reconnectTimer == null) return
    window.clearTimeout(this.reconnectTimer)
    this.reconnectTimer = null
  }
}

let singleton: SikGatewayClient | null = null

export const getSikGatewayClient = () => {
  if (!singleton) {
    singleton = new SikGatewayClient(resolveGatewayUrl())
  }
  return singleton
}
