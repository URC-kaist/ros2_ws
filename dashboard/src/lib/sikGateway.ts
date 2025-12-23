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

export type TelemBattery = {
  total_capacity_mah: number
  available_capacity_mah: number
  temperature_c: number
}

export type LinkStatus = {
  connected: boolean
  last_rx_ms: number
  last_tx_ms: number
}

type MessageHandler<T> = (payload: T) => void

type GatewayMessage =
  | ({ type: 'telem_battery' } & TelemBattery)
  | ({ type: 'link_status' } & LinkStatus)

const DEFAULT_PATH = '/sik-ws'

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
  private linkListeners = new Set<MessageHandler<LinkStatus>>()
  private batteryListeners = new Set<MessageHandler<TelemBattery>>()
  private url: string

  constructor(url: string) {
    this.url = url
  }

  connect() {
    if (this.ws || this.connected) return
    this.ws = new WebSocket(this.url)
    this.ws.addEventListener('open', () => {
      this.connected = true
    })
    this.ws.addEventListener('close', () => {
      this.connected = false
      this.ws = null
    })
    this.ws.addEventListener('message', (event) => {
      const message = this.safeParse(event.data)
      if (!message) return
      if (message.type === 'telem_battery') {
        for (const listener of this.batteryListeners) {
          listener(message)
        }
      } else if (message.type === 'link_status') {
        for (const listener of this.linkListeners) {
          listener(message)
        }
      }
    })
  }

  onLinkStatus(handler: MessageHandler<LinkStatus>) {
    this.linkListeners.add(handler)
    return () => this.linkListeners.delete(handler)
  }

  onTelemBattery(handler: MessageHandler<TelemBattery>) {
    this.batteryListeners.add(handler)
    return () => this.batteryListeners.delete(handler)
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

  sendHeartbeat() {
    this.send({ type: 'heartbeat' })
  }

  private send(payload: Record<string, unknown>) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return
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
}

let singleton: SikGatewayClient | null = null

export const getSikGatewayClient = () => {
  if (!singleton) {
    singleton = new SikGatewayClient(resolveGatewayUrl())
  }
  return singleton
}
