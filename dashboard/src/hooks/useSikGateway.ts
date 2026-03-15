import { useEffect, useState } from 'react'
import { getSikGatewayClient } from '../lib/sikGateway'

export const useSikGateway = () => {
  const gateway = getSikGatewayClient()
  const [connected, setConnected] = useState(() => gateway.isConnected())

  useEffect(() => {
    gateway.connect()
    const unsubscribe = gateway.onConnectionStatus(setConnected)
    return () => unsubscribe()
  }, [gateway])

  return {
    gateway,
    connected,
  }
}
