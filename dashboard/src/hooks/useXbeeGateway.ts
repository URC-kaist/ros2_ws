import { useEffect, useState } from 'react'
import { getXbeeGatewayClient } from '../lib/xbeeGateway'

export const useXbeeGateway = () => {
  const gateway = getXbeeGatewayClient()
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
