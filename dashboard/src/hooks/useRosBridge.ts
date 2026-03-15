import { useEffect, useState } from 'react'
import { getRosBridgeClient } from '../lib/rosBridge'

export const useRosBridge = () => {
  const ros = getRosBridgeClient()
  const [connected, setConnected] = useState(() => ros.isConnected())

  useEffect(() => {
    ros.connect()
    const unsubscribe = ros.onConnectionStatus(setConnected)
    return () => unsubscribe()
  }, [ros])

  return {
    ros,
    connected,
  }
}
