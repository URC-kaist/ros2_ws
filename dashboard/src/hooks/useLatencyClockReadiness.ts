import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  evaluateChronyReadiness,
  fetchBaseChronyStatus,
  parseRoverChronyDiagnostic,
  type ChronyStatus,
} from '../lib/chronyStatus'
import type { DiagnosticArray } from '../lib/rosMessages'
import { getRosBridgeClient } from '../lib/rosBridge'

const BASE_POLL_INTERVAL_MS = 2_000
const READINESS_TICK_MS = 1_000

type RosBridgeClient = ReturnType<typeof getRosBridgeClient>

export function useLatencyClockReadiness(ros: RosBridgeClient) {
  const [base, setBase] = useState<ChronyStatus | null>(null)
  const [rover, setRover] = useState<ChronyStatus | null>(null)
  const [baseError, setBaseError] = useState<string | null>(null)
  const [roverError, setRoverError] = useState<string | null>(null)
  const [checking, setChecking] = useState(false)
  const [nowEpochMs, setNowEpochMs] = useState(Date.now())
  const baseRef = useRef<ChronyStatus | null>(null)
  const roverRef = useRef<ChronyStatus | null>(null)
  const refreshPromiseRef = useRef<Promise<ChronyStatus | null> | null>(null)
  const mountedRef = useRef(true)

  useEffect(() => {
    mountedRef.current = true
    return () => {
      mountedRef.current = false
    }
  }, [])

  const refreshBase = useCallback(() => {
    if (refreshPromiseRef.current) return refreshPromiseRef.current
    const request = fetchBaseChronyStatus()
      .then((status) => {
        baseRef.current = status
        if (mountedRef.current) {
          setBase(status)
          setBaseError(null)
          setNowEpochMs(Date.now())
        }
        return status
      })
      .catch((error: unknown) => {
        const message = error instanceof Error ? error.message : 'Chrony status check failed'
        baseRef.current = null
        if (mountedRef.current) {
          setBase(null)
          setBaseError(message)
          setNowEpochMs(Date.now())
        }
        return null
      })
      .finally(() => {
        refreshPromiseRef.current = null
      })
    refreshPromiseRef.current = request
    return request
  }, [])

  const checkNow = useCallback(async () => {
    setChecking(true)
    try {
      const nextBase = await refreshBase()
      const now = Date.now()
      if (mountedRef.current) setNowEpochMs(now)
      return evaluateChronyReadiness(nextBase, roverRef.current, now)
    } finally {
      if (mountedRef.current) setChecking(false)
    }
  }, [refreshBase])

  useEffect(() => {
    return ros.subscribe<DiagnosticArray>(
      '/system_status/clock',
      'diagnostic_msgs/DiagnosticArray',
      (message) => {
        try {
          const status = parseRoverChronyDiagnostic(message)
          roverRef.current = status
          setRover(status)
          setRoverError(null)
          setNowEpochMs(Date.now())
        } catch (error) {
          const message =
            error instanceof Error ? error.message : 'Invalid rover chrony status'
          roverRef.current = null
          setRover(null)
          setRoverError(message)
        }
      },
      { throttleRate: 500, queueSize: 5 }
    )
  }, [ros])

  useEffect(() => {
    void refreshBase()
    const pollTimer = window.setInterval(() => void refreshBase(), BASE_POLL_INTERVAL_MS)
    const readinessTimer = window.setInterval(
      () => setNowEpochMs(Date.now()),
      READINESS_TICK_MS
    )
    return () => {
      window.clearInterval(pollTimer)
      window.clearInterval(readinessTimer)
    }
  }, [refreshBase])

  const readiness = useMemo(
    () => evaluateChronyReadiness(base, rover, nowEpochMs),
    [base, nowEpochMs, rover]
  )

  return {
    base,
    rover,
    readiness,
    checking,
    error: roverError || baseError,
    checkNow,
  }
}
