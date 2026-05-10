import { useEffect, useMemo, useState } from 'react'
import { useRosBridge } from './useRosBridge'
import type { JointStateMsg } from '../lib/rosMessages'

const ARM_JOINT_NAMES = ['arm_j1', 'arm_j2', 'arm_j3', 'arm_j4', 'arm_j5', 'arm_j6'] as const
const STALE_AFTER_MS = 1500
const JOINT_STATE_THROTTLE_MS = 150

type ArmJointName = (typeof ARM_JOINT_NAMES)[number]

export type ManipulatorState = {
  connected: boolean
  joints: Record<ArmJointName, number>
  missingJoints: ArmJointName[]
  lastUpdateMs: number | null
  ageMs: number | null
  stale: boolean
}

const zeroJoints = ARM_JOINT_NAMES.reduce(
  (acc, name) => ({ ...acc, [name]: 0 }),
  {} as Record<ArmJointName, number>,
)

export const useManipulatorState = (): ManipulatorState => {
  const { ros, connected } = useRosBridge()
  const [joints, setJoints] = useState<Record<ArmJointName, number>>(zeroJoints)
  const [seen, setSeen] = useState<Set<ArmJointName>>(() => new Set())
  const [lastUpdateMs, setLastUpdateMs] = useState<number | null>(null)
  const [nowMs, setNowMs] = useState(() => Date.now())

  useEffect(() => {
    const unsubscribe = ros.subscribe<JointStateMsg>(
      '/joint_states',
      'sensor_msgs/msg/JointState',
      (message) => {
        const names = message.name ?? []
        const positions = message.position ?? []
        const updates: Partial<Record<ArmJointName, number>> = {}
        const touchedNames: ArmJointName[] = []

        for (const jointName of ARM_JOINT_NAMES) {
          const index = names.indexOf(jointName)
          const position = positions[index]
          if (index >= 0 && Number.isFinite(position)) {
            updates[jointName] = position
            touchedNames.push(jointName)
          }
        }

        if (touchedNames.length === 0) return

        setJoints((current) => ({ ...current, ...updates }))
        setSeen((current) => {
          const next = new Set(current)
          for (const jointName of touchedNames) next.add(jointName)
          return next
        })

        const updateMs = Date.now()
        setLastUpdateMs(updateMs)
        setNowMs(updateMs)
      },
      { throttleRate: JOINT_STATE_THROTTLE_MS, queueSize: 1 },
    )

    return () => unsubscribe()
  }, [ros])

  useEffect(() => {
    const timer = window.setInterval(() => setNowMs(Date.now()), 250)
    return () => window.clearInterval(timer)
  }, [])

  const ageMs = lastUpdateMs == null ? null : Math.max(0, nowMs - lastUpdateMs)
  const missingJoints = useMemo(
    () => ARM_JOINT_NAMES.filter((name) => !seen.has(name)),
    [seen],
  )

  return {
    connected,
    joints,
    missingJoints,
    lastUpdateMs,
    ageMs,
    stale: ageMs == null || ageMs > STALE_AFTER_MS,
  }
}

export { ARM_JOINT_NAMES }
