import { resolveRosBridgeUrl } from './rosBridge'

type ActionStatusMessage = {
  op: 'status'
  id?: string
  level?: string
  msg?: string
}

type ActionFeedbackMessage<TFeedback> = {
  op: 'action_feedback'
  id?: string
  action?: string
  values?: TFeedback
}

type ActionResultMessage<TResult> = {
  op: 'action_result'
  id?: string
  action?: string
  values?: TResult | string
  status?: number
  result?: boolean
}

type FragmentMessage = {
  op: 'fragment'
  id: string
  data: string
  num: number
  total: number
}

type RosbridgeMessage<TFeedback, TResult> =
  | ActionStatusMessage
  | ActionFeedbackMessage<TFeedback>
  | ActionResultMessage<TResult>
  | FragmentMessage

export type ActionGoalOptions<TGoal, TFeedback, TResult> = {
  action: string
  actionType: string
  goal: TGoal
  onFeedback?: (feedback: TFeedback) => void
  onResult?: (result: TResult, status: number | undefined, successful: boolean) => void
  onError?: (message: string) => void
}

export type ActiveActionGoal = {
  id: string
  cancel: () => void
  dispose: () => void
}

const makeGoalId = (action: string) =>
  `${action.replace(/[^a-zA-Z0-9]/g, '_')}:${Date.now()}:${Math.random()
    .toString(16)
    .slice(2)}`

const getStatusMessage = (message: ActionStatusMessage) => {
  if (typeof message.msg === 'string' && message.msg.trim()) return message.msg
  return 'rosbridge action status reported an error'
}

export const sendRosbridgeActionGoal = <TGoal, TFeedback, TResult>({
  action,
  actionType,
  goal,
  onFeedback,
  onResult,
  onError,
}: ActionGoalOptions<TGoal, TFeedback, TResult>): ActiveActionGoal => {
  const goalId = makeGoalId(action)
  const socket = new WebSocket(resolveRosBridgeUrl())
  const fragments = new Map<string, string[]>()
  let disposed = false
  let cancelRequested = false

  const dispose = () => {
    disposed = true
    fragments.clear()
    if (socket.readyState === WebSocket.OPEN || socket.readyState === WebSocket.CONNECTING) {
      socket.close()
    }
  }

  const send = (message: Record<string, unknown>) => {
    if (socket.readyState !== WebSocket.OPEN) return
    socket.send(JSON.stringify(message))
  }

  const handleMessage = (rawMessage: RosbridgeMessage<TFeedback, TResult>) => {
    if (disposed) return

    if (rawMessage.op === 'fragment') {
      const current = fragments.get(rawMessage.id) ?? []
      current[rawMessage.num] = rawMessage.data
      fragments.set(rawMessage.id, current)
      if (current.filter((part) => part != null).length !== rawMessage.total) return
      fragments.delete(rawMessage.id)
      handleMessage(JSON.parse(current.join('')) as RosbridgeMessage<TFeedback, TResult>)
      return
    }

    if ('id' in rawMessage && rawMessage.id && rawMessage.id !== goalId) return

    if (rawMessage.op === 'status') {
      if (rawMessage.level === 'error') {
        onError?.(getStatusMessage(rawMessage))
      }
      return
    }

    if (rawMessage.op === 'action_feedback') {
      if (rawMessage.action && rawMessage.action !== action) return
      if (rawMessage.values) onFeedback?.(rawMessage.values as TFeedback)
      return
    }

    if (rawMessage.op === 'action_result') {
      if (rawMessage.action && rawMessage.action !== action) return
      if (typeof rawMessage.values === 'string') {
        onError?.(rawMessage.values)
      } else if (rawMessage.values) {
        onResult?.(rawMessage.values as TResult, rawMessage.status, Boolean(rawMessage.result))
      } else {
        onError?.('Action completed without a result payload')
      }
      dispose()
    }
  }

  socket.addEventListener('open', () => {
    send({
      op: 'send_action_goal',
      id: goalId,
      action,
      action_type: actionType,
      args: goal,
      feedback: true,
    })
    if (cancelRequested) {
      send({
        op: 'cancel_action_goal',
        id: goalId,
        action,
      })
    }
  })

  socket.addEventListener('message', (event) => {
    try {
      const data =
        typeof event.data === 'string'
          ? event.data
          : event.data instanceof Blob
            ? null
            : String(event.data)
      if (!data) {
        onError?.('Unsupported rosbridge action message encoding')
        return
      }
      handleMessage(JSON.parse(data) as RosbridgeMessage<TFeedback, TResult>)
    } catch (error) {
      onError?.(error instanceof Error ? error.message : 'Failed to parse action response')
    }
  })

  socket.addEventListener('error', () => {
    onError?.('ROS bridge action socket failed')
  })

  socket.addEventListener('close', () => {
    if (!disposed) {
      disposed = true
      onError?.('ROS bridge action socket closed')
    }
  })

  return {
    id: goalId,
    cancel: () => {
      if (disposed) return
      cancelRequested = true
      send({
        op: 'cancel_action_goal',
        id: goalId,
        action,
      })
    },
    dispose,
  }
}
