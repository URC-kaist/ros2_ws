import { useEffect, useRef, useState } from 'react'

const POLL_INTERVAL_MS = 1000

type GamepadInfo = { index: number; id: string }

const MicroscopeViewCard = () => {
  const [gamepads, setGamepads] = useState<GamepadInfo[]>([])
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null)
  const [connected, setConnected] = useState(false)
  const selectedRef = useRef<number | null>(null)

  useEffect(() => {
    selectedRef.current = selectedIndex
  }, [selectedIndex])

  useEffect(() => {
    const updatePads = () => {
      const pads = navigator.getGamepads?.() ?? []
      const list = pads.reduce<GamepadInfo[]>((acc, pad, index) => {
        if (pad) acc.push({ index, id: pad.id || `Gamepad ${index + 1}` })
        return acc
      }, [])

      setGamepads(list)

      if (list.length === 0) {
        setSelectedIndex(null)
        setConnected(false)
        return
      }

      const current = selectedRef.current
      const currentPad = current != null ? pads[current] : null
      const nextIndex = currentPad ? current : list[0].index

      if (current !== nextIndex) {
        setSelectedIndex(nextIndex)
      }

      setConnected(Boolean(pads[nextIndex]))
    }

    updatePads()
    window.addEventListener('gamepadconnected', updatePads)
    window.addEventListener('gamepaddisconnected', updatePads)
    const interval = window.setInterval(updatePads, POLL_INTERVAL_MS)

    return () => {
      window.removeEventListener('gamepadconnected', updatePads)
      window.removeEventListener('gamepaddisconnected', updatePads)
      window.clearInterval(interval)
    }
  }, [])

  return (
    <article className="card camera-turret-card microscope-card">
      <header className="camera-turret__header">
        <h3>Microscope View</h3>
        <span className={`pill ${connected ? 'pill--on' : 'pill--off'}`}>
          {connected ? 'gamepad connected' : 'no gamepad'}
        </span>
      </header>

      <label className="camera-turret__picker">
        <span className="camera-turret__label">Joystick</span>
        <select
          value={selectedIndex ?? ''}
          onChange={(event) => {
            const next = event.target.value === '' ? null : Number(event.target.value)
            setSelectedIndex(next)
            const pads = navigator.getGamepads?.() ?? []
            setConnected(next != null && Boolean(pads[next]))
          }}
        >
          <option value="">No control</option>
          {gamepads.length === 0 ? (
            <option value="" disabled>
              No gamepad detected
            </option>
          ) : (
            gamepads.map((pad) => (
              <option key={pad.index} value={pad.index}>
                {pad.id}
              </option>
            ))
          )}
        </select>
      </label>

      <div className="camera-turret__feed">
        <div className="video-feed-placeholder camera-turret__placeholder">Awaiting stream...</div>
      </div>
      <p className="camera-turret__hint">RS: fine focus · LT/RT: zoom</p>
    </article>
  )
}

export default MicroscopeViewCard
