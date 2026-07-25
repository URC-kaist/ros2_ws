import { useEffect, useState } from 'react'
import { FiMaximize2, FiMinimize2 } from 'react-icons/fi'
import ControlPanel from './components/ControlPanel'
import MissionPanels from './components/MissionPanels'
import MissionTabs, { type MissionTabId } from './components/MissionTabs'
import { isRoverDirectProfile } from './lib/operatingProfile'
import './components/layout.css'

const App = () => {
  const [activeTab, setActiveTab] = useState<MissionTabId>(
    isRoverDirectProfile ? 'live-feed' : 'science'
  )
  const [isFullscreen, setIsFullscreen] = useState(false)

  useEffect(() => {
    const syncFullscreen = () => setIsFullscreen(Boolean(document.fullscreenElement))
    document.addEventListener('fullscreenchange', syncFullscreen)
    return () => document.removeEventListener('fullscreenchange', syncFullscreen)
  }, [])

  const toggleFullscreen = () => {
    if (document.fullscreenElement) {
      document.exitFullscreen().catch(() => undefined)
    } else {
      document.documentElement.requestFullscreen?.().catch(() => undefined)
    }
  }

  return (
    <div className="app">
      <button
        className="fullscreen-btn"
        type="button"
        aria-pressed={isFullscreen}
        aria-label={isFullscreen ? 'Exit fullscreen' : 'Enter fullscreen'}
        onClick={toggleFullscreen}
      >
        {isFullscreen ? <FiMinimize2 aria-hidden="true" /> : <FiMaximize2 aria-hidden="true" />}
      </button>
      <ControlPanel />
      <main className="main">
        <MissionTabs activeTab={activeTab} onTabChange={setActiveTab} />
        <MissionPanels activeTab={activeTab} />
      </main>
    </div>
  )
}

export default App
