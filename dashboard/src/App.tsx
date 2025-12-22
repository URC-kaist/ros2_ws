import { useState } from 'react'
import ControlPanel from './components/ControlPanel'
import MissionPanels from './components/MissionPanels'
import MissionTabs, { type MissionTabId } from './components/MissionTabs'
import './components/layout.css'

const App = () => {
  const [activeTab, setActiveTab] = useState<MissionTabId>('science')

  return (
    <div className="app">
      <ControlPanel />
      <main className="main">
        <MissionTabs activeTab={activeTab} onTabChange={setActiveTab} />
        <MissionPanels activeTab={activeTab} />
      </main>
    </div>
  )
}

export default App
