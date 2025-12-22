import { useState } from 'react'
import ControlEstopSection from './ControlPanel/ControlEstopSection'
import ControlPanelHeader from './ControlPanel/ControlPanelHeader'
import ControlSettings from './ControlPanel/ControlSettings'
import ControlStatusList from './ControlPanel/ControlStatusList'
import ControlVectorPlot, { type CmdVel } from './ControlPanel/ControlVectorPlot'
import './ControlPanel/ControlPanel.css'

const ControlPanel = () => {
  const [settingsOpen, setSettingsOpen] = useState(false)
  const [sensitivity, setSensitivity] = useState<'low' | 'med' | 'high'>('med')
  const cmdVel: CmdVel = { x: 0.44, y: 0.18, yaw: 0.12 }

  return (
    <aside className="control-panel">
      <ControlPanelHeader
        isSettingsOpen={settingsOpen}
        onToggleSettings={() => setSettingsOpen((open) => !open)}
      />
      <ControlStatusList />
      <ControlVectorPlot cmdVel={cmdVel} sensitivity={sensitivity} />
      <ControlEstopSection />
      {settingsOpen ? <ControlSettings sensitivity={sensitivity} onSelect={setSensitivity} /> : null}
    </aside>
  )
}

export default ControlPanel
