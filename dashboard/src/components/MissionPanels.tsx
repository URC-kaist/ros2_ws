import { useState } from 'react'
import AutonomousTabPanel from './AutonomousTabPanel'
import DeliveryPanel from './DeliveryPanel'
import LiveFeedTabPanel from './LiveFeedTabPanel'
import ScienceTabPanel from './ScienceTabPanel'
import type { MissionTabId } from './MissionTabs'
import SystemStatusPanel from './SystemStatusPanel'
import { createMissionSpec, type MissionSpec } from '../lib/missions'
import './MissionPanels.css'

type MissionPanelsProps = {
  activeTab: MissionTabId
}

const MissionPanels = ({ activeTab }: MissionPanelsProps) => {
  const [missionList, setMissionList] = useState<MissionSpec[]>([])
  const [previewMissions, setPreviewMissions] = useState<MissionSpec[]>([])
  const [grabFromMap, setGrabFromMap] = useState(false)

  const handleGrabFromMap = ({ lat, lon }: { lat: number; lon: number }) => {
    setMissionList((prev) => {
      const nextId =
        prev.reduce(
          (max, mission) =>
            Number.isFinite(mission.mission_id) ? Math.max(max, mission.mission_id) : max,
          0
        ) + 1
      return [
        ...prev,
        createMissionSpec(nextId, {
          target_latitude: lat,
          target_longitude: lon,
        }),
      ]
    })
  }

  return (
    <section className="tab-panels">
      {activeTab === 'status' && <SystemStatusPanel />}
      {activeTab === 'live-feed' && <LiveFeedTabPanel />}
      {activeTab === 'delivery' && <DeliveryPanel />}
      {activeTab === 'science' && <ScienceTabPanel />}
      {activeTab === 'autonomous' && (
        <AutonomousTabPanel
          missionList={missionList}
          previewMissions={previewMissions}
          grabFromMap={grabFromMap}
          onGrabCoordinate={handleGrabFromMap}
          onGrabFromMapChange={setGrabFromMap}
          onMissionListChange={setMissionList}
          onMissionPreview={setPreviewMissions}
        />
      )}
    </section>
  )
}

export default MissionPanels
