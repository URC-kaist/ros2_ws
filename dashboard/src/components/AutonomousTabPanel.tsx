import AutonomyHealthCard from './AutonomyHealthCard'
import MapPreview from './MapPreview'
import MissionMasterPanel from './MissionMasterPanel'
import RecentObjectsCard from './RecentObjectsCard'
import type { MissionSpec } from '../lib/missions'

type AutonomousTabPanelProps = {
  missionList: MissionSpec[]
  previewMissions: MissionSpec[]
  grabFromMap: boolean
  onGrabCoordinate: (coord: { lat: number; lon: number }) => void
  onGrabFromMapChange: (enabled: boolean) => void
  onMissionListChange: (missions: MissionSpec[]) => void
  onMissionPreview: (missions: MissionSpec[]) => void
}

const AutonomousTabPanel = ({
  missionList,
  previewMissions,
  grabFromMap,
  onGrabCoordinate,
  onGrabFromMapChange,
  onMissionListChange,
  onMissionPreview,
}: AutonomousTabPanelProps) => {
  const missionsForMap = grabFromMap ? missionList : previewMissions

  return (
    <div className="autonomy-layout" role="tabpanel">
      <MissionMasterPanel
        missionList={missionList}
        grabFromMap={grabFromMap}
        healthSlot={<AutonomyHealthCard />}
        mapSlot={
          <MapPreview
            missionList={missionsForMap}
            grabFromMap={grabFromMap}
            onGrabCoordinate={onGrabCoordinate}
          />
        }
        onGrabFromMapChange={onGrabFromMapChange}
        onMissionListChange={onMissionListChange}
        onMissionPreview={onMissionPreview}
      />
      <RecentObjectsCard />
    </div>
  )
}

export default AutonomousTabPanel
