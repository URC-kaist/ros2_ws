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
      <div className="autonomy-left">
        <article className="card card--map">
          <MapPreview
            missionList={missionsForMap}
            grabFromMap={grabFromMap}
            onGrabCoordinate={onGrabCoordinate}
          />
        </article>
      </div>
      <div className="autonomy-right">
        <AutonomyHealthCard />
        <RecentObjectsCard />
        <MissionMasterPanel
          missionList={missionList}
          grabFromMap={grabFromMap}
          onGrabFromMapChange={onGrabFromMapChange}
          onMissionListChange={onMissionListChange}
          onMissionPreview={onMissionPreview}
        />
      </div>
    </div>
  )
}

export default AutonomousTabPanel
