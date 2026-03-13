import AutonomyHealthCard from './AutonomyHealthCard'
import MapPreview from './MapPreview'
import MissionMasterPanel from './MissionMasterPanel'
import TransitiveVideoCard from './TransitiveVideoCard'
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
        <article className="card video-feed-card">
          <h3>Vision</h3>
          <div className="video-feed-grid">
            <div className="video-feed-item">
              <span className="video-feed-label">ArUco</span>
              <TransitiveVideoCard
                embedded
                source="/aruco_tracker/debug"
                videoWidth={320}
                videoHeight={180}
              />
            </div>
            <div className="video-feed-item">
              <span className="video-feed-label">YOLO</span>
              <TransitiveVideoCard
                embedded
                source="/yolo/annotated_image"
                videoWidth={320}
                videoHeight={180}
              />
            </div>
          </div>
        </article>
      </div>
      <div className="autonomy-right">
        <AutonomyHealthCard />
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
