import { useSystemStatusCards } from '../hooks/useSystemStatusCards'
import CameraTurretCard from './CameraTurretCard'
import GnssStatusCard from './GnssStatusCard'
import RocketM2Card from './RocketM2Card'
import SystemStatusCards from './SystemStatusCards'
import './SystemStatusPanel.css'

const SystemStatusPanel = () => {
  const {
    cards,
    baseStatus,
    baseHeadingInput,
    setBaseHeadingInput,
    applyBaseHeading,
  } = useSystemStatusCards()

  return (
    <div className="panel-grid" role="tabpanel">
      <GnssStatusCard />
      <RocketM2Card />
      <CameraTurretCard />
      <SystemStatusCards
        cards={cards}
        baseStatus={baseStatus}
        baseHeadingInput={baseHeadingInput}
        onBaseHeadingInputChange={setBaseHeadingInput}
        onApplyBaseHeading={applyBaseHeading}
      />
    </div>
  )
}

export default SystemStatusPanel
