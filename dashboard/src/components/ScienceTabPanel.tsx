import CentrifugeCard from './CentrifugeCard'
import PanoramaCaptureCard from './PanoramaCaptureCard'
import {
  ScienceCarriageCard,
  ScienceDrillCard,
  SciencePumpLedCard,
} from './ScienceModuleControls'
import SpectrophotometerCard from './SpectrophotometerCard'

const ScienceTabPanel = () => {
  return (
    <div className="science-layout" role="tabpanel">
      <div className="science-column">
        <SciencePumpLedCard />
        <ScienceCarriageCard />
        <ScienceDrillCard />
      </div>
      <div className="science-column">
        <CentrifugeCard />
        <PanoramaCaptureCard />
        <SpectrophotometerCard />
      </div>
    </div>
  )
}

export default ScienceTabPanel
