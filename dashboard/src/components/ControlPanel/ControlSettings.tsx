import Modal from '../ui/Modal'
import './ControlSettings.css'

type ControlSettingsProps = {
  sensitivity: 'low' | 'med' | 'high'
  onSelect: (level: 'low' | 'med' | 'high') => void
  onClose: () => void
}

const ControlSettings = ({ sensitivity, onSelect, onClose }: ControlSettingsProps) => {
  return (
    <Modal ariaLabel="Control settings" onClose={onClose} panelClassName="settings-panel">
      <div className="settings-title">Sensitivity</div>
      <div className="settings-options" role="group" aria-label="Sensitivity presets">
        {(['low', 'med', 'high'] as const).map((level) => (
          <button
            key={level}
            type="button"
            className={`settings-chip ${sensitivity === level ? 'active' : ''}`}
            onClick={() => onSelect(level)}
          >
            {level.toUpperCase()}
          </button>
        ))}
      </div>
    </Modal>
  )
}

export default ControlSettings
