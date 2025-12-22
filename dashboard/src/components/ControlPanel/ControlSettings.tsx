import './ControlSettings.css'

type ControlSettingsProps = {
  sensitivity: 'low' | 'med' | 'high'
  onSelect: (level: 'low' | 'med' | 'high') => void
}

const ControlSettings = ({ sensitivity, onSelect }: ControlSettingsProps) => {
  return (
    <section className="settings-panel" role="dialog" aria-label="Control settings">
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
    </section>
  )
}

export default ControlSettings
