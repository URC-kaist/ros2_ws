import './ControlPanelHeader.css'

type ControlPanelHeaderProps = {
  isSettingsOpen: boolean
  onToggleSettings: () => void
}

const ControlPanelHeader = ({ isSettingsOpen, onToggleSettings }: ControlPanelHeaderProps) => {
  return (
    <div className="panel-header">
      <div className="badge">MR2</div>
      <p className="panel-title">Control</p>
      <button
        className="settings-button"
        type="button"
        aria-label="Open settings"
        aria-pressed={isSettingsOpen}
        onClick={onToggleSettings}
      >
        <svg viewBox="0 0 24 24" aria-hidden="true">
          <path d="M12 8.6a3.4 3.4 0 1 0 0 6.8 3.4 3.4 0 0 0 0-6.8Zm9.2 3.4c0-.5 0-1-.1-1.5l2-1.6-2-3.4-2.4 1a9.2 9.2 0 0 0-2.6-1.5l-.3-2.6h-4l-.3 2.6a9.2 9.2 0 0 0-2.6 1.5l-2.4-1-2 3.4 2 1.6c-.1.5-.1 1-.1 1.5s0 1 .1 1.5l-2 1.6 2 3.4 2.4-1a9.2 9.2 0 0 0 2.6 1.5l.3 2.6h4l.3-2.6a9.2 9.2 0 0 0 2.6-1.5l2.4 1 2-3.4-2-1.6c.1-.5.1-1 .1-1.5Z" />
        </svg>
      </button>
    </div>
  )
}

export default ControlPanelHeader
