import type { BaseStatus } from '../lib/sikGateway'
import {
  formatKey,
  formatValue,
  type StatusCard,
} from '../lib/systemStatus'

type SystemStatusCardsProps = {
  cards: StatusCard[]
  baseStatus: BaseStatus | null
  baseHeadingInput: string
  onBaseHeadingInputChange: (value: string) => void
  onApplyBaseHeading: () => void
}

const SystemStatusCards = ({
  cards,
  baseStatus,
  baseHeadingInput,
  onBaseHeadingInputChange,
  onApplyBaseHeading,
}: SystemStatusCardsProps) => {
  return (
    <>
      {cards.map(({ spec, snapshot, values }) => (
        <article className="card" key={spec.id}>
          <h3>{spec.label}</h3>
          {spec.id === 'base_station' ? (
            <div className="base-heading-row">
              <div className="base-heading-header">
                <label htmlFor="base-heading-status" className="base-heading-label">
                  Heading offset
                </label>
                <span className="status-kv-value">
                  {baseStatus && Number.isFinite(baseStatus.heading_offset_deg)
                    ? `${baseStatus.heading_offset_deg.toFixed(1)}°`
                    : '—'}
                </span>
              </div>
              <div className="base-heading-input-row">
                <input
                  id="base-heading-status"
                  type="number"
                  inputMode="decimal"
                  value={baseHeadingInput}
                  onChange={(event) => onBaseHeadingInputChange(event.target.value)}
                  placeholder="deg"
                  className="base-heading-input"
                />
                <button
                  type="button"
                  onClick={onApplyBaseHeading}
                  className="base-heading-apply"
                >
                  Apply
                </button>
              </div>
            </div>
          ) : null}
          {values.length === 0 ? (
            <div className="status-empty">No data yet.</div>
          ) : (
            <div className="status-kv-grid">
              {values.map((item) => (
                <div className="status-kv-row" key={item.key}>
                  <span className="status-kv-key">{formatKey(spec.id, item.key)}</span>
                  <strong className="status-kv-value">
                    {formatValue(spec.id, item.key, item.value)}
                  </strong>
                </div>
              ))}
            </div>
          )}
          {snapshot ? (
            <div className="status-updated">
              Updated {new Date(snapshot.updatedAt).toLocaleTimeString()}
            </div>
          ) : null}
        </article>
      ))}
    </>
  )
}

export default SystemStatusCards
