const CentrifugeCard = () => {
  return (
    <article className="card centrifuge-card">
      <header className="centrifuge-card__header">
        <div>
          <h3>Centrifuge</h3>
          <p>Rotor positioning, spin profiles, and run telemetry.</p>
        </div>
        <span className="pill">standby</span>
      </header>

      <div className="centrifuge-tabs" role="tablist" aria-label="Centrifuge interface tabs">
        <button
          className="centrifuge-tab centrifuge-tab--active"
          type="button"
          role="tab"
          aria-selected="true"
        >
          Controls
        </button>
        <button className="centrifuge-tab" type="button" role="tab" aria-selected="false">
          Get/Set
        </button>
        <button className="centrifuge-tab" type="button" role="tab" aria-selected="false">
          Stats
        </button>
      </div>

      <section className="centrifuge-section">
        <h4>Controls</h4>
        <div className="centrifuge-controls">
          <button className="centrifuge-button" type="button">
            Home
          </button>
          <button className="centrifuge-button" type="button">
            Start
          </button>
          <button className="centrifuge-button centrifuge-button--ghost" type="button">
            Stop
          </button>
          <button className="centrifuge-button centrifuge-button--ghost" type="button">
            Eject
          </button>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Get / Set</h4>
        <div className="centrifuge-form">
          <label className="centrifuge-field">
            <span>Position</span>
            <select defaultValue="3">
              <option value="1">1</option>
              <option value="2">2</option>
              <option value="3">3</option>
              <option value="4">4</option>
              <option value="5">5</option>
              <option value="6">6</option>
              <option value="7">7</option>
              <option value="8">8</option>
            </select>
          </label>
          <label className="centrifuge-field">
            <span>RPM</span>
            <input type="number" defaultValue={1200} min={0} step={50} />
          </label>
          <label className="centrifuge-field">
            <span>Duration (min)</span>
            <input type="number" defaultValue={10} min={1} step={1} />
          </label>
          <div className="centrifuge-actions">
            <button className="centrifuge-button centrifuge-button--ghost" type="button">
              Get
            </button>
            <button className="centrifuge-button" type="button">
              Set
            </button>
          </div>
        </div>
      </section>

      <section className="centrifuge-section">
        <h4>Statistics</h4>
        <div className="centrifuge-stats">
          <div className="centrifuge-stat">
            <span className="centrifuge-stat__label">Current Position</span>
            <span className="centrifuge-stat__value">3 / 8</span>
          </div>
          <div className="centrifuge-stat">
            <span className="centrifuge-stat__label">RPM</span>
            <span className="centrifuge-stat__value">1200</span>
          </div>
          <div className="centrifuge-progress">
            <div className="centrifuge-progress__meta">
              <span>Time</span>
              <span>3 min 50 sec / 10 min</span>
            </div>
            <progress
              className="centrifuge-progress__bar"
              value={230}
              max={600}
              aria-label="Run time progress"
            />
          </div>
        </div>
      </section>
    </article>
  )
}

export default CentrifugeCard
