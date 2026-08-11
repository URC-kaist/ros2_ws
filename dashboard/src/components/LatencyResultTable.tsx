import type { AutomatedUplinkSegments } from '../lib/rtpLatencyReport'

type LatencyResultTableProps = {
  segments: AutomatedUplinkSegments
}

const rows: Array<{ key: keyof AutomatedUplinkSegments; label: string }> = [
  { key: 'rocket_m2', label: 'Rover RTP -> Base RTP' },
  { key: 'base_to_browser', label: 'Base RTP -> Browser' },
  { key: 'decode_render', label: 'Browser -> Render' },
  { key: 'total', label: 'Total' },
]

function formatUs(value: number | null) {
  return value == null || !Number.isFinite(value) ? '--' : `${(value / 1000).toFixed(2)}`
}

const LatencyResultTable = ({ segments }: LatencyResultTableProps) => (
  <div className="latency-result-table__scroll">
    <table className="latency-result-table">
      <thead>
        <tr>
          <th>Segment</th>
          <th>N</th>
          <th>p50</th>
          <th>p95</th>
          <th>p99</th>
          <th>max</th>
        </tr>
      </thead>
      <tbody>
        {rows.map(({ key, label }) => {
          const distribution = segments[key]
          return (
            <tr key={key}>
              <th>{label}</th>
              <td>{distribution.count}</td>
              <td>{formatUs(distribution.p50)}</td>
              <td>{formatUs(distribution.p95)}</td>
              <td>{formatUs(distribution.p99)}</td>
              <td>{formatUs(distribution.max)}</td>
            </tr>
          )
        })}
      </tbody>
    </table>
  </div>
)

export default LatencyResultTable
