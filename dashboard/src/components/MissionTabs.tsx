import './MissionTabs.css'

const TABS = [
  { id: 'status', label: 'Status' },
  { id: 'delivery', label: 'Delivery' },
  { id: 'science', label: 'Science' },
  { id: 'autonomous', label: 'Autonomous' },
  { id: 'manipulation', label: 'Manipulation' },
  { id: 'navigation', label: 'Navigation' },
  { id: 'comms', label: 'Comms' },
]

type MissionTabId = (typeof TABS)[number]['id']

type MissionTabsProps = {
  activeTab: MissionTabId
  onTabChange: (tab: MissionTabId) => void
}

const MissionTabs = ({ activeTab, onTabChange }: MissionTabsProps) => {
  return (
    <nav className="tabs" role="tablist">
      {TABS.map((tab) => (
        <button
          key={tab.id}
          className={`tab ${activeTab === tab.id ? 'active' : ''}`}
          role="tab"
          type="button"
          onClick={() => onTabChange(tab.id)}
        >
          {tab.label}
        </button>
      ))}
    </nav>
  )
}

export type { MissionTabId }
export default MissionTabs
