import { isRoverDirectProfile } from '../lib/operatingProfile'
import './MissionTabs.css'

const TABS = [
  { id: 'status', label: 'Status' },
  { id: 'live-feed', label: 'Live Feed' },
  { id: 'science', label: 'Science' },
  { id: 'delivery', label: 'Delivery / Servicing' },
  { id: 'autonomous', label: 'Autonomous' },
] as const

type MissionTabId = (typeof TABS)[number]['id']

const visibleTabs = isRoverDirectProfile
  ? TABS.filter((tab) => ['status', 'live-feed', 'delivery'].includes(tab.id)).map((tab) =>
      tab.id === 'delivery' ? { ...tab, label: 'Arm' } : tab
    )
  : TABS

type MissionTabsProps = {
  activeTab: MissionTabId
  onTabChange: (tab: MissionTabId) => void
}

const MissionTabs = ({ activeTab, onTabChange }: MissionTabsProps) => {
  return (
    <nav className="tabs" role="tablist">
      {visibleTabs.map((tab) => (
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
