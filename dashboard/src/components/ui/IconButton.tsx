import type { ReactNode } from 'react'
import './IconButton.css'

type IconButtonProps = {
  ariaLabel: string
  onClick: () => void
  pressed?: boolean
  children: ReactNode
  className?: string
}

const IconButton = ({ ariaLabel, onClick, pressed, children, className }: IconButtonProps) => {
  return (
    <button
      className={['icon-button', className].filter(Boolean).join(' ')}
      type="button"
      aria-label={ariaLabel}
      aria-pressed={pressed}
      onClick={onClick}
    >
      {children}
    </button>
  )
}

export default IconButton
