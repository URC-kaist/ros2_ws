import type { ReactNode } from 'react'
import './Modal.css'

type ModalProps = {
  ariaLabel: string
  onClose: () => void
  children: ReactNode
  panelClassName?: string
}

const Modal = ({ ariaLabel, onClose, children, panelClassName }: ModalProps) => {
  return (
    <div className="modal-backdrop" role="presentation" onClick={onClose}>
      <section
        className={['modal-panel', panelClassName].filter(Boolean).join(' ')}
        role="dialog"
        aria-label={ariaLabel}
        onClick={(event) => event.stopPropagation()}
      >
        {children}
      </section>
    </div>
  )
}

export default Modal
