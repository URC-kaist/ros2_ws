declare module '@transitive-sdk/utils-web' {
  import type { ComponentType } from 'react'

  export type TransitiveCapabilityProps = {
    jwt: string
    count?: string | number
    quantizer?: string | number
    rosversion?: string | number
    source?: string
    timeout?: string | number
    type?: string
  }

  export const TransitiveCapability: ComponentType<TransitiveCapabilityProps>
}
