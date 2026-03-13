# MR2 Dashboard

The dashboard is a Vite + React + TypeScript frontend for the MR2 base station.
It combines three runtime integrations:

1. `rosbridge` for ROS topics and services
2. the MR2 base gateway for SiK telemetry, link state, and mission control
3. Transitive token minting for browser video access

## Development

```bash
cd dashboard
npm install
npm run dev
```

The Vite dev server listens on all interfaces (`0.0.0.0`).

## Scripts

- `npm run dev` starts the Vite dev server
- `npm run build` builds the production bundle
- `npm run preview` serves the built bundle locally
- `npm run lint` runs ESLint
- `npm run typecheck` runs the TypeScript checker
- `npm run check` runs typecheck, lint, and build in sequence

## Environment

Copy `.env.example` to `.env.local` when you need to override endpoints.

Available variables:

- `VITE_ROSBRIDGE_URL`
  Default: `ws(s)://<current-host>/rosbridge-ws`
- `VITE_SIK_WS_URL`
  Default: `ws(s)://<current-host>/sik-ws`
- `VITE_TRANSITIVE_TOKEN_URL`
  Default: `/transitive/token`

If these variables are unset, the dashboard assumes it is being served behind
the same host that proxies the gateway and rosbridge endpoints.

## Runtime Dependencies

### ROS bridge

The dashboard uses `roslib` and expects `window.ROSLIB` to be provided by
`public/roslib.min.js`.

If you update `roslib`, replace `public/roslib.min.js` and rebuild the
dashboard.

### Gateway

The gateway WebSocket client lives in `src/lib/sikGateway.ts`.
It carries:

- rover telemetry (`telem_nav`, `telem_battery`)
- gateway link status
- base antenna status
- Rocket M2 status
- outbound commands such as drive, gripper, mission control, and base heading

### ROS topics and services

The ROS bridge client lives in `src/lib/rosBridge.ts`.
It is used for:

- system diagnostics
- battery telemetry topics
- mission list and mission status
- map-coordinate conversion via `/toLL`
- science and autonomy visualization topics

## Package Layout

```text
dashboard/
├── public/                 # static vendor assets such as roslib and uPlot
├── src/
│   ├── components/         # React UI modules
│   ├── lib/                # ROS and gateway transport clients
│   ├── types/              # ambient type declarations
│   ├── App.tsx             # top-level shell
│   └── main.tsx            # React entrypoint
├── vite.config.js
└── eslint.config.js
```

## Current Refactor Priorities

- replace component-local connection bootstrapping with shared React hooks or providers
- move shared domain types out of leaf components
- split large mixed-responsibility components such as `MapPreview`,
  `MissionMasterPanel`, and `SystemStatusPanel`
- document and standardize local verification so dashboard changes are easier to review
