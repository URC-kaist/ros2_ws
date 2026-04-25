# MR2 Dashboard

The dashboard is the browser frontend for MR2 base-station operations. It is a
Vite + React + TypeScript application that combines:

- `rosbridge` for ROS topics and services
- the MR2 base gateway for XBEE telemetry, mission control, antenna status,
  Rocket M2 status, and browser video delivery

## Runtime Model

The dashboard talks to two external systems:

1. `rosbridge`
   Used for diagnostics, mission topics, map-coordinate conversion via `/toLL`,
   science capture services, and autonomy visualization topics.
2. base gateway
   Used for rover telemetry, battery/link state, mission control, drive/arm
   commands, base antenna heading, Rocket M2 state, and browser video streams.

If no explicit environment overrides are provided, the dashboard assumes it is
served behind the same host that proxies:

- `/rosbridge-ws`
- `/xbee-ws`
- `/video-ws`
- `/video/streams`

## Quick Start

```bash
cd dashboard
npm install
cp .env.local.example .env.local
npm run dev
```

The Vite dev server binds to `0.0.0.0`. In this repo it is configured to allow
access from `mr2-ubuntu` as well as the default local hostnames.

## Local Setup

For a simple same-host setup:

1. Leave `VITE_ROSBRIDGE_URL` unset if the browser can reach
   `ws://<host>/rosbridge-ws`.
2. Leave `VITE_XBEE_WS_URL` unset if the browser can reach
   `ws://<host>/xbee-ws`.
3. Leave `VITE_VIDEO_WS_URL` unset if the browser can reach
   `ws://<host>/video-ws`.
4. Leave `VITE_VIDEO_STREAMS_URL` unset if the browser can reach
   `http(s)://<host>/video/streams`.

For a split-host setup:

1. Set `VITE_ROSBRIDGE_URL` to the real rosbridge WebSocket URL.
2. Set `VITE_XBEE_WS_URL` to the real gateway WebSocket URL.
3. Set `VITE_VIDEO_WS_URL` to the real video WebSocket URL.
4. Set `VITE_VIDEO_STREAMS_URL` to the real video metadata endpoint URL.
5. Restart `npm run dev` after changing env files.

## Environment Files

Use [`dashboard/.env.local.example`](/home/gmmyung/mr2-stack/dashboard/.env.local.example)
as the copy source for local overrides:

```bash
cp .env.local.example .env.local
```

Vite loads `.env.local` automatically during development and build. The
template includes every environment variable currently read by the dashboard.

### Connection Endpoints

| Variable | Purpose | Default if unset |
| --- | --- | --- |
| `VITE_ROSBRIDGE_URL` | Full rosbridge WebSocket URL | `ws(s)://<current-host>/rosbridge-ws` |
| `VITE_XBEE_WS_URL` | Full base gateway WebSocket URL | `ws(s)://<current-host>/xbee-ws` |
| `VITE_VIDEO_WS_URL` | Full video WebSocket URL | `ws(s)://<current-host>/video-ws` |
| `VITE_VIDEO_STREAMS_URL` | Video metadata endpoint | `http(s)://<current-origin>/video/streams` |

### Science Export Metadata

| Variable | Purpose | Default if unset |
| --- | --- | --- |
| `VITE_SPECTRO_CALIBRATION_PATH` | Written into spectrometer CSV exports as metadata | `unknown` unless supplied by the service response |

## Development Commands

- `npm run dev` starts the Vite dev server
- `npm run build` builds the production bundle
- `npm run preview` serves the built bundle locally
- `npm run lint` runs ESLint
- `npm run typecheck` runs the TypeScript checker
- `npm run check` runs `typecheck`, `lint`, and `build`

## Package Structure

```text
dashboard/
├── public/                 # vendored browser assets such as roslib and uPlot
├── src/
│   ├── components/         # UI composition and cards/panels
│   ├── hooks/              # connection and feature-specific runtime hooks
│   ├── lib/                # shared transport/domain helpers and types
│   ├── types/              # ambient browser/global declarations
│   ├── App.tsx             # top-level shell
│   └── main.tsx            # React entrypoint
├── .env.local.example      # local override template
├── eslint.config.js
├── package.json
└── vite.config.js
```

## External Browser Dependencies

### `ROSLIB`

The dashboard expects `window.ROSLIB` to be available from
[`dashboard/public/roslib.min.js`](/home/gmmyung/mr2-stack/dashboard/public/roslib.min.js).
The runtime wrapper is in
[`dashboard/src/lib/rosBridge.ts`](/home/gmmyung/mr2-stack/dashboard/src/lib/rosBridge.ts).

### `uPlot`

The spectrophotometer card expects a global `uPlot` constructor from the
vendored browser asset under `public/vendor/uplot/`.

### Gateway Transport

The base gateway WebSocket client is implemented in
[`dashboard/src/lib/xbeeGateway.ts`](/home/gmmyung/mr2-stack/dashboard/src/lib/xbeeGateway.ts).
It owns:

- rover telemetry (`telem_nav`, `telem_battery`)
- link status
- base antenna/base-station status
- Rocket M2 status
- outbound control messages such as drive, arm, mission control, heartbeat, and
  base heading

The video gateway client is implemented in
[`dashboard/src/lib/videoGateway.ts`](/home/gmmyung/mr2-stack/dashboard/src/lib/videoGateway.ts).
It owns:

- stream discovery via `/video/streams`
- per-stream subscriptions over `/video-ws`
- browser-side H.264 chunk delivery to the WebCodecs renderer

## Current Refactor Status

The dashboard has already been restructured around:

- shared transport hooks
- shared mission and ROS payload models
- split map-preview hooks for data vs map rendering
- split system-status data/presentation layers
- extracted tab-specific panels for science and autonomy

The remaining follow-up item is lightweight smoke verification, tracked in
issue `#74`.
