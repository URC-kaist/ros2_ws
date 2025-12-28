# Dashboard Map Provider

## Overview

The MR2 Dashboard uses **MapLibre GL** as its mapping library with tiles provided by **Esri ArcGIS Online**.

## Map Library

- **MapLibre GL** (version 4.7.1+)
  - Open-source mapping library
  - Fork of Mapbox GL JS
  - Supports vector and raster tiles
  - GPU-accelerated rendering

## Tile Provider

The dashboard uses **Esri's ArcGIS Online World Imagery** service for satellite/aerial imagery:

- **Service URL**: `https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}`
- **Type**: Raster tiles
- **Tile Size**: 256x256 pixels
- **Attribution**: Sources: Esri, Maxar, Earthstar Geographics, and the GIS User Community

## Implementation

The map is implemented in the `MapPreview` component (`src/components/MapPreview.tsx`):

```typescript
import maplibregl from 'maplibre-gl'
import 'maplibre-gl/dist/maplibre-gl.css'

const map = new maplibregl.Map({
  container: mapRef.current,
  style: {
    version: 8,
    sources: {
      imagery: {
        type: 'raster',
        tiles: [
          'https://services.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
        ],
        tileSize: 256,
        attribution: 'Sources: Esri, Maxar, Earthstar Geographics, and the GIS User Community',
      },
    },
    layers: [
      {
        id: 'imagery',
        type: 'raster',
        source: 'imagery',
      },
    ],
  },
  center: [0, 0],
  zoom: 2,
})
```

## Features

- Navigation controls (zoom in/out, without compass)
- Satellite/aerial imagery
- Smooth pan and zoom
- GPU-accelerated rendering

## Changing the Map Provider

To use a different tile provider, modify the `style` configuration in `MapPreview.tsx`:

1. Update the `tiles` URL in the `sources` section
2. Update the `attribution` text to match the new provider
3. Adjust `tileSize` if needed (some providers use 512px tiles)

### Popular Alternatives

- **OpenStreetMap**: `https://tile.openstreetmap.org/{z}/{x}/{y}.png`
- **Stamen Terrain**: `https://stamen-tiles.a.ssl.fastly.net/terrain/{z}/{x}/{y}.jpg`
- **CARTO**: Various styles available at https://github.com/CartoDB/basemap-styles

Note: Always check the tile provider's terms of service and usage limits before switching providers.
