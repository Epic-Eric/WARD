# React 3D Viewer

This directory contains the separate React + Three.js live viewer.

Main files:

- `src/App.jsx`
- `src/styles.css`
- `vite.config.js`

## Purpose

The React app gives a 3D view of:

- raw scan geometry
- the saved baseline
- residual foreground geometry

It is intended for visual inspection rather than command/control.

## Run

Install dependencies:

```bash
npm install
```

Start the dev server:

```bash
npm run dev
```

Build:

```bash
npm run build
```

## Data Source

The viewer fetches:

- `/api/status`
- `/api/live-points`

It expects the FastAPI server to already be running on `127.0.0.1:8000`.

## View Modes

Current modes:

- `Raw`
- `Residual`
- `Baseline`

Residual mode currently means:

- use the current scan geometry
- only keep rays whose current range is smaller than the baseline by more than the monitoring tolerance
- color those foreground rays by occlusion depth

## Scene Conventions

The viewer is configured as:

- `Z-up`
- grid on the `XY` plane
- point and mesh positions taken directly from server `x,y,z` values

## Rendering Notes

The app renders both:

- point cloud
- triangle mesh

Special handling:

- `is_air` samples are not rendered as visible points
- mesh triangles are skipped when required vertices are missing or marked as air
- residual colors are driven by occlusion depth, not by the absolute current range

## When To Edit This Component

Edit this app when you need:

- 3D camera behavior changes
- mesh rules
- point styling
- alternate residual or baseline presentation
- richer 3D inspection tools than the control UI provides
