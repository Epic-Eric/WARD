# React 3D Viewer

This directory contains the separate React + Three.js live viewer.

Expected workflow:

- edit in VS Code
- commit changes through GitHub
- run the Vite dev server locally while the FastAPI backend is running

Main files:

- `src/App.jsx`
- `src/styles.css`
- `vite.config.js`

## Purpose

The React app gives a 3D view of:

- raw scan geometry
- the saved baseline
- residual foreground geometry
- debris clusters, boxes, and rankings

It is primarily for visual inspection, but it also mirrors core robot controls through the bottom command bar.

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

## Debris Visualization

When residual clusters are available, the viewer shows:

- 3D cluster bounding boxes
- ranked debris cards in the side panel
- on-canvas debris labels
- grouped count-only labels when many labels overlap on screen

The viewer also supports:

- selecting a debris candidate from the leaderboard
- highlighting the selected cluster
- showing debris metadata in the point hover card when the hovered point belongs to a cluster

## Debris Score

The score shown in the viewer comes from the control server.

It is based on:

- maximum occlusion depth
- mean occlusion depth
- number of supporting points
- angular occupancy ratio
- compactness of the cluster

The viewer does **not** compute the score itself. It only renders the score sent by the server.

For the exact formula, see:

- [README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/README.md)
- [control_server/services/README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/control_server/services/README.md)

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
- grouped debris labels suppress overlapping detailed labels when the scene is crowded

## When To Edit This Component

Edit this app when you need:

- 3D camera behavior changes
- mesh rules
- point styling
- alternate residual or baseline presentation
- richer 3D inspection tools than the control UI provides
- debris label, grouping, or cluster-inspection changes
