# FastAPI Control UI

This directory contains the single-file browser control dashboard served by FastAPI.

Expected workflow:

- edit in VS Code
- keep changes versioned in GitHub
- run the FastAPI server separately while iterating on the UI

Main file:

- `index.html`

The page is served from:

- `http://127.0.0.1:8000`

## Purpose

The control UI is the operator-facing dashboard for:

- starting a scan
- saving the last scan as a baseline
- starting and stopping monitoring
- watching connection and robot state
- viewing live raw, residual, and baseline plots
- reviewing top debris candidates from residual clustering

## Plot Modes

The live plot section supports:

- `Raw`: current scan view
- `Residual`: filtered foreground current rays that are closer than the baseline
- `Baseline`: hold-to-view baseline snapshot

Residual mode uses the monitoring tolerance field.

Important:

- the tolerance does not change the saved baseline
- it only changes which rays qualify as foreground residuals

Residual mode is foreground-only:

- the point stays at the current scan position
- it appears only when the current ray is closer than the baseline by more than the tolerance
- this means the residual plot shows the shape of the current blocking object

## Debris Clusters In The UI

When residuals are present, the control page also shows the top debris candidates.

Each debris candidate is produced by the control server after:

- residual foreground gating
- spatial clustering
- debris validation heuristics

The control page displays:

- top 3 ranked debris candidates
- point count
- max occlusion depth
- cluster size in millimeters

## Debris Score

The score shown in the control page is a ranking heuristic, not a probability.

It is computed on the control-server side from:

- maximum occlusion
- mean occlusion
- point count
- angular occupancy ratio
- compactness of the cluster bounding box

The exact formula is documented in:

- [README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/README.md)
- [control_server/services/README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/control_server/services/README.md)

## Main Controls

Important buttons currently wired in the UI:

- `Start Scan`
- `Save Last Scan as Baseline`
- `Start Monitoring`
- `Stop Monitoring`
- `Raw`
- `Residual`
- `Baseline`

## Data Source

The page polls:

- `/api/status`
- `/api/live-points`

The plots are plain canvas 2D views:

- top view: `X/Y`
- side view: `X/Z`

## Status Areas

The page shows:

- robot connection state
- robot scan state
- sensor timeout banner
- current capture and baseline paths
- residual capture summary
- debris ranking cards
- monitoring schedule and last event text

## Editing Notes

This file intentionally keeps the control app self-contained:

- styles
- markup
- API polling
- plot rendering

If you add new server-side control routes, update this file so the new command has:

- a button
- a request path
- disabled-state logic
- visible status feedback
