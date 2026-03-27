# FastAPI Control UI

This directory contains the single-file browser control dashboard served by FastAPI.

Main file:

- `index.html`

The page is served from:

- `http://127.0.0.1:8000`

## Purpose

The control UI is the operator-facing dashboard for:

- starting a scan
- saving a baseline
- starting and stopping monitoring
- watching connection and robot state
- viewing live raw, residual, and baseline plots

## Plot Modes

The live plot section supports:

- `Raw`: current scan view
- `Residual`: filtered foreground current rays that are closer than the baseline
- `Baseline`: hold-to-view baseline snapshot

Residual mode uses the monitoring tolerance field.

Important:

- the tolerance does not change the saved baseline
- it only changes which rays qualify as foreground residuals

## Main Controls

Important buttons currently wired in the UI:

- `Start Scan`
- `Save Baseline`
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
