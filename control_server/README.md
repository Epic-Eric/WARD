# Control Server

This directory contains the Python control server for WARD.

## Role

The control server does four jobs:

1. accepts the robot TCP stream on port `9000`
2. records raw scan captures as CSV files
3. manages baseline saving, residual generation, and monitoring loops
4. serves the FastAPI control page and JSON APIs on port `8000`

It is also the component that computes:

- baseline-vs-current residuals
- debris clustering
- debris scores

## Entry Point

- `point_cloud_server.py`

This file is intentionally thin. It wires together:

- `RobotTcpHub`
- `ScanAutomationController`
- the FastAPI app returned by `services.api.create_app(...)`

## How To Run

From the repo root:

```bash
python3 control_server/point_cloud_server.py
```

Then open:

- `http://127.0.0.1:8000`

## Runtime Dependencies

- `fastapi`
- `uvicorn`

Install with:

```bash
python3 -m pip install fastapi uvicorn
```

## Data Flow

1. The robot connects over TCP and sends telemetry lines.
2. `RobotTcpHub` parses those lines, updates live state, and records raw points through `CaptureRecorder`.
3. `ScanAutomationController` uses completed captures to:
   - save a baseline
   - compute live residual payloads
   - run monitoring
   - write residual CSVs
4. `services.residuals` turns residual foreground points into debris candidates and scores.
5. `services.api` exposes that state to the FastAPI page and the React viewer.

## Files And Output

- `captures/point_cloud_*.csv`
  Raw scan captures.
- `captures/residual_*.csv`
  Residual foreground captures.
- `captures/baseline_scan.csv`
  Current baseline scan.
- `captures/baseline_scan_meta.json`
  Baseline metadata such as source capture and scan degrees.

## Debris Score

Debris scoring is implemented in `services/residuals.py`.

The score is a weighted heuristic that combines:

- maximum occlusion depth
- mean occlusion depth
- point count
- angular occupancy ratio
- compactness based on bounding-box diagonal

The exact formula is documented in the root [README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/README.md) and in [services/README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/control_server/services/README.md).

## Component Boundary

The control server is the source of truth for:

- baseline persistence
- residual generation
- debris clustering
- debris score calculation

The firmware only supplies raw scan data, and the UIs only render/control that server state.

## Service Docs

For module-by-module ownership, see:

- [services/README.md](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/control_server/services/README.md)
