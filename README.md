# WARD Point Cloud Scanner

WARD is a Nano RP2040 Connect based point-cloud scanner with:

- Arduino firmware in `src/main.cpp`
- a FastAPI control and capture server in `point_cloud_server.py`
- a browser control UI in `static/index.html`
- a React + Three.js viewer in `viewer-react/`
- a simple offline plotting utility in `plot_lidar.py`

## Repo Layout

- `src/main.cpp`: robot firmware, stepper control, sensor reads, TCP protocol
- `point_cloud_server.py`: robot TCP listener, capture writer, baseline/residual logic, FastAPI API
- `static/index.html`: control dashboard served by FastAPI
- `viewer-react/`: separate 3D viewer app for raw, baseline, and residual scans
- `captures/`: generated scan CSVs, baseline CSV, residual CSVs
- `plot_lidar.py`: local CSV plotting helper

## System Architecture

1. The Nano connects to Wi-Fi and opens a TCP client to the computer on port `9000`.
2. The Python server accepts robot telemetry and writes point-cloud captures to CSV.
3. The FastAPI app serves status and live plot data on port `8000`.
4. The control UI and the React viewer both consume `/api/status` and `/api/live-points`.

## FastAPI Server

Entry point:

- `python3 point_cloud_server.py`

Default ports:

- HTTP UI: `8000`
- robot TCP listener: `9000`

Main responsibilities:

- accept robot connections and parse the line protocol
- store captures under `captures/`
- save and delete a baseline scan
- run monitoring scans on an interval
- compute live residual views and residual capture CSVs
- expose the control UI and JSON APIs

Primary API routes:

- `GET /`
- `GET /api/status`
- `GET /api/live-points`
- `POST /api/robot/start-scan`
- `POST /api/robot/stop-scan`
- `POST /api/robot/hard-stop`
- `POST /api/robot/release-motors`
- `POST /api/baseline/save`
- `DELETE /api/baseline`
- `POST /api/monitoring/start`
- `POST /api/monitoring/stop`

## Current Residual Semantics

Residual mode is not a signed difference field anymore.

It now means:

- start from the current scan
- compare each current ray against the baseline at the same `yaw,pitch`
- keep only rays where `current_distance < baseline_distance - monitoring_tolerance`
- render those foreground current rays as the residual plot

This keeps the residual view shaped like the current occluding object instead of the baseline.

## Quick Start

### Firmware

From the repo root:

```bash
/Users/haysoncheung/.platformio/penv/bin/pio run -e nanorp2040connect -t upload
```

### Control Server

```bash
python3 point_cloud_server.py
```

Open:

- `http://127.0.0.1:8000`

### React Viewer

```bash
cd viewer-react
npm install
npm run dev
```

Open:

- `http://localhost:5173`

## Offline Plot Utility

To plot a saved CSV:

```bash
python3 plot_lidar.py --file captures/your_capture.csv
```

Notes:

- file mode does not require `pyserial`
- live serial mode does require `pyserial`

## Additional Component Docs

- [Firmware README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/src/README.md)
- [Control UI README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/static/README.md)
- [React Viewer README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/viewer-react/README.md)
