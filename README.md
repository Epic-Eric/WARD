# WARD Debris Removal Robot

![Course](https://img.shields.io/badge/Course-ESC204%20Praxis%203-002a5c?style=for-the-badge)
![School](https://img.shields.io/badge/University-University%20of%20Toronto-1d4f91?style=for-the-badge)
![Board](https://img.shields.io/badge/Board-Arduino%20Nano%20RP2040%20Connect-0b7a75?style=for-the-badge)
![Stack](https://img.shields.io/badge/Stack-PlatformIO%20%7C%20FastAPI%20%7C%20React-b9781f?style=for-the-badge)

WARD is a debris-removal robot project for **ESC204 Praxis 3** at the **University of Toronto**. The robot performs turret-based distance scans, streams point-cloud telemetry to a computer, saves a baseline map of a region, and highlights new debris through residual analysis and clustering.

## Team

- Hayson Cheung
- Eric Xie
- Nick Fry
- Michael Strojny
- Jessica Yi
- Malia Lubach

## What The System Does

- scans the environment with a yaw/pitch turret
- streams point-cloud telemetry from the robot to the computer over Wi-Fi
- records scans as CSV captures
- saves a baseline scan of the environment
- computes foreground residuals relative to the baseline
- clusters residual foreground points into debris candidates
- visualizes scans in both a FastAPI control page and a React + Three.js viewer

## System Overview

1. The Arduino Nano RP2040 Connect joins Wi-Fi and connects to the computer over TCP.
2. The control server listens for robot telemetry on port `9000`.
3. The control server records captures, serves the operator UI on port `8000`, and computes baseline/residual/debris results.
4. The React viewer consumes the server APIs for live 3D visualization.

## Repository Layout

- `firmware/src/main.cpp`
  Robot firmware, Wi-Fi, turret motion, sensor handling, and robot command protocol.
- `firmware/network.env` *(gitignored)*
  Wi-Fi credentials and server host/IP for your machine. Copy from `network.env.example`.
- `firmware/network.env.example`
  Template for `network.env`.
- `firmware/load_env.py`
  PlatformIO pre-build script that injects `network.env` values as C defines.
- `firmware/platformio.ini`
  PlatformIO environment and upload configuration.
- `scripts/discover_host.sh`
  Auto-detects this Mac's LAN IP and hostname and writes them into `firmware/network.env`.
- `control_server/point_cloud_server.py`
  Thin FastAPI bootstrap.
- `control_server/services/`
  Modular Python services for API routes, robot TCP handling, scan automation, residual analysis, clustering, capture storage, and geometry helpers.
- `control_server/captures/`
  Generated scan captures, saved baseline CSV, and residual CSV output.
- `static/index.html`
  Operator-facing FastAPI control page.
- `viewer-react/`
  Separate React + Three.js 3D viewer.
- `scripts/plot_lidar.py`
  Offline plotting utility for saved scan CSV files.

## Development Workflow

- **Version control:** GitHub
- **IDE:** VS Code
- **Firmware toolchain:** PlatformIO

Recommended day-to-day flow:

1. Edit firmware, server, or UI code in VS Code.
2. Use PlatformIO to build and upload firmware to the Nano RP2040 Connect.
3. Run the Python control server locally.
4. Run the React viewer locally if 3D inspection is needed.
5. Commit and review code through GitHub.

## Prerequisites

- Arduino Nano RP2040 Connect
- PlatformIO CLI or the PlatformIO VS Code extension
- Python 3
- Node.js and `npm`
- The robot and computer on the same Wi-Fi network

## Network Configuration

Wi-Fi credentials and the server host address are stored in `firmware/network.env`, which is **not committed to git** (it contains passwords). A template is provided at `firmware/network.env.example`.

### First-time setup

```bash
cp firmware/network.env.example firmware/network.env
# Edit firmware/network.env with your Wi-Fi SSID, password, and Mac's IP/hostname
```

`firmware/network.env` format:

```
WIFI_SSID=YourNetworkName
WIFI_PASSWORD=YourPassword
SERVER_HOST=Your-Mac-Hostname.local
SERVER_FALLBACK_IP=192.168.1.100
```

`load_env.py` (a PlatformIO pre-build script) reads this file and injects the values as C defines at compile time, so no credentials are hardcoded in source.

### Auto-discovering your Mac's host address

Run this before flashing whenever you switch networks:

```bash
bash scripts/discover_host.sh
```

This detects your Mac's current LAN IP and mDNS hostname and writes them into `firmware/network.env`. Use `--dry-run` to preview without writing.

## How To Run

### 1. Upload Firmware

From the repo root:

```bash
cd firmware
pio run -e nanorp2040connect -t upload
```

If `pio` is not on your `PATH`, use the direct binary:

```bash
/Users/haysoncheung/.platformio/penv/bin/pio run -e nanorp2040connect -t upload
```

Optional serial monitor:

```bash
cd firmware
pio device monitor -b 115200
```

### 2. Start The Control Server

From the repo root:

```bash
python3 control_server/point_cloud_server.py
```

Then open:

- `http://127.0.0.1:8000`

This page lets you:

- start a scan
- save or delete a baseline
- start or stop monitoring
- inspect live robot state and scan status
- view raw, residual, and baseline plots

### 3. Start The 3D Viewer

From the repo root:

```bash
cd viewer-react
npm install
npm run dev
```

Then open:

- `http://localhost:5173`

The 3D viewer shows:

- raw scans
- baseline data
- residual foreground geometry
- clustered debris candidates
- imported/exported scan bundles

### 4. Optional Offline Plotting

To plot a saved capture:

```bash
python3 scripts/plot_lidar.py --file control_server/captures/your_capture.csv
```

## Typical Operating Sequence

1. Upload firmware to the robot.
2. Start the FastAPI control server.
3. Power on the robot and confirm it connects.
4. Run a scan to verify live telemetry.
5. Save the last scan as a baseline for the region.
6. Start monitoring or run manual scans.
7. Inspect residuals and debris clusters to identify new objects in front of the baseline scene.

## Residual Logic

Residual mode is intentionally **foreground-only**.

For each current scan ray:

- compare the current distance to the baseline distance at the same `yaw,pitch`
- keep the ray only if the current reading is closer than the baseline by more than the monitoring tolerance
- render that current foreground geometry as the residual view

This means the residual view should show the shape of the current occluding object, not a signed difference field and not the baseline geometry.

## Debris Clustering Pipeline

The control server turns residual foreground points into debris candidates in `control_server/services/residuals.py`.

Current pipeline:

1. Build residual foreground samples from the current scan against the baseline.
2. Reject rays below the monitoring tolerance.
3. Run spatial neighbor search using a 3D hash grid.
4. Only connect points that are consistent in:
   - Cartesian distance
   - `yaw`
   - `pitch`
   - occlusion depth
5. Require local support so isolated stray points do not seed clusters.
6. Run a DBSCAN-like connected-components pass.
7. Run a stricter split pass to break apart clusters that are only weakly connected by thin bridges.
8. Reject candidates that do not satisfy debris heuristics such as minimum point count, angular support, occupancy ratio, and occlusion depth.

## Debris Score

Each accepted debris cluster receives a score in `control_server/services/residuals.py` using five normalized components:

- `max occlusion component = min(1.0, max_occlusion_mm / 250.0)`
- `mean occlusion component = min(1.0, mean_occlusion_mm / 150.0)`
- `count component = min(1.0, point_count / 24.0)`
- `occupancy component = min(1.0, occupancy_ratio)`
- `compactness component = max(0.0, 1.0 - min(1.0, diagonal_mm / DEBRIS_MAX_DIAGONAL_MM))`

Those are combined as:

```text
score =
  100 * (
    0.34 * max_occlusion_component +
    0.24 * mean_occlusion_component +
    0.22 * count_component +
    0.10 * occupancy_component +
    0.10 * compactness_component
  )
```

Interpretation:

- higher occlusion depth increases confidence
- more supporting points increases confidence
- denser angular occupancy increases confidence
- smaller, tighter clusters score better than long sprawling ones

The score is a ranking heuristic for debris candidates, not a probability.

## Default Network Ports

- FastAPI HTTP server: `8000`
- Robot TCP listener: `9000`

## Component Docs

- [Firmware README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/firmware/src/README.md)
- [Control Server README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/control_server/README.md)
- [Service Modules README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/control_server/services/README.md)
- [Control UI README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/static/README.md)
- [React Viewer README](/Users/haysoncheung/programs/PRAXIS/praxis3/WARD/viewer-react/README.md)

## Notes

- The robot firmware and control server must agree on the TCP protocol.
- The control UI depends on the FastAPI server being up.
- The React viewer depends on the FastAPI server APIs.
- Captures, baseline files, and residual outputs are stored under `control_server/captures/`.
