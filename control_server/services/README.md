# Control Server Services

This folder contains the modular service layer behind the WARD control server.

## Design Intent

The goal of this split is to keep the server readable and modular:

- transport logic lives in one place
- automation logic lives in one place
- scan geometry and interpolation live in one place
- residual and debris logic live in one place
- API routing stays thin

## Service Map

### `api.py`

Owns the FastAPI HTTP layer.

Responsibilities:

- create the FastAPI app
- expose `/api/status`
- expose `/api/live-points`
- expose robot control endpoints such as start/stop/hard-stop/zero
- expose baseline and monitoring endpoints

This file should stay thin. It validates requests and delegates real work to `robot_hub.py` and `automation.py`.

### `automation.py`

Owns higher-level scan workflow and derived products.

Responsibilities:

- save the last captured scan as the baseline
- load baseline state and metadata
- build live raw, baseline, and residual payloads
- start/stop monitoring
- write residual captures during monitoring
- track automation status fields exposed to the UI

This is the main coordinator between raw robot data and operator-facing derived state.

### `capture.py`

Owns capture-file creation and recorder state.

Responsibilities:

- create new CSV capture files
- write raw point rows
- flush and close captures safely
- track simple session counters through `SessionStats`

### `common.py`

Owns small shared helpers.

Responsibilities:

- timestamp formatting
- LAN IP and hostname detection
- JSON load/write helpers
- optional float formatting for CSV writing

### `config.py`

Owns constants and paths.

Responsibilities:

- filesystem paths
- network ports
- scan defaults
- interpolation constants
- residual tolerance defaults
- clustering thresholds
- debris heuristic thresholds

When tuning clustering behavior, this is the first place to check.

### `housekeeping.py`

Owns capture-retention utilities.

Responsibilities:

- prune old `point_cloud_*.csv` and `residual_*.csv` files
- apply keep-latest and retention-day policies

This is a utility module. It is not a transport or API module.

### `residuals.py`

Owns residual-file generation, debris clustering, and debris scoring.

Responsibilities:

- write residual CSV captures from baseline vs current scan
- cluster residual foreground samples into debris candidates
- compute cluster bounding boxes
- compute debris scores

This is the most important module for debris detection.

### `robot_hub.py`

Owns the robot TCP connection and live robot state.

Responsibilities:

- listen for the robot TCP client
- parse robot protocol messages
- update robot state, pose, and sensor state
- record live point samples
- expose live snapshots to the API and automation layers
- track command state and scan completion results

If the UI state looks wrong while the robot is running, this is usually the first module to inspect.

### `scan_math.py`

Owns geometry and interpolation helpers.

Responsibilities:

- normalize scan degrees
- parse and load capture CSVs
- convert spherical samples to Cartesian coordinates
- build dense surface samples on the angular grid
- encode air / no-return rays
- infer scan degrees from captures

This module is intentionally pure and utility-like.

## Debris Detection Pipeline

Debris detection currently works like this:

1. Build residual foreground rays by comparing current scan vs baseline at matching `yaw,pitch`.
2. Keep only rays where the current reading is closer than the baseline by more than the monitoring tolerance.
3. Use a 3D spatial hash to find nearby candidate neighbors.
4. Only connect residual rays when they are compatible in:
   - Cartesian distance
   - `yaw` gap
   - `pitch` gap
   - occlusion-depth difference
5. Require local support so isolated points do not become clusters.
6. Run a DBSCAN-like connected-components pass.
7. Run a stricter split pass to break apart weakly connected super-clusters.
8. Reject low-quality clusters using debris heuristics.

## Debris Heuristics

An accepted cluster must satisfy multiple checks, including:

- minimum point count
- minimum unique yaw coverage
- minimum unique pitch coverage
- minimum maximum occlusion
- minimum mean occlusion
- minimum occupancy ratio
- maximum allowed bounding-box diagonal

These heuristics are tuned to reject:

- isolated stray points
- sparse sheets
- clusters held together by thin bridges
- very large sprawling regions that are unlikely to be a single debris object

## Debris Score Formula

The debris score is implemented by `score_debris_cluster(...)` in `residuals.py`.

Normalized components:

- `max_component = min(1.0, max_occlusion_mm / 250.0)`
- `mean_component = min(1.0, mean_occlusion_mm / 150.0)`
- `count_component = min(1.0, point_count / 24.0)`
- `occupancy_component = min(1.0, occupancy_ratio)`
- `compactness_component = max(0.0, 1.0 - min(1.0, diagonal_mm / DEBRIS_MAX_DIAGONAL_MM))`

Final score:

```text
score =
  100 * (
    0.34 * max_component +
    0.24 * mean_component +
    0.22 * count_component +
    0.10 * occupancy_component +
    0.10 * compactness_component
  )
```

Interpretation:

- larger foreground occlusions score higher
- denser, better-supported clusters score higher
- tighter and more compact clusters score higher

The score is a prioritization heuristic for debris candidates. It is not a calibrated probability.

## Practical Tuning Notes

If clustering is too permissive:

- reduce `CLUSTER_MAX_YAW_GAP_DEG`
- reduce `CLUSTER_MAX_PITCH_GAP_DEG`
- reduce `CLUSTER_OCCLUSION_DELTA_MM`
- increase `DEBRIS_MIN_OCCUPANCY_RATIO`

If clusters are still merging too aggressively:

- tighten the `CLUSTER_SPLIT_*` constants
- lower `CLUSTER_SPLIT_MAX_RADIUS_MM`
- lower `CLUSTER_SPLIT_OCCLUSION_DELTA_MM`

If valid debris is being rejected:

- reduce `DEBRIS_MIN_POINTS`
- reduce `DEBRIS_MIN_MAX_OCCLUSION_MM`
- reduce `DEBRIS_MIN_MEAN_OCCLUSION_MM`

## Ownership Rule Of Thumb

- transport issue: `robot_hub.py`
- HTTP/API issue: `api.py`
- baseline/monitoring issue: `automation.py`
- residual or debris issue: `residuals.py`
- geometry/interpolation issue: `scan_math.py`
- capture-file issue: `capture.py`
