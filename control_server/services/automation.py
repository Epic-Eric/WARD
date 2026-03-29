from __future__ import annotations

import csv
import math
import shutil
import threading
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any

# Sensor / laser mounting geometry (all relative to the pitch-axis intersection)
_SENSOR_FACE_OFFSET_MM = 34.0   # sensor face is 34 mm forward of the axis
_SENSOR_ABOVE_AXIS_MM = 35.0    # sensor centre is 35 mm above the pitch axis
_LASER_ABOVE_AXIS_MM = 13.0     # laser is 22 mm below sensor → 35 - 22 = 13 mm above axis


def _compute_laser_angles(centroid_mm: list[float]) -> tuple[float, float]:
    """Return (yaw_deg, pitch_deg) motor angles to aim the clearing laser at a
    target whose 3-D position in the *axial frame* is given by centroid_mm.

    Yaw is unchanged: the forward offset is along the boresight so it introduces
    no lateral parallax.

    Pitch uses the same parallax formula as the sensor, but inverted:
        sensor: pitch_true  = pitch_motor - atan(t / R)
        laser:  pitch_motor = pitch_true  + atan(t / R)
    where t = _LASER_ABOVE_AXIS_MM (13 mm, smaller than the sensor's 35 mm
    because the laser is 22 mm below the sensor) and R is the distance from the
    pitch axis to the target.
    """
    cx, cy, cz = float(centroid_mm[0]), float(centroid_mm[1]), float(centroid_mm[2])
    r_horiz = math.hypot(cx, cy)
    yaw_deg = math.degrees(math.atan2(cy, cx))
    R = math.hypot(r_horiz, cz)
    pitch_true = math.degrees(math.atan2(cz, r_horiz)) if r_horiz > 0 or cz != 0 else 0.0
    parallax_correction = math.degrees(math.atan(_LASER_ABOVE_AXIS_MM / R)) if R > 0 else 0.0
    pitch_deg = pitch_true + parallax_correction
    return round(yaw_deg, 2), round(pitch_deg, 2)

from .common import load_json, write_json
from .config import (
    DEFAULT_RESIDUAL_TOLERANCE_MM,
    DEFAULT_SCAN_DEGREES,
    MONITOR_INTERVAL_SECONDS,
    SCAN_WAIT_TIMEOUT_SECONDS,
)
from .residuals import cluster_residual_samples, write_residual_capture
from .robot_hub import RobotTcpHub
from .scan_math import (
    build_surface_samples,
    format_scan_command,
    infer_scan_degrees_from_samples,
    normalize_bundle_view_payload,
    normalize_scan_degrees,
    point_map_from_samples,
    points_from_samples,
    pose_key,
    load_pose_point_map,
)


class ScanAutomationController:
    def __init__(self, robot_hub: RobotTcpHub, capture_dir: Path, baseline_path: Path, baseline_meta_path: Path) -> None:
        self.robot_hub = robot_hub
        self.capture_dir = capture_dir
        self.baseline_path = baseline_path
        self.baseline_meta_path = baseline_meta_path

        self._lock = threading.Lock()
        self._pending_operation: str | None = None
        self._operation_thread: threading.Thread | None = None
        self._monitoring_active = False
        self._monitor_thread: threading.Thread | None = None
        self._monitor_stop_event = threading.Event()
        self._next_monitor_scan_at: str | None = None
        self._automation_event = "Automation idle."
        self._automation_error: str | None = None
        self._configured_scan_degrees = DEFAULT_SCAN_DEGREES
        self._residual_tolerance_mm = DEFAULT_RESIDUAL_TOLERANCE_MM
        self._baseline_exists = False
        self._baseline_degrees: float | None = None
        self._baseline_saved_at: str | None = None
        self._baseline_source_capture: str | None = None
        self._baseline_point_map: dict[tuple[int, int], dict[str, float | int]] | None = None
        self._baseline_surface_samples: list[dict[str, float | int]] | None = None
        self._last_residual_path: str | None = None
        self._last_residual_summary: str | None = None
        self._last_residual_points: int | None = None
        self._last_monitor_capture: str | None = None
        self._last_debris_clusters: list[dict] = []
        self._clearing_debris_active = False
        self._clearing_debris_sequence = 0
        self._auto_clear_debris = True
        self._load_baseline_state()

    def _load_baseline_state(self) -> None:
        meta = load_json(self.baseline_meta_path)
        with self._lock:
            self._baseline_exists = self.baseline_path.exists()
            self._baseline_degrees = None
            self._baseline_saved_at = None
            self._baseline_source_capture = None
            self._baseline_point_map = None
            self._baseline_surface_samples = None
            if meta is not None:
                baseline_degrees = meta.get("scan_degrees")
                if baseline_degrees is not None:
                    try:
                        self._baseline_degrees = normalize_scan_degrees(float(baseline_degrees))
                    except (TypeError, ValueError):
                        self._baseline_degrees = None
                self._baseline_saved_at = meta.get("saved_at")
                self._baseline_source_capture = meta.get("source_capture")
            if self._baseline_exists:
                try:
                    self._baseline_point_map = load_pose_point_map(self.baseline_path)
                    self._baseline_surface_samples = build_surface_samples(self._baseline_point_map, fill_air=True)
                except Exception as exc:
                    self._baseline_exists = False
                    self._automation_error = str(exc)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "pending_operation": self._pending_operation,
                "monitoring_active": self._monitoring_active,
                "monitor_interval_seconds": MONITOR_INTERVAL_SECONDS,
                "next_monitor_scan_at": self._next_monitor_scan_at,
                "automation_event": self._automation_event,
                "automation_error": self._automation_error,
                "configured_scan_degrees": self._configured_scan_degrees,
                "residual_tolerance_mm": self._residual_tolerance_mm,
                "baseline_exists": self._baseline_exists,
                "baseline_path": str(self.baseline_path) if self._baseline_exists else None,
                "baseline_degrees": self._baseline_degrees,
                "baseline_saved_at": self._baseline_saved_at,
                "baseline_source_capture": self._baseline_source_capture,
                "last_residual_path": self._last_residual_path,
                "last_residual_summary": self._last_residual_summary,
                "last_residual_points": self._last_residual_points,
                "last_monitor_capture": self._last_monitor_capture,
                "last_debris_clusters": list(self._last_debris_clusters),
                "clearing_debris_active": self._clearing_debris_active,
                "clearing_debris_sequence": self._clearing_debris_sequence,
                "auto_clear_debris": self._auto_clear_debris,
            }

    def set_auto_clear_debris(self, enabled: bool) -> bool:
        with self._lock:
            self._auto_clear_debris = bool(enabled)
        return self._auto_clear_debris

    def aim_laser_at_cluster(self, centroid_mm: list[float]) -> tuple[float, float]:
        """Compute laser motor angles for a centroid (axial frame) and send MOVE_TO."""
        yaw_deg, pitch_deg = _compute_laser_angles(centroid_mm)
        self.robot_hub.send_command(f"MOVE_TO,{yaw_deg:.2f},{pitch_deg:.2f}")
        return yaw_deg, pitch_deg

    def set_residual_tolerance(self, tolerance_mm: float) -> float:
        normalized = max(0.0, float(tolerance_mm))
        with self._lock:
            self._residual_tolerance_mm = normalized
        return normalized

    def set_configured_scan_degrees(self, scan_degrees: float) -> float:
        normalized = normalize_scan_degrees(scan_degrees)
        with self._lock:
            self._configured_scan_degrees = normalized
        return normalized

    def build_live_plot_payload(self, raw_payload: dict[str, Any], mode: str = "auto", tolerance_mm: float | None = None) -> dict[str, Any]:
        with self._lock:
            baseline_available = (
                self._baseline_exists
                and self._baseline_point_map is not None
                and self._baseline_surface_samples is not None
                and self._pending_operation != "saving_baseline"
            )
            baseline_point_map = self._baseline_point_map
            baseline_surface_samples = [dict(sample) for sample in self._baseline_surface_samples] if self._baseline_surface_samples is not None else None
            configured_tolerance_mm = self._residual_tolerance_mm

        normalized_mode = mode.strip().lower() if mode else "auto"
        if normalized_mode not in {"auto", "raw", "residual", "baseline"}:
            normalized_mode = "auto"
        normalized_tolerance_mm = configured_tolerance_mm if tolerance_mm is None else self.set_residual_tolerance(tolerance_mm)

        raw_point_map = point_map_from_samples(raw_payload.get("samples", []))
        raw_surface_samples = build_surface_samples(raw_point_map, frame_id=raw_payload.get("frame_id"), fill_air=True)

        if normalized_mode == "baseline" and baseline_available and baseline_surface_samples is not None:
            return self._baseline_payload(raw_payload, baseline_surface_samples, normalized_mode, baseline_available, normalized_tolerance_mm)
        if normalized_mode == "raw" or not baseline_available or baseline_point_map is None:
            return self._raw_payload(raw_payload, raw_surface_samples, normalized_mode, baseline_available, normalized_tolerance_mm)
        return self._residual_payload(raw_payload, baseline_point_map, normalized_mode, baseline_available, normalized_tolerance_mm)

    def _baseline_payload(self, raw_payload: dict[str, Any], baseline_surface_samples: list[dict[str, float | int]], normalized_mode: str, baseline_available: bool, normalized_tolerance_mm: float) -> dict[str, Any]:
        payload = dict(raw_payload)
        payload["points"] = points_from_samples(baseline_surface_samples)
        payload["samples"] = baseline_surface_samples
        payload["render_mode"] = "baseline"
        payload["requested_mode"] = normalized_mode
        payload["baseline_available"] = baseline_available
        payload["residual_tolerance_mm"] = normalized_tolerance_mm
        payload["matched_points"] = len(baseline_surface_samples)
        payload["clusters"] = []
        payload["top_clusters"] = []
        return payload

    def _raw_payload(self, raw_payload: dict[str, Any], raw_surface_samples: list[dict[str, float | int]], normalized_mode: str, baseline_available: bool, normalized_tolerance_mm: float) -> dict[str, Any]:
        payload = dict(raw_payload)
        payload["points"] = points_from_samples(raw_surface_samples)
        payload["samples"] = raw_surface_samples
        payload["render_mode"] = "raw"
        payload["requested_mode"] = normalized_mode
        payload["baseline_available"] = baseline_available
        payload["residual_tolerance_mm"] = normalized_tolerance_mm
        payload["matched_points"] = len(raw_surface_samples)
        payload["clusters"] = []
        payload["top_clusters"] = []
        return payload

    def _residual_payload(self, raw_payload: dict[str, Any], baseline_point_map: dict[tuple[int, int], dict[str, float | int]], normalized_mode: str, baseline_available: bool, normalized_tolerance_mm: float) -> dict[str, Any]:
        current_samples = [dict(sample) for sample in raw_payload.get("samples", [])]
        if not current_samples:
            payload = dict(raw_payload)
            payload.update({"points": [], "samples": [], "render_mode": "residual", "requested_mode": normalized_mode, "baseline_available": baseline_available, "residual_tolerance_mm": normalized_tolerance_mm, "matched_points": 0, "clusters": [], "top_clusters": []})
            return payload

        target_poses = [(float(sample["yaw_deg"]), float(sample["pitch_deg"])) for sample in current_samples]
        baseline_surface = build_surface_samples(baseline_point_map, frame_id=raw_payload.get("frame_id"), target_poses=target_poses, fill_air=True)
        baseline_surface_map = point_map_from_samples(baseline_surface)
        residual_samples: list[dict[str, float | int]] = []
        for current_sample in current_samples:
            baseline_sample = baseline_surface_map.get(pose_key(float(current_sample["yaw_deg"]), float(current_sample["pitch_deg"])))
            if baseline_sample is None:
                continue
            baseline_distance_mm = float(baseline_sample["distance_mm"])
            current_distance_mm = float(current_sample["distance_mm"])
            occlusion_distance_mm = baseline_distance_mm - current_distance_mm
            if current_distance_mm >= baseline_distance_mm or occlusion_distance_mm < normalized_tolerance_mm:
                continue
            residual_samples.append(
                {
                    "frame_id": int(current_sample["frame_id"]),
                    "point_index": int(current_sample["point_index"]),
                    "yaw_deg": float(current_sample["yaw_deg"]),
                    "pitch_deg": float(current_sample["pitch_deg"]),
                    "distance_mm": current_distance_mm,
                    "x_mm": float(current_sample["x_mm"]),
                    "y_mm": float(current_sample["y_mm"]),
                    "z_mm": float(current_sample["z_mm"]),
                    "is_air": False,
                    "occlusion_distance_mm": occlusion_distance_mm,
                    "baseline_x_mm": float(baseline_sample["x_mm"]),
                    "baseline_y_mm": float(baseline_sample["y_mm"]),
                    "baseline_z_mm": float(baseline_sample["z_mm"]),
                    "baseline_distance_mm": baseline_distance_mm,
                }
            )

        clusters, sample_to_cluster = cluster_residual_samples(residual_samples, normalized_tolerance_mm)
        cluster_lookup = {cluster["cluster_id"]: cluster for cluster in clusters}
        annotated_points: list[list[float]] = []
        annotated_samples: list[dict[str, float | int]] = []
        for index, sample in enumerate(residual_samples):
            cluster_id = sample_to_cluster.get(index)
            if cluster_id is None:
                continue
            sample["cluster_id"] = cluster_id
            sample["cluster_score"] = cluster_lookup[cluster_id]["score"]
            annotated_samples.append(sample)
            annotated_points.append([float(sample["x_mm"]), float(sample["y_mm"]), float(sample["z_mm"]), float(sample["distance_mm"]), 0.0, float(sample.get("occlusion_distance_mm", 0.0)), cluster_id])

        payload = dict(raw_payload)
        payload["points"] = annotated_points
        payload["samples"] = annotated_samples
        payload["render_mode"] = "residual"
        payload["requested_mode"] = normalized_mode
        payload["baseline_available"] = baseline_available
        payload["residual_tolerance_mm"] = normalized_tolerance_mm
        payload["matched_points"] = len(annotated_points)
        payload["clusters"] = clusters
        payload["top_clusters"] = clusters[:3]
        return payload

    def _start_scan_and_wait(self, scan_degrees: float) -> dict[str, Any]:
        token = self.robot_hub.scan_result_token()
        self.robot_hub.send_command(format_scan_command(scan_degrees))
        return self.robot_hub.wait_for_scan_completion(token, timeout_seconds=SCAN_WAIT_TIMEOUT_SECONDS)

    def _persist_baseline_from_capture(
        self,
        source_path: Path,
        scan_degrees: float | None = None,
        source_capture: str | None = None,
    ) -> None:
        point_map = load_pose_point_map(source_path)
        resolved_scan_degrees = scan_degrees or infer_scan_degrees_from_samples(list(point_map.values())) or DEFAULT_SCAN_DEGREES
        resolved_scan_degrees = normalize_scan_degrees(float(resolved_scan_degrees))
        self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source_path, self.baseline_path)
        saved_at = datetime.now().isoformat(timespec="seconds")
        resolved_source_capture = source_capture or str(source_path)
        write_json(
            self.baseline_meta_path,
            {
                "saved_at": saved_at,
                "source_capture": resolved_source_capture,
                "scan_degrees": resolved_scan_degrees,
            },
        )
        with self._lock:
            self._baseline_exists = True
            self._baseline_degrees = resolved_scan_degrees
            self._baseline_saved_at = saved_at
            self._baseline_source_capture = resolved_source_capture
            self._baseline_point_map = point_map
            self._baseline_surface_samples = build_surface_samples(point_map, fill_air=True)
            self._automation_error = None
            self._automation_event = f"Saved baseline from {source_path.name} at {resolved_scan_degrees:.2f} degrees."

    def save_baseline_from_latest_capture(self) -> None:
        with self._lock:
            if self._pending_operation is not None:
                raise RuntimeError("Wait for the current automation operation to finish.")
            if self._monitoring_active:
                raise RuntimeError("Stop monitoring before saving a baseline.")
        status = self.robot_hub.snapshot()
        if status["scan_in_progress"]:
            raise RuntimeError("Wait for the current scan to finish before saving the baseline.")
        last_scan_result = status.get("last_scan_result") or {}
        capture_path_raw = last_scan_result.get("capture_path") or status.get("last_capture")
        if not capture_path_raw:
            raise RuntimeError("No completed scan is available to save as the baseline.")
        source_path = Path(str(capture_path_raw))
        if not source_path.exists():
            raise RuntimeError("The latest scan capture file no longer exists.")
        self._persist_baseline_from_capture(source_path)

    def _baseline_worker(self, scan_degrees: float) -> None:
        try:
            result = self._start_scan_and_wait(scan_degrees)
            if result.get("status") != "completed":
                raise RuntimeError(f"Baseline scan did not complete: {result.get('status')}")
            capture_path_raw = result.get("capture_path")
            if not capture_path_raw:
                raise RuntimeError("Baseline scan completed without a capture file.")
            source_path = Path(capture_path_raw)
            self._persist_baseline_from_capture(source_path, scan_degrees=scan_degrees)
        except Exception as exc:
            with self._lock:
                self._automation_error = str(exc)
                self._automation_event = "Baseline capture failed."
        finally:
            with self._lock:
                self._pending_operation = None
                self._operation_thread = None

    def import_baseline_from_view(self, view_payload: dict[str, Any], scan_degrees: float | None = None, source_name: str | None = None) -> None:
        normalized_view = normalize_bundle_view_payload(view_payload)
        if normalized_view is None:
            raise RuntimeError("Imported baseline payload did not contain valid samples.")
        normalized_samples = normalized_view["samples"]
        resolved_scan_degrees = scan_degrees or infer_scan_degrees_from_samples(normalized_samples) or DEFAULT_SCAN_DEGREES
        resolved_scan_degrees = normalize_scan_degrees(float(resolved_scan_degrees))
        with self._lock:
            if self._pending_operation is not None:
                raise RuntimeError("Wait for the current automation operation to finish.")
            if self._monitoring_active:
                raise RuntimeError("Stop monitoring before importing a baseline.")
        self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
        with self.baseline_path.open("w", newline="", encoding="utf-8") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(["frame_id", "point_index", "yaw_deg", "pitch_deg", "distance_mm", "x_mm", "y_mm", "z_mm"])
            for sample in normalized_samples:
                writer.writerow([int(sample["frame_id"]), int(sample["point_index"]), sample["yaw_deg"], sample["pitch_deg"], sample["distance_mm"], sample["x_mm"], sample["y_mm"], sample["z_mm"]])
        saved_at = datetime.now().isoformat(timespec="seconds")
        source_capture = source_name or "imported_bundle"
        write_json(self.baseline_meta_path, {"saved_at": saved_at, "source_capture": source_capture, "scan_degrees": resolved_scan_degrees})
        with self._lock:
            self._baseline_exists = True
            self._baseline_degrees = resolved_scan_degrees
            self._baseline_saved_at = saved_at
            self._baseline_source_capture = source_capture
            self._baseline_point_map = load_pose_point_map(self.baseline_path)
            self._baseline_surface_samples = build_surface_samples(self._baseline_point_map, fill_air=True)
            self._automation_error = None
            self._automation_event = f"Imported baseline from {source_capture} at {resolved_scan_degrees:.2f} degrees."

    def delete_baseline(self) -> None:
        with self._lock:
            if self._pending_operation is not None:
                raise RuntimeError("Wait for the current automation operation to finish.")
            if self._monitoring_active:
                raise RuntimeError("Stop monitoring before deleting the baseline.")
        if self.baseline_path.exists():
            self.baseline_path.unlink()
        if self.baseline_meta_path.exists():
            self.baseline_meta_path.unlink()
        with self._lock:
            self._baseline_exists = False
            self._baseline_degrees = None
            self._baseline_saved_at = None
            self._baseline_source_capture = None
            self._baseline_point_map = None
            self._baseline_surface_samples = None
            self._last_residual_path = None
            self._last_residual_summary = None
            self._last_residual_points = None
            self._automation_error = None
            self._automation_event = "Baseline deleted."

    def start_monitoring(self, scan_degrees: float, auto_clear_debris: bool = True) -> None:
        normalized = self.set_configured_scan_degrees(scan_degrees)
        with self._lock:
            if not self._baseline_exists:
                raise RuntimeError("Save a baseline before starting monitoring.")
            if self._pending_operation is not None:
                raise RuntimeError("Another automation operation is already running.")
            if self._monitoring_active:
                raise RuntimeError("Monitoring is already active.")
            self._monitoring_active = True
            self._auto_clear_debris = bool(auto_clear_debris)
            self._monitor_stop_event.clear()
            self._next_monitor_scan_at = None
            self._automation_error = None
            self._automation_event = f"Monitoring started with {normalized:.2f} degree sweeps."
            self._monitor_thread = threading.Thread(target=self._monitoring_worker, args=(normalized,), name="monitoring-worker", daemon=True)
            self._monitor_thread.start()

    def stop_monitoring(self, request_robot_stop: bool = True) -> None:
        with self._lock:
            active = self._monitoring_active
            self._monitor_stop_event.set()
            self._next_monitor_scan_at = None
            if active:
                self._automation_event = "Stopping monitoring loop."
        if request_robot_stop:
            status = self.robot_hub.snapshot()
            if status["robot_connected"] and status["scan_in_progress"]:
                try:
                    self.robot_hub.send_command("STOP_SCAN")
                except RuntimeError:
                    pass

    def _monitoring_worker(self, scan_degrees: float) -> None:
        try:
            while not self._monitor_stop_event.is_set():
                with self._lock:
                    self._automation_event = f"Starting monitoring scan at {scan_degrees:.2f} degrees."
                    self._automation_error = None
                try:
                    result = self._start_scan_and_wait(scan_degrees)
                    if result.get("status") != "completed":
                        raise RuntimeError(f"Monitoring scan did not complete: {result.get('status')}")
                    capture_path_raw = result.get("capture_path")
                    if not capture_path_raw:
                        raise RuntimeError("Monitoring scan completed without a capture file.")
                    capture_path = Path(capture_path_raw)
                    with self._lock:
                        residual_tolerance_mm = self._residual_tolerance_mm
                    residual_path, summary, matched_points, debris_clusters = write_residual_capture(self.capture_dir, self.baseline_path, capture_path, residual_tolerance_mm)
                    with self._lock:
                        self._last_monitor_capture = str(capture_path)
                        self._last_residual_path = str(residual_path)
                        self._last_residual_summary = summary
                        self._last_residual_points = matched_points
                        self._last_debris_clusters = debris_clusters
                        cluster_count = len(debris_clusters)
                        self._automation_event = (
                            f"Monitoring scan complete. {cluster_count} debris cluster(s) detected."
                            if cluster_count else
                            f"Monitoring scan complete. No debris detected."
                        )

                    if debris_clusters and not self._monitor_stop_event.is_set() and self._auto_clear_debris:
                        with self._lock:
                            self._clearing_debris_active = True
                            self._clearing_debris_sequence += 1
                        try:
                            for cluster in debris_clusters:
                                if self._monitor_stop_event.is_set():
                                    break
                                snap = self.robot_hub.snapshot()
                                if not snap.get("robot_connected"):
                                    break
                                centroid_mm = cluster.get("centroid_mm", [0.0, 0.0, 0.0])
                                yaw_deg, pitch_deg = _compute_laser_angles(centroid_mm)
                                cluster_id = cluster.get("cluster_id", "?")
                                with self._lock:
                                    self._automation_event = (
                                        f"Clearing debris: {cluster_id} "
                                        f"(laser yaw={yaw_deg:.1f}\u00b0, pitch={pitch_deg:.1f}\u00b0)"
                                    )
                                try:
                                    self.robot_hub.send_command(f"MOVE_TO,{yaw_deg:.2f},{pitch_deg:.2f}")
                                except RuntimeError as exc:
                                    with self._lock:
                                        self._automation_error = str(exc)
                                    break
                                # Poll until robot returns to idle (up to 30 s)
                                for _ in range(150):
                                    if self._monitor_stop_event.wait(0.2):
                                        break
                                    if self.robot_hub.snapshot().get("robot_state") in {"idle", "disconnected"}:
                                        break
                                with self._lock:
                                    self._automation_event = f"Debris cleared: {cluster_id}"
                        finally:
                            with self._lock:
                                self._clearing_debris_active = False
                except Exception as exc:
                    with self._lock:
                        self._automation_error = str(exc)
                        self._automation_event = "Monitoring scan failed."
                next_scan_at = datetime.now() + timedelta(seconds=MONITOR_INTERVAL_SECONDS)
                with self._lock:
                    self._next_monitor_scan_at = next_scan_at.isoformat(timespec="seconds")
                if self._monitor_stop_event.wait(MONITOR_INTERVAL_SECONDS):
                    break
        finally:
            with self._lock:
                self._monitoring_active = False
                self._monitor_thread = None
                self._next_monitor_scan_at = None
                if self._automation_event == "Stopping monitoring loop.":
                    self._automation_error = None
