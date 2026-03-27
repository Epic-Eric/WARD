#!/usr/bin/env python3
"""
FastAPI control server for the Nano RP2040 Connect point cloud scanner.

HTTP UI:
    http://127.0.0.1:8000

Robot TCP port:
    9000
"""

from __future__ import annotations

import csv
import json
import math
import shutil
import socket
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path
from typing import Any

try:
    from fastapi import FastAPI, HTTPException, Request
    from fastapi.responses import FileResponse
    import uvicorn
except ModuleNotFoundError as exc:
    FastAPI = None
    HTTPException = RuntimeError
    Request = Any
    FileResponse = None
    uvicorn = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
INDEX_HTML = STATIC_DIR / "index.html"
CAPTURE_DIR = BASE_DIR / "captures"
BASELINE_PATH = CAPTURE_DIR / "baseline_scan.csv"
BASELINE_META_PATH = CAPTURE_DIR / "baseline_scan_meta.json"

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000
ROBOT_HOST = "0.0.0.0"
ROBOT_PORT = 9000
SOCKET_TIMEOUT_SECONDS = 1.0
ROBOT_STALE_TIMEOUT_SECONDS = 4.0
SCAN_WAIT_TIMEOUT_SECONDS = 900.0
MONITOR_INTERVAL_SECONDS = 300
DEFAULT_SCAN_DEGREES = 20.0
MAX_SCAN_DEGREES = 360.0
DEFAULT_YAW_STEP_DEG = 1.0
DEFAULT_PITCH_STEP_DEG = 0.9
INTERPOLATION_NEIGHBOR_RADIUS = 2.5
INTERPOLATION_MAX_NEIGHBORS = 8
DEFAULT_RESIDUAL_TOLERANCE_MM = 25.0
AIR_DISTANCE_MM = 20000.0


def timestamp_string() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def detect_lan_ip() -> str:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        probe.connect(("8.8.8.8", 80))
        return probe.getsockname()[0]
    except OSError:
        return "unknown"
    finally:
        probe.close()


def detect_hostname() -> str:
    hostname = socket.gethostname()
    if hostname.endswith(".local"):
        return hostname
    return f"{hostname}.local"


def normalize_scan_degrees(scan_degrees: float) -> float:
    normalized = float(scan_degrees)
    if normalized <= 0.0 or normalized > MAX_SCAN_DEGREES:
        raise ValueError(
            f"scan_degrees must be between 0 and {MAX_SCAN_DEGREES:.0f}."
        )
    return round(normalized, 2)


def format_scan_command(scan_degrees: float) -> str:
    return f"START_SCAN,{normalize_scan_degrees(scan_degrees):.2f}"


def format_optional_float(value: float | None) -> str | None:
    if value is None:
        return None
    return f"{value:.2f}"


def parse_number(value: str | None) -> float:
    if value is None:
        raise ValueError("Missing numeric value.")
    return float(value)


def pose_key(yaw_deg: float, pitch_deg: float) -> tuple[int, int]:
    return (int(round(yaw_deg * 100.0)), int(round(pitch_deg * 100.0)))


def spherical_to_cartesian_mm(distance_mm: float, yaw_deg: float, pitch_deg: float) -> tuple[float, float, float]:
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    cos_pitch = math.cos(pitch_rad)
    return (
        distance_mm * cos_pitch * math.cos(yaw_rad),
        distance_mm * cos_pitch * math.sin(yaw_rad),
        distance_mm * math.sin(pitch_rad),
    )


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True), encoding="utf-8")


@dataclass
class SessionStats:
    frames: int = 0
    points: int = 0
    current_frame: int | None = None
    current_frame_points: int = 0

    def reset(self) -> None:
        self.frames = 0
        self.points = 0
        self.current_frame = None
        self.current_frame_points = 0


def open_capture_file(
    output_dir: Path, prefix: str = "point_cloud"
) -> tuple[Path, csv.writer, Any]:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"{prefix}_{timestamp_string()}.csv"
    file_obj = csv_path.open("w", newline="", encoding="utf-8")
    writer = csv.writer(file_obj)
    writer.writerow(
        [
            "frame_id",
            "point_index",
            "yaw_deg",
            "pitch_deg",
            "distance_mm",
            "x_mm",
            "y_mm",
            "z_mm",
        ]
    )
    return csv_path, writer, file_obj


def load_pose_point_map(
    csv_path: Path,
) -> dict[tuple[int, int], dict[str, float | int]]:
    if not csv_path.exists():
        raise FileNotFoundError(f"Capture file not found: {csv_path}")

    point_map: dict[tuple[int, int], dict[str, float | int]] = {}
    with csv_path.open("r", newline="", encoding="utf-8") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            try:
                point_index = int(row["point_index"])
                yaw_deg = parse_number(row["yaw_deg"])
                pitch_deg = parse_number(row["pitch_deg"])
                point_map[pose_key(yaw_deg, pitch_deg)] = {
                    "frame_id": int(float(row["frame_id"])),
                    "point_index": point_index,
                    "yaw_deg": yaw_deg,
                    "pitch_deg": pitch_deg,
                    "distance_mm": parse_number(row["distance_mm"]),
                    "x_mm": parse_number(row["x_mm"]),
                    "y_mm": parse_number(row["y_mm"]),
                    "z_mm": parse_number(row["z_mm"]),
                }
            except (KeyError, TypeError, ValueError):
                continue

    if not point_map:
        raise RuntimeError(f"No valid points were found in {csv_path}")

    return point_map


def point_map_from_samples(
    samples: list[dict[str, float | int]],
) -> dict[tuple[int, int], dict[str, float | int]]:
    point_map: dict[tuple[int, int], dict[str, float | int]] = {}
    for row in samples:
        try:
            point_index = int(row["point_index"])
            yaw_deg = float(row["yaw_deg"])
            pitch_deg = float(row["pitch_deg"])
            point_map[pose_key(yaw_deg, pitch_deg)] = {
                "frame_id": int(row["frame_id"]),
                "point_index": point_index,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg,
                "distance_mm": float(row["distance_mm"]),
                "x_mm": float(row["x_mm"]),
                "y_mm": float(row["y_mm"]),
                "z_mm": float(row["z_mm"]),
            }
        except (KeyError, TypeError, ValueError):
            continue
    return point_map


def normalize_bundle_view_payload(payload: dict[str, Any] | None) -> dict[str, Any] | None:
    if not isinstance(payload, dict):
        return None

    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        return None

    normalized_samples: list[dict[str, float | int | bool]] = []
    for index, sample in enumerate(samples):
        if not isinstance(sample, dict):
            continue
        try:
            normalized_samples.append(
                {
                    "frame_id": int(sample.get("frame_id", 1)),
                    "point_index": int(sample.get("point_index", index)),
                    "yaw_deg": float(sample["yaw_deg"]),
                    "pitch_deg": float(sample["pitch_deg"]),
                    "distance_mm": float(sample["distance_mm"]),
                    "x_mm": float(sample["x_mm"]),
                    "y_mm": float(sample["y_mm"]),
                    "z_mm": float(sample["z_mm"]),
                    "is_air": bool(sample.get("is_air", False)),
                }
            )
        except (KeyError, TypeError, ValueError):
            continue

    if not normalized_samples:
        return None

    return {
        "samples": normalized_samples,
        "render_mode": str(payload.get("render_mode", "raw")),
    }


def infer_scan_degrees_from_samples(samples: list[dict[str, float | int | bool]]) -> float | None:
    yaw_values = sorted(
        {
            round(float(sample["yaw_deg"]), 2)
            for sample in samples
        }
    )
    if not yaw_values:
        return None
    inferred = max(1.0, yaw_values[-1] - yaw_values[0])
    try:
        return normalize_scan_degrees(inferred)
    except ValueError:
        return None


def infer_axis_grid(
    point_map: dict[tuple[int, int], dict[str, float | int]],
    axis_name: str,
    fallback_step: float,
) -> tuple[list[float], float]:
    values = sorted(
        {
            round(float(sample[axis_name]), 2)
            for sample in point_map.values()
        }
    )
    if not values:
        return [], fallback_step
    if len(values) == 1:
        return values, fallback_step

    diffs = [
        round(next_value - current_value, 2)
        for current_value, next_value in zip(values, values[1:])
        if next_value - current_value > 0.01
    ]
    step = fallback_step
    if diffs:
        sorted_diffs = sorted(diffs)
        step = sorted_diffs[len(sorted_diffs) // 2]

    count = max(1, int(round((values[-1] - values[0]) / step)))
    expanded = [round(values[0] + index * step, 2) for index in range(count + 1)]
    return expanded, step


def interpolate_distance_mm(
    point_map: dict[tuple[int, int], dict[str, float | int]],
    yaw_deg: float,
    pitch_deg: float,
    yaw_step: float,
    pitch_step: float,
) -> float | None:
    exact_sample = point_map.get(pose_key(yaw_deg, pitch_deg))
    if exact_sample is not None:
        return float(exact_sample["distance_mm"])

    yaw_norm = max(yaw_step, 0.01)
    pitch_norm = max(pitch_step, 0.01)
    candidates: list[tuple[float, float]] = []
    for sample in point_map.values():
        delta_yaw = abs(float(sample["yaw_deg"]) - yaw_deg)
        delta_pitch = abs(float(sample["pitch_deg"]) - pitch_deg)
        normalized_distance = math.hypot(delta_yaw / yaw_norm, delta_pitch / pitch_norm)
        candidates.append((normalized_distance, float(sample["distance_mm"])))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    neighbors = [
        item for item in candidates if item[0] <= INTERPOLATION_NEIGHBOR_RADIUS
    ][:INTERPOLATION_MAX_NEIGHBORS]
    if not neighbors:
        return None

    weighted_sum = 0.0
    total_weight = 0.0
    for normalized_distance, distance_mm in neighbors:
        weight = 1.0 / max(normalized_distance, 0.15) ** 2
        weighted_sum += distance_mm * weight
        total_weight += weight

    if total_weight <= 0.0:
        return None
    return weighted_sum / total_weight


def resolve_surface_distance_mm(
    point_map: dict[tuple[int, int], dict[str, float | int]],
    yaw_deg: float,
    pitch_deg: float,
    yaw_step: float,
    pitch_step: float,
    fill_air: bool = False,
) -> tuple[float | None, bool]:
    exact_sample = point_map.get(pose_key(yaw_deg, pitch_deg))
    if exact_sample is not None:
        return float(exact_sample["distance_mm"]), False

    interpolated_distance = interpolate_distance_mm(
        point_map, yaw_deg, pitch_deg, yaw_step, pitch_step
    )
    if interpolated_distance is not None:
        return interpolated_distance, False

    if fill_air:
        return AIR_DISTANCE_MM, True
    return None, True


def build_surface_samples(
    point_map: dict[tuple[int, int], dict[str, float | int]],
    frame_id: int | None = None,
    target_poses: list[tuple[float, float]] | None = None,
    fill_air: bool = False,
) -> list[dict[str, float | int]]:
    if not point_map:
        return []

    yaw_values, yaw_step = infer_axis_grid(point_map, "yaw_deg", DEFAULT_YAW_STEP_DEG)
    pitch_values, pitch_step = infer_axis_grid(point_map, "pitch_deg", DEFAULT_PITCH_STEP_DEG)
    if target_poses is None:
        target_poses = [(yaw_deg, pitch_deg) for yaw_deg in yaw_values for pitch_deg in pitch_values]
    else:
        target_poses = sorted(
            {
                (round(float(yaw_deg), 2), round(float(pitch_deg), 2))
                for yaw_deg, pitch_deg in target_poses
            }
        )

    resolved_frame_id = frame_id
    if resolved_frame_id is None:
        resolved_frame_id = int(next(iter(point_map.values()))["frame_id"])

    surface_samples: list[dict[str, float | int]] = []
    for point_index, (yaw_deg, pitch_deg) in enumerate(target_poses):
        distance_mm, is_air = resolve_surface_distance_mm(
            point_map, yaw_deg, pitch_deg, yaw_step, pitch_step, fill_air=fill_air
        )
        if distance_mm is None:
            continue
        x_mm, y_mm, z_mm = spherical_to_cartesian_mm(distance_mm, yaw_deg, pitch_deg)
        surface_samples.append(
            {
                "frame_id": resolved_frame_id,
                "point_index": point_index,
                "yaw_deg": yaw_deg,
                "pitch_deg": pitch_deg,
                "distance_mm": distance_mm,
                "x_mm": x_mm,
                "y_mm": y_mm,
                "z_mm": z_mm,
                "is_air": is_air,
            }
        )

    return surface_samples


def points_from_samples(samples: list[dict[str, float | int]]) -> list[list[float]]:
    return [
        [
            float(sample["x_mm"]),
            float(sample["y_mm"]),
            float(sample["z_mm"]),
            float(sample["distance_mm"]),
            1.0 if bool(sample.get("is_air", False)) else 0.0,
            float(sample.get("occlusion_distance_mm", 0.0)),
        ]
        for sample in samples
    ]


def write_residual_capture(
    output_dir: Path,
    baseline_path: Path,
    current_path: Path,
    tolerance_mm: float = DEFAULT_RESIDUAL_TOLERANCE_MM,
) -> tuple[Path, str, int]:
    baseline_points = load_pose_point_map(baseline_path)
    current_points = load_pose_point_map(current_path)
    current_samples = [dict(sample) for sample in current_points.values()]
    if not current_samples:
        raise RuntimeError("Current scan did not contain any rays.")

    target_poses = [
        (float(sample["yaw_deg"]), float(sample["pitch_deg"])) for sample in current_samples
    ]
    baseline_surface = build_surface_samples(
        baseline_points,
        frame_id=int(next(iter(current_samples))["frame_id"]),
        target_poses=target_poses,
        fill_air=True,
    )
    if not baseline_surface:
        raise RuntimeError("Baseline surface could not be built.")

    baseline_surface_map = point_map_from_samples(baseline_surface)

    residual_path, writer, file_obj = open_capture_file(output_dir, "residual")
    written_points = 0
    try:
        for current in current_samples:
            key = pose_key(float(current["yaw_deg"]), float(current["pitch_deg"]))
            baseline = baseline_surface_map.get(key)
            if baseline is None:
                continue
            baseline_distance_mm = float(baseline["distance_mm"])
            current_distance_mm = float(current["distance_mm"])
            occlusion_distance_mm = baseline_distance_mm - current_distance_mm
            if current_distance_mm >= baseline_distance_mm:
                continue
            if occlusion_distance_mm < max(0.0, tolerance_mm):
                continue
            writer.writerow(
                [
                    current["frame_id"],
                    current["point_index"],
                    format_optional_float(current["yaw_deg"]),
                    format_optional_float(current["pitch_deg"]),
                    format_optional_float(current_distance_mm),
                    format_optional_float(current["x_mm"]),
                    format_optional_float(current["y_mm"]),
                    format_optional_float(current["z_mm"]),
                ]
            )
            written_points += 1
    finally:
        file_obj.flush()
        file_obj.close()

    summary = f"Residual saved with {written_points} foreground rays from the current scan."
    return residual_path, summary, written_points


class CaptureRecorder:
    def __init__(self, output_dir: Path) -> None:
        self.output_dir = output_dir
        self.csv_path: Path | None = None
        self.writer: csv.writer | None = None
        self.file_obj: Any | None = None
        self.rows_written = 0
        self.pending_reason: str | None = "connection start"
        self.last_capture_path: Path | None = None

    def start_new_capture(self, reason: str) -> None:
        self.close()
        self.csv_path, self.writer, self.file_obj = open_capture_file(self.output_dir)
        self.rows_written = 0
        self.pending_reason = None
        print(f"Writing capture to {self.csv_path} ({reason})")

    def ensure_open(self) -> None:
        if self.writer is None or self.file_obj is None:
            self.start_new_capture(self.pending_reason or "implicit capture start")

    def mark_run_boundary(self, reason: str) -> None:
        if self.rows_written > 0:
            self.close()
        self.pending_reason = reason

    def write_point(self, row: list[str]) -> None:
        self.ensure_open()
        assert self.writer is not None
        self.writer.writerow(row)
        self.rows_written += 1

    def flush(self) -> None:
        if self.file_obj is not None:
            self.file_obj.flush()

    def close(self) -> None:
        if self.file_obj is not None:
            self.file_obj.flush()
            self.file_obj.close()
            self.last_capture_path = self.csv_path
        self.csv_path = None
        self.writer = None
        self.file_obj = None
        self.rows_written = 0

    def snapshot(self) -> dict[str, Any]:
        return {
            "current_capture": str(self.csv_path) if self.csv_path else None,
            "last_capture": str(self.last_capture_path) if self.last_capture_path else None,
            "pending_reason": self.pending_reason,
        }


class RobotTcpHub:
    def __init__(self, host: str, port: int, output_dir: Path) -> None:
        self.host = host
        self.port = port
        self.output_dir = output_dir

        self._lock = threading.Lock()
        self._scan_condition = threading.Condition(self._lock)
        self._stop_event = threading.Event()
        self._server_thread: threading.Thread | None = None
        self._server_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None

        self._recorder = CaptureRecorder(output_dir)
        self._stats = SessionStats()

        self._robot_connected = False
        self._robot_address: str | None = None
        self._robot_protocol: str | None = None
        self._robot_state = "disconnected"
        self._scan_in_progress = False
        self._last_robot_message_monotonic: float | None = None
        self._last_robot_seen_at: str | None = None
        self._current_yaw_deg: float | None = None
        self._current_pitch_deg: float | None = None
        self._sensor_status: str | None = None
        self._sensor_timeout = False
        self._last_sensor_error: str | None = None
        self._last_event = "Waiting for robot connection."
        self._last_error: str | None = None
        self._last_command: str | None = None
        self._last_command_status: str | None = None
        self._last_command_detail: str | None = None
        self._scan_result_sequence = 0
        self._last_scan_result: dict[str, Any] | None = None
        self._live_frame_id: int | None = None
        self._live_points: list[list[float]] = []
        self._live_samples: list[dict[str, float | int]] = []

    def start(self) -> None:
        if self._server_thread is not None:
            return

        self._server_thread = threading.Thread(
            target=self._serve_forever,
            name="robot-tcp-server",
            daemon=True,
        )
        self._server_thread.start()

    def stop(self) -> None:
        self._stop_event.set()

        with self._lock:
            if self._client_socket is not None:
                try:
                    self._client_socket.close()
                except OSError:
                    pass
                self._client_socket = None

            if self._server_socket is not None:
                try:
                    self._server_socket.close()
                except OSError:
                    pass
                self._server_socket = None

        if self._server_thread is not None:
            self._server_thread.join(timeout=2.0)
            self._server_thread = None

        self._recorder.close()

    def send_command(self, command: str) -> None:
        encoded = f"{command}\n".encode("utf-8")

        with self._lock:
            self._expire_stale_client_locked("Robot heartbeat timed out.")
            if self._client_socket is None or not self._robot_connected:
                raise RuntimeError("Robot is not connected.")

            try:
                self._client_socket.sendall(encoded)
            except OSError as exc:
                self._last_error = f"Failed to send command: {exc}"
                raise RuntimeError("Failed to send command to robot.") from exc

            self._last_command = command
            self._last_command_status = "sent"
            self._last_command_detail = None
            self._last_event = f"Sent command {command}"

    def scan_result_token(self) -> int:
        with self._lock:
            return self._scan_result_sequence

    def wait_for_scan_completion(
        self, after_token: int, timeout_seconds: float = SCAN_WAIT_TIMEOUT_SECONDS
    ) -> dict[str, Any]:
        with self._scan_condition:
            deadline = time.monotonic() + timeout_seconds
            while self._scan_result_sequence <= after_token:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("Timed out waiting for scan completion.")
                self._scan_condition.wait(timeout=min(remaining, 0.5))

            if self._last_scan_result is None:
                raise RuntimeError("Scan completed without a result payload.")
            return dict(self._last_scan_result)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._expire_stale_client_locked("Robot heartbeat timed out.")
            status = {
                "robot_connected": self._robot_connected,
                "robot_address": self._robot_address,
                "robot_protocol": self._robot_protocol,
                "robot_state": self._robot_state,
                "robot_connection_state": self._derived_connection_state_locked(),
                "scan_in_progress": self._scan_in_progress,
                "last_robot_seen_at": self._last_robot_seen_at,
                "current_yaw_deg": self._current_yaw_deg,
                "current_pitch_deg": self._current_pitch_deg,
                "sensor_status": self._sensor_status,
                "sensor_timeout": self._sensor_timeout,
                "last_sensor_error": self._last_sensor_error,
                "frames": self._stats.frames,
                "points": self._stats.points,
                "current_frame": self._stats.current_frame,
                "current_frame_points": self._stats.current_frame_points,
                "last_event": self._last_event,
                "last_error": self._last_error,
                "last_command": self._last_command,
                "last_command_status": self._last_command_status,
                "last_command_detail": self._last_command_detail,
                "last_scan_result": dict(self._last_scan_result)
                if self._last_scan_result is not None
                else None,
                "robot_tcp_port": self.port,
                "http_port": HTTP_PORT,
                "detected_lan_ip": detect_lan_ip(),
                "detected_hostname": detect_hostname(),
            }
            status.update(self._recorder.snapshot())
            return status

    def live_points_snapshot(self) -> dict[str, Any]:
        with self._lock:
            self._expire_stale_client_locked("Robot heartbeat timed out.")
            return {
                "frame_id": self._live_frame_id,
                "points": list(self._live_points),
                "samples": [dict(sample) for sample in self._live_samples],
                "render_mode": "raw",
                "scan_in_progress": self._scan_in_progress,
                "robot_state": self._robot_state,
                "robot_connection_state": self._derived_connection_state_locked(),
                "last_robot_seen_at": self._last_robot_seen_at,
                "current_yaw_deg": self._current_yaw_deg,
                "current_pitch_deg": self._current_pitch_deg,
                "sensor_status": self._sensor_status,
                "sensor_timeout": self._sensor_timeout,
            }

    def _serve_forever(self) -> None:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(4)
        server_socket.settimeout(SOCKET_TIMEOUT_SECONDS)

        with self._lock:
            self._server_socket = server_socket
            self._last_event = f"Listening for robot on {self.host}:{self.port}"

        print(f"Robot TCP listener on {self.host}:{self.port}")

        try:
            while not self._stop_event.is_set():
                try:
                    client_socket, address = server_socket.accept()
                except socket.timeout:
                    with self._lock:
                        self._expire_stale_client_locked("Robot heartbeat timed out.")
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    continue

                threading.Thread(
                    target=self._handle_client,
                    args=(client_socket, address),
                    name=f"robot-client-{address[0]}:{address[1]}",
                    daemon=True,
                ).start()
        finally:
            with self._lock:
                self._server_socket = None

            try:
                server_socket.close()
            except OSError:
                pass

    def _handle_client(self, client_socket: socket.socket, address: tuple[str, int]) -> None:
        peer = f"{address[0]}:{address[1]}"
        print(f"Accepted robot connection from {peer}")

        previous_socket: socket.socket | None = None
        with self._lock:
            previous_socket = self._client_socket
            self._client_socket = client_socket
            self._robot_connected = True
            self._robot_address = peer
            self._robot_state = "connected"
            self._scan_in_progress = False
            self._last_error = None
            self._last_event = f"Robot connected from {peer}"
            self._note_robot_activity_locked()

        if previous_socket is not None and previous_socket is not client_socket:
            try:
                previous_socket.close()
            except OSError:
                pass

        client_socket.settimeout(SOCKET_TIMEOUT_SECONDS)
        buffer = ""

        try:
            while not self._stop_event.is_set():
                with self._lock:
                    if self._client_socket is not client_socket:
                        break
                try:
                    chunk = client_socket.recv(4096)
                except socket.timeout:
                    continue
                except OSError:
                    break

                if not chunk:
                    break

                buffer += chunk.decode("utf-8", errors="replace")
                while "\n" in buffer:
                    raw_line, buffer = buffer.split("\n", 1)
                    self._handle_robot_line(raw_line.strip())

            if buffer.strip():
                self._handle_robot_line(buffer.strip())
        finally:
            try:
                client_socket.close()
            except OSError:
                pass

            with self._scan_condition:
                is_active_socket = self._client_socket is client_socket
                scan_was_active = self._scan_in_progress
                current_frame = self._stats.current_frame
                current_points = self._stats.current_frame_points
                capture_path = str(self._recorder.csv_path) if self._recorder.csv_path else None

                if is_active_socket:
                    self._client_socket = None
                    self._robot_connected = False
                    self._robot_address = None
                    self._robot_state = "disconnected"
                    self._scan_in_progress = False
                    self._current_yaw_deg = None
                    self._current_pitch_deg = None
                    self._sensor_status = None
                    self._sensor_timeout = False
                    self._last_sensor_error = None
                    self._stats.current_frame = None
                    self._stats.current_frame_points = 0
                    self._live_frame_id = None
                    self._live_points = []
                    self._live_samples = []
                    self._last_event = "Robot disconnected."

                    if scan_was_active:
                        self._scan_result_sequence += 1
                        self._last_scan_result = {
                            "status": "disconnected",
                            "frame_id": current_frame,
                            "capture_path": capture_path,
                            "point_count": current_points,
                        }
                        self._scan_condition.notify_all()

            if is_active_socket:
                self._recorder.close()
                print("Robot disconnected.")

    def _handle_robot_line(self, line: str) -> None:
        if not line:
            return

        parts = line.split(",")
        message_type = parts[0]

        with self._scan_condition:
            self._note_robot_activity_locked()

            if message_type == "HELLO":
                self._stats.reset()
                self._recorder.mark_run_boundary("HELLO")
                self._robot_protocol = ",".join(parts[1:]) if len(parts) > 1 else None
                self._robot_state = "idle"
                self._scan_in_progress = False
                self._sensor_status = "READY"
                self._sensor_timeout = False
                self._last_sensor_error = None
                self._last_event = f"Robot announced: {line}"
                return

            if message_type == "RESET":
                self._stats.reset()
                self._recorder.mark_run_boundary("RESET")
                self._sensor_timeout = False
                self._last_sensor_error = None
                self._last_event = f"Run boundary: {line}"
                if len(parts) > 1:
                    try:
                        self._stats.current_frame = int(parts[1])
                        self._live_frame_id = self._stats.current_frame
                    except ValueError:
                        self._stats.current_frame = None
                        self._live_frame_id = None
                self._live_points = []
                self._live_samples = []
                return

            if message_type == "STATE" and len(parts) >= 2:
                state = parts[1].strip().lower()
                self._robot_state = state
                self._scan_in_progress = state in {
                    "scanning",
                    "stopping",
                    "hard_stopping",
                    "releasing",
                }
                self._last_event = f"Robot state: {state}"
                return

            if message_type == "POSE" and len(parts) >= 3:
                try:
                    self._current_yaw_deg = float(parts[1])
                    self._current_pitch_deg = float(parts[2])
                except ValueError:
                    self._current_yaw_deg = None
                    self._current_pitch_deg = None
                return

            if message_type == "SENSOR_STATUS" and len(parts) >= 2:
                self._sensor_status = ",".join(parts[1:])
                if self._sensor_status == "READY":
                    self._sensor_timeout = False
                    self._last_sensor_error = None
                if self._sensor_status == "RECOVERING_SENSOR":
                    self._last_event = "Sensor recovery in progress."
                return

            if message_type == "HEARTBEAT":
                if len(parts) >= 3:
                    self._robot_state = parts[2].strip().lower()
                    self._scan_in_progress = self._robot_state in {
                        "scanning",
                        "stopping",
                        "hard_stopping",
                        "releasing",
                    }
                if len(parts) >= 5:
                    try:
                        self._current_yaw_deg = float(parts[3])
                        self._current_pitch_deg = float(parts[4])
                    except ValueError:
                        pass
                self._last_event = f"Heartbeat from robot at {parts[1] if len(parts) > 1 else '?'}"
                return

            if message_type == "COMMAND_ACK" and len(parts) >= 2:
                self._last_command_status = "ack"
                self._last_command_detail = ",".join(parts[1:])
                self._last_event = f"Command ack: {self._last_command_detail}"
                return

            if message_type == "COMMAND_FAIL" and len(parts) >= 2:
                self._last_command_status = "fail"
                self._last_command_detail = ",".join(parts[1:])
                self._last_error = f"Command rejected: {self._last_command_detail}"
                self._last_event = self._last_error
                return

            if message_type == "FRAME_BEGIN" and len(parts) >= 3:
                self._recorder.ensure_open()
                self._stats.frames += 1
                self._stats.current_frame_points = 0
                self._scan_in_progress = True
                self._sensor_status = "SCANNING"
                self._sensor_timeout = False
                self._last_sensor_error = None
                self._robot_state = "scanning"
                try:
                    self._stats.current_frame = int(parts[1])
                    self._live_frame_id = self._stats.current_frame
                except ValueError:
                    self._stats.current_frame = None
                    self._live_frame_id = None
                self._live_points = []
                self._live_samples = []
                self._last_event = f"Frame {parts[1]} started at device millis={parts[2]}"
                return

            if message_type == "FRAME_END" and len(parts) >= 3:
                self._recorder.flush()
                self._scan_in_progress = False
                self._robot_state = "idle"
                if self._sensor_status == "SCANNING":
                    self._sensor_status = "READY"
                self._last_event = f"Frame {parts[1]} finished with {parts[2]} points"
                self._scan_result_sequence += 1
                self._last_scan_result = {
                    "status": "completed",
                    "frame_id": self._stats.current_frame,
                    "capture_path": str(self._recorder.csv_path)
                    if self._recorder.csv_path
                    else None,
                    "point_count": self._stats.current_frame_points,
                }
                self._scan_condition.notify_all()
                return

            if message_type == "POINT" and len(parts) == 9:
                self._recorder.write_point(parts[1:])
                self._stats.points += 1
                self._stats.current_frame_points += 1
                try:
                    frame_id = int(parts[1])
                    point_index = int(parts[2])
                    yaw_deg = float(parts[3])
                    pitch_deg = float(parts[4])
                    distance_mm = float(parts[5])
                    x_mm = float(parts[6])
                    y_mm = float(parts[7])
                    z_mm = float(parts[8])
                except ValueError:
                    frame_id = 0
                    point_index = 0
                    yaw_deg = 0.0
                    pitch_deg = 0.0
                    distance_mm = 0.0
                    x_mm = 0.0
                    y_mm = 0.0
                    z_mm = 0.0

                self._live_points.append([x_mm, y_mm, z_mm, distance_mm])
                self._live_samples.append(
                    {
                        "frame_id": frame_id,
                        "point_index": point_index,
                        "yaw_deg": yaw_deg,
                        "pitch_deg": pitch_deg,
                        "distance_mm": distance_mm,
                        "x_mm": x_mm,
                        "y_mm": y_mm,
                        "z_mm": z_mm,
                    }
                )
                if self._stats.current_frame_points % 25 == 0:
                    self._recorder.flush()
                return

            if message_type == "SENSOR_TIMEOUT":
                self._sensor_timeout = True
                self._sensor_status = "SENSOR_TIMEOUT"
                self._last_sensor_error = (
                    ",".join(parts[1:]) if len(parts) > 1 else "SENSOR_TIMEOUT"
                )
                self._last_event = f"Sensor timeout: {self._last_sensor_error}"
                return

            if message_type == "ABORT":
                self._scan_in_progress = False
                self._robot_state = "idle"
                if self._sensor_status == "SCANNING":
                    self._sensor_status = "ABORTED"
                self._last_event = f"Scan aborted: {line}"
                frame_id = None
                if len(parts) > 1:
                    try:
                        frame_id = int(parts[1])
                    except ValueError:
                        frame_id = self._stats.current_frame
                self._scan_result_sequence += 1
                self._last_scan_result = {
                    "status": "aborted",
                    "frame_id": frame_id,
                    "capture_path": str(self._recorder.csv_path)
                    if self._recorder.csv_path
                    else None,
                    "point_count": self._stats.current_frame_points,
                }
                self._scan_condition.notify_all()
                return

            self._last_event = f"Ignored message: {line}"

    def _note_robot_activity_locked(self) -> None:
        self._last_robot_message_monotonic = time.monotonic()
        self._last_robot_seen_at = datetime.now().isoformat(timespec="seconds")

    def _is_robot_stale_locked(self) -> bool:
        if not self._robot_connected or self._last_robot_message_monotonic is None:
            return False
        return (
            time.monotonic() - self._last_robot_message_monotonic
        ) > ROBOT_STALE_TIMEOUT_SECONDS

    def _derived_connection_state_locked(self) -> str:
        if not self._robot_connected:
            return "disconnected"
        if self._is_robot_stale_locked():
            return "stale"
        if self._sensor_status == "RECOVERING_SENSOR":
            return "recovering_sensor"
        if self._scan_in_progress:
            return "scanning"
        return "idle"

    def _expire_stale_client_locked(self, reason: str) -> None:
        if not self._is_robot_stale_locked():
            return

        stale_socket = self._client_socket
        scan_was_active = self._scan_in_progress
        current_frame = self._stats.current_frame
        current_points = self._stats.current_frame_points
        capture_path = str(self._recorder.csv_path) if self._recorder.csv_path else None
        self._client_socket = None
        self._robot_connected = False
        self._robot_address = None
        self._robot_state = "disconnected"
        self._scan_in_progress = False
        self._last_error = reason
        self._last_event = reason
        self._sensor_status = "STALE_LINK"
        self._stats.current_frame = None
        self._stats.current_frame_points = 0

        if scan_was_active:
            self._scan_result_sequence += 1
            self._last_scan_result = {
                "status": "disconnected",
                "frame_id": current_frame,
                "capture_path": capture_path,
                "point_count": current_points,
            }
            self._scan_condition.notify_all()

        self._recorder.close()

        if stale_socket is not None:
            try:
                stale_socket.close()
            except OSError:
                pass


class ScanAutomationController:
    def __init__(
        self,
        robot_hub: RobotTcpHub,
        capture_dir: Path,
        baseline_path: Path,
        baseline_meta_path: Path,
    ) -> None:
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
                        self._baseline_degrees = normalize_scan_degrees(
                            float(baseline_degrees)
                        )
                    except (TypeError, ValueError):
                        self._baseline_degrees = None
                self._baseline_saved_at = meta.get("saved_at")
                self._baseline_source_capture = meta.get("source_capture")

            if self._baseline_exists:
                try:
                    self._baseline_point_map = load_pose_point_map(self.baseline_path)
                    self._baseline_surface_samples = build_surface_samples(
                        self._baseline_point_map,
                        fill_air=True,
                    )
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
            }

    def set_residual_tolerance(self, tolerance_mm: float) -> float:
        normalized = max(0.0, float(tolerance_mm))
        with self._lock:
            self._residual_tolerance_mm = normalized
        return normalized

    def build_live_plot_payload(
        self,
        raw_payload: dict[str, Any],
        mode: str = "auto",
        tolerance_mm: float | None = None,
    ) -> dict[str, Any]:
        with self._lock:
            baseline_available = (
                self._baseline_exists
                and self._baseline_point_map is not None
                and self._baseline_surface_samples is not None
                and self._pending_operation != "saving_baseline"
            )
            baseline_point_map = self._baseline_point_map
            baseline_surface_samples = (
                [dict(sample) for sample in self._baseline_surface_samples]
                if self._baseline_surface_samples is not None
                else None
            )
            configured_tolerance_mm = self._residual_tolerance_mm

        normalized_mode = mode.strip().lower() if mode else "auto"
        if normalized_mode not in {"auto", "raw", "residual", "baseline"}:
            normalized_mode = "auto"
        normalized_tolerance_mm = (
            configured_tolerance_mm
            if tolerance_mm is None
            else self.set_residual_tolerance(tolerance_mm)
        )

        raw_point_map = point_map_from_samples(raw_payload.get("samples", []))
        raw_surface_samples = build_surface_samples(
            raw_point_map, frame_id=raw_payload.get("frame_id"), fill_air=True
        )

        if (
            normalized_mode == "baseline"
            and baseline_available
            and baseline_surface_samples is not None
        ):
            payload = dict(raw_payload)
            payload["points"] = points_from_samples(baseline_surface_samples)
            payload["samples"] = baseline_surface_samples
            payload["render_mode"] = "baseline"
            payload["requested_mode"] = normalized_mode
            payload["baseline_available"] = baseline_available
            payload["residual_tolerance_mm"] = normalized_tolerance_mm
            payload["matched_points"] = len(baseline_surface_samples)
            return payload

        if normalized_mode == "raw" or not baseline_available or baseline_point_map is None:
            payload = dict(raw_payload)
            payload["points"] = points_from_samples(raw_surface_samples)
            payload["samples"] = raw_surface_samples
            payload["render_mode"] = "raw"
            payload["requested_mode"] = normalized_mode
            payload["baseline_available"] = baseline_available
            payload["residual_tolerance_mm"] = normalized_tolerance_mm
            payload["matched_points"] = len(raw_surface_samples)
            return payload

        current_surface_samples = [dict(sample) for sample in raw_payload.get("samples", [])]
        if not current_surface_samples:
            payload = dict(raw_payload)
            payload["points"] = []
            payload["samples"] = []
            payload["render_mode"] = "residual"
            payload["requested_mode"] = normalized_mode
            payload["baseline_available"] = baseline_available
            payload["residual_tolerance_mm"] = normalized_tolerance_mm
            payload["matched_points"] = 0
            return payload
        target_poses = [
            (float(sample["yaw_deg"]), float(sample["pitch_deg"]))
            for sample in current_surface_samples
        ]
        baseline_surface_samples = build_surface_samples(
            baseline_point_map,
            frame_id=raw_payload.get("frame_id"),
            target_poses=target_poses,
            fill_air=True,
        )
        baseline_surface_map = point_map_from_samples(baseline_surface_samples)
        residual_points: list[list[float]] = []
        residual_samples: list[dict[str, float | int]] = []
        for current_sample in current_surface_samples:
            yaw_deg = float(current_sample["yaw_deg"])
            pitch_deg = float(current_sample["pitch_deg"])
            baseline_sample = baseline_surface_map.get(pose_key(yaw_deg, pitch_deg))
            if baseline_sample is None:
                continue

            baseline_distance_mm = float(baseline_sample["distance_mm"])
            current_distance_mm = float(current_sample["distance_mm"])
            occlusion_distance_mm = baseline_distance_mm - current_distance_mm
            if current_distance_mm >= baseline_distance_mm:
                continue
            if occlusion_distance_mm < normalized_tolerance_mm:
                continue

            residual_points.append(
                [
                    float(current_sample["x_mm"]),
                    float(current_sample["y_mm"]),
                    float(current_sample["z_mm"]),
                    current_distance_mm,
                    0.0,
                    occlusion_distance_mm,
                ]
            )
            residual_samples.append(
                {
                    "frame_id": int(current_sample["frame_id"]),
                    "point_index": int(current_sample["point_index"]),
                    "yaw_deg": yaw_deg,
                    "pitch_deg": pitch_deg,
                    "distance_mm": current_distance_mm,
                    "x_mm": float(current_sample["x_mm"]),
                    "y_mm": float(current_sample["y_mm"]),
                    "z_mm": float(current_sample["z_mm"]),
                    "is_air": False,
                    "occlusion_distance_mm": occlusion_distance_mm,
                    "current_x_mm": float(current_sample["x_mm"]),
                    "current_y_mm": float(current_sample["y_mm"]),
                    "current_z_mm": float(current_sample["z_mm"]),
                    "baseline_x_mm": float(baseline_sample["x_mm"]),
                    "baseline_y_mm": float(baseline_sample["y_mm"]),
                    "baseline_z_mm": float(baseline_sample["z_mm"]),
                    "residual_x_mm": float(current_sample["x_mm"]) - float(baseline_sample["x_mm"]),
                    "residual_y_mm": float(current_sample["y_mm"]) - float(baseline_sample["y_mm"]),
                    "residual_z_mm": float(current_sample["z_mm"]) - float(baseline_sample["z_mm"]),
                    "baseline_distance_mm": baseline_distance_mm,
                    "current_distance_mm": current_distance_mm,
                }
            )

        payload = dict(raw_payload)
        payload["points"] = residual_points
        payload["samples"] = residual_samples
        payload["render_mode"] = "residual"
        payload["requested_mode"] = normalized_mode
        payload["baseline_available"] = baseline_available
        payload["residual_tolerance_mm"] = normalized_tolerance_mm
        payload["matched_points"] = len(residual_points)
        return payload

    def set_configured_scan_degrees(self, scan_degrees: float) -> float:
        normalized = normalize_scan_degrees(scan_degrees)
        with self._lock:
            self._configured_scan_degrees = normalized
        return normalized

    def _start_scan_and_wait(self, scan_degrees: float) -> dict[str, Any]:
        token = self.robot_hub.scan_result_token()
        self.robot_hub.send_command(format_scan_command(scan_degrees))
        return self.robot_hub.wait_for_scan_completion(token)

    def start_baseline_save(self, scan_degrees: float) -> None:
        normalized = self.set_configured_scan_degrees(scan_degrees)
        with self._lock:
            if self._pending_operation is not None:
                raise RuntimeError("Another automation operation is already running.")
            if self._monitoring_active:
                raise RuntimeError("Stop monitoring before saving a baseline.")

            self._pending_operation = "saving_baseline"
            self._automation_error = None
            self._automation_event = (
                f"Starting baseline capture with {normalized:.2f} degree sweep."
            )
            self._operation_thread = threading.Thread(
                target=self._baseline_worker,
                args=(normalized,),
                name="baseline-save-worker",
                daemon=True,
            )
            self._operation_thread.start()

    def _baseline_worker(self, scan_degrees: float) -> None:
        try:
            result = self._start_scan_and_wait(scan_degrees)
            if result.get("status") != "completed":
                raise RuntimeError(
                    f"Baseline scan did not complete: {result.get('status')}"
                )

            capture_path_raw = result.get("capture_path")
            if not capture_path_raw:
                raise RuntimeError("Baseline scan completed without a capture file.")

            source_path = Path(capture_path_raw)
            self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source_path, self.baseline_path)
            saved_at = datetime.now().isoformat(timespec="seconds")
            write_json(
                self.baseline_meta_path,
                {
                    "saved_at": saved_at,
                    "source_capture": str(source_path),
                    "scan_degrees": scan_degrees,
                },
            )

            with self._lock:
                self._baseline_exists = True
                self._baseline_degrees = scan_degrees
                self._baseline_saved_at = saved_at
                self._baseline_source_capture = str(source_path)
                self._baseline_point_map = load_pose_point_map(self.baseline_path)
                self._baseline_surface_samples = build_surface_samples(
                    self._baseline_point_map,
                    fill_air=True,
                )
                self._automation_error = None
                self._automation_event = (
                    f"Saved baseline from {source_path.name} at {scan_degrees:.2f} degrees."
                )
        except Exception as exc:
            with self._lock:
                self._automation_error = str(exc)
                self._automation_event = "Baseline capture failed."
        finally:
            with self._lock:
                self._pending_operation = None
                self._operation_thread = None

    def import_baseline_from_view(
        self,
        view_payload: dict[str, Any],
        scan_degrees: float | None = None,
        source_name: str | None = None,
    ) -> None:
        normalized_view = normalize_bundle_view_payload(view_payload)
        if normalized_view is None:
            raise RuntimeError("Imported baseline payload did not contain valid samples.")

        normalized_samples = normalized_view["samples"]
        resolved_scan_degrees = scan_degrees
        if resolved_scan_degrees is None:
            inferred_scan_degrees = infer_scan_degrees_from_samples(normalized_samples)
            if inferred_scan_degrees is not None:
                resolved_scan_degrees = inferred_scan_degrees
            else:
                resolved_scan_degrees = DEFAULT_SCAN_DEGREES
        resolved_scan_degrees = normalize_scan_degrees(float(resolved_scan_degrees))

        with self._lock:
            if self._pending_operation is not None:
                raise RuntimeError("Wait for the current automation operation to finish.")
            if self._monitoring_active:
                raise RuntimeError("Stop monitoring before importing a baseline.")

        self.baseline_path.parent.mkdir(parents=True, exist_ok=True)
        with self.baseline_path.open("w", newline="", encoding="utf-8") as file_obj:
            writer = csv.writer(file_obj)
            writer.writerow(
                [
                    "frame_id",
                    "point_index",
                    "yaw_deg",
                    "pitch_deg",
                    "distance_mm",
                    "x_mm",
                    "y_mm",
                    "z_mm",
                ]
            )
            for sample in normalized_samples:
                writer.writerow(
                    [
                        int(sample["frame_id"]),
                        int(sample["point_index"]),
                        format_optional_float(float(sample["yaw_deg"])),
                        format_optional_float(float(sample["pitch_deg"])),
                        format_optional_float(float(sample["distance_mm"])),
                        format_optional_float(float(sample["x_mm"])),
                        format_optional_float(float(sample["y_mm"])),
                        format_optional_float(float(sample["z_mm"])),
                    ]
                )

        saved_at = datetime.now().isoformat(timespec="seconds")
        source_capture = source_name or "imported_bundle"
        write_json(
            self.baseline_meta_path,
            {
                "saved_at": saved_at,
                "source_capture": source_capture,
                "scan_degrees": resolved_scan_degrees,
            },
        )

        with self._lock:
            self._baseline_exists = True
            self._baseline_degrees = resolved_scan_degrees
            self._baseline_saved_at = saved_at
            self._baseline_source_capture = source_capture
            self._baseline_point_map = load_pose_point_map(self.baseline_path)
            self._baseline_surface_samples = build_surface_samples(
                self._baseline_point_map,
                fill_air=True,
            )
            self._automation_error = None
            self._automation_event = (
                f"Imported baseline from {source_capture} at {resolved_scan_degrees:.2f} degrees."
            )

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

    def start_monitoring(self, scan_degrees: float) -> None:
        normalized = self.set_configured_scan_degrees(scan_degrees)
        with self._lock:
            if not self._baseline_exists:
                raise RuntimeError("Save a baseline before starting monitoring.")
            if self._pending_operation is not None:
                raise RuntimeError("Another automation operation is already running.")
            if self._monitoring_active:
                raise RuntimeError("Monitoring is already active.")

            self._monitoring_active = True
            self._monitor_stop_event.clear()
            self._next_monitor_scan_at = None
            self._automation_error = None
            self._automation_event = (
                f"Monitoring started with {normalized:.2f} degree sweeps."
            )
            self._monitor_thread = threading.Thread(
                target=self._monitoring_worker,
                args=(normalized,),
                name="monitoring-worker",
                daemon=True,
            )
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
                    self._automation_event = (
                        f"Starting monitoring scan at {scan_degrees:.2f} degrees."
                    )
                    self._automation_error = None

                try:
                    result = self._start_scan_and_wait(scan_degrees)
                    if result.get("status") != "completed":
                        raise RuntimeError(
                            f"Monitoring scan did not complete: {result.get('status')}"
                        )

                    capture_path_raw = result.get("capture_path")
                    if not capture_path_raw:
                        raise RuntimeError("Monitoring scan completed without a capture file.")

                    capture_path = Path(capture_path_raw)
                    with self._lock:
                        residual_tolerance_mm = self._residual_tolerance_mm
                    residual_path, summary, matched_points = write_residual_capture(
                        self.capture_dir,
                        self.baseline_path,
                        capture_path,
                        residual_tolerance_mm,
                    )

                    with self._lock:
                        self._last_monitor_capture = str(capture_path)
                        self._last_residual_path = str(residual_path)
                        self._last_residual_summary = summary
                        self._last_residual_points = matched_points
                        self._automation_event = (
                            f"Monitoring scan complete. Residual saved to {residual_path.name}."
                        )
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


robot_hub = RobotTcpHub(ROBOT_HOST, ROBOT_PORT, CAPTURE_DIR)
automation = ScanAutomationController(
    robot_hub, CAPTURE_DIR, BASELINE_PATH, BASELINE_META_PATH
)


def combined_status() -> dict[str, Any]:
    status = robot_hub.snapshot()
    status.update(automation.snapshot())
    return status


async def read_scan_degrees(request: Request) -> float:
    try:
        payload = await request.json()
    except Exception:
        payload = {}

    raw_value = payload.get("scan_degrees", DEFAULT_SCAN_DEGREES)
    try:
        return normalize_scan_degrees(float(raw_value))
    except (TypeError, ValueError) as exc:
        raise HTTPException(
            status_code=400,
            detail=f"scan_degrees must be between 0 and {MAX_SCAN_DEGREES:.0f}.",
        ) from exc


async def read_baseline_import(request: Request) -> tuple[dict[str, Any], float | None, str | None]:
    try:
        payload = await request.json()
    except Exception as exc:
        raise HTTPException(status_code=400, detail="Invalid JSON payload.") from exc

    source_view = str(payload.get("source_view", "baseline")).strip().lower()
    if source_view not in {"baseline", "raw"}:
        raise HTTPException(status_code=400, detail="source_view must be baseline or raw.")

    bundle = payload.get("bundle")
    view_payload: dict[str, Any] | None = None
    source_name = payload.get("source_name")
    scan_degrees: float | None = None

    if isinstance(bundle, dict):
        bundle_views = bundle.get("views")
        if not isinstance(bundle_views, dict):
            raise HTTPException(status_code=400, detail="Bundle did not contain views.")
        preferred_payload = bundle_views.get(source_view)
        if not isinstance(preferred_payload, dict) and source_view == "baseline":
            preferred_payload = bundle_views.get("raw")
        if not isinstance(preferred_payload, dict):
            raise HTTPException(
                status_code=400,
                detail="Bundle did not contain an importable baseline or raw view.",
            )
        view_payload = preferred_payload
        if not source_name:
            source_name = str(payload.get("source_name") or bundle.get("imported_name") or "imported_bundle")

        bundle_status = bundle.get("status")
        if isinstance(bundle_status, dict):
            candidate_scan_degrees = bundle_status.get("baseline_degrees")
            if candidate_scan_degrees is None:
                candidate_scan_degrees = bundle_status.get("configured_scan_degrees")
            if candidate_scan_degrees is not None:
                try:
                    scan_degrees = normalize_scan_degrees(float(candidate_scan_degrees))
                except (TypeError, ValueError):
                    scan_degrees = None
    else:
        direct_view = payload.get("view")
        if not isinstance(direct_view, dict):
            raise HTTPException(status_code=400, detail="Missing importable view payload.")
        view_payload = direct_view
        if not source_name:
            source_name = "imported_view"

    raw_scan_degrees = payload.get("scan_degrees")
    if raw_scan_degrees is not None:
        try:
            scan_degrees = normalize_scan_degrees(float(raw_scan_degrees))
        except (TypeError, ValueError) as exc:
            raise HTTPException(
                status_code=400,
                detail=f"scan_degrees must be between 0 and {MAX_SCAN_DEGREES:.0f}.",
            ) from exc

    assert view_payload is not None
    return view_payload, scan_degrees, source_name


if FastAPI is not None:
    app = FastAPI(title="WARD Point Cloud Control")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(INDEX_HTML)

    @app.get("/api/status")
    async def api_status() -> dict[str, Any]:
        return combined_status()

    @app.get("/api/live-points")
    async def api_live_points(
        mode: str = "auto", tolerance_mm: float = DEFAULT_RESIDUAL_TOLERANCE_MM
    ) -> dict[str, Any]:
        return automation.build_live_plot_payload(
            robot_hub.live_points_snapshot(), mode, tolerance_mm
        )

    @app.post("/api/robot/start-scan")
    async def api_start_scan(request: Request) -> dict[str, Any]:
        scan_degrees = await read_scan_degrees(request)
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")
        if status["monitoring_active"]:
            raise HTTPException(status_code=409, detail="Stop monitoring before manual scans.")
        if status["pending_operation"] is not None:
            raise HTTPException(
                status_code=409,
                detail="Wait for the current automation operation to finish.",
            )

        try:
            automation.set_configured_scan_degrees(scan_degrees)
            robot_hub.send_command(format_scan_command(scan_degrees))
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/robot/stop-scan")
    async def api_stop_scan() -> dict[str, Any]:
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")

        if status["monitoring_active"]:
            automation.stop_monitoring(request_robot_stop=False)

        if status["scan_in_progress"]:
            try:
                robot_hub.send_command("STOP_SCAN")
            except RuntimeError as exc:
                raise HTTPException(status_code=500, detail=str(exc)) from exc
        elif not status["monitoring_active"]:
            raise HTTPException(status_code=409, detail="Robot is not scanning.")

        return combined_status()

    @app.post("/api/robot/hard-stop")
    async def api_hard_stop() -> dict[str, Any]:
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if not status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is not scanning.")

        automation.stop_monitoring(request_robot_stop=False)
        try:
            robot_hub.send_command("HARD_STOP")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/robot/release-motors")
    async def api_release_motors() -> dict[str, Any]:
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")

        automation.stop_monitoring(request_robot_stop=False)
        try:
            robot_hub.send_command("RELEASE_MOTORS")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/robot/zero-turret")
    async def api_zero_turret() -> dict[str, Any]:
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is currently scanning.")
        if status["monitoring_active"]:
            raise HTTPException(status_code=409, detail="Stop monitoring before zeroing.")
        if status["pending_operation"] is not None:
            raise HTTPException(
                status_code=409,
                detail="Wait for the current automation operation to finish.",
            )

        try:
            robot_hub.send_command("ZERO_TURRET")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/baseline/save")
    async def api_save_baseline(request: Request) -> dict[str, Any]:
        scan_degrees = await read_scan_degrees(request)
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")

        try:
            automation.start_baseline_save(scan_degrees)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/baseline/import")
    async def api_import_baseline(request: Request) -> dict[str, Any]:
        view_payload, scan_degrees, source_name = await read_baseline_import(request)
        try:
            automation.import_baseline_from_view(
                view_payload,
                scan_degrees=scan_degrees,
                source_name=source_name,
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status()

    @app.delete("/api/baseline")
    async def api_delete_baseline() -> dict[str, Any]:
        try:
            automation.delete_baseline()
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/monitoring/start")
    async def api_start_monitoring(request: Request) -> dict[str, Any]:
        scan_degrees = await read_scan_degrees(request)
        status = combined_status()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")

        try:
            automation.start_monitoring(scan_degrees)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status()

    @app.post("/api/monitoring/stop")
    async def api_stop_monitoring() -> dict[str, Any]:
        automation.stop_monitoring()
        return combined_status()

else:
    app = None


def main() -> None:
    if IMPORT_ERROR is not None:
        print("FastAPI server dependencies are missing.")
        print("Install them with:")
        print("  pip install fastapi uvicorn")
        raise SystemExit(1)

    assert uvicorn is not None
    robot_hub.start()
    try:
        uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT)
    finally:
        robot_hub.stop()


if __name__ == "__main__":
    main()
