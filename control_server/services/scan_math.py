from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

from .common import format_optional_float
from .config import (
    AIR_DISTANCE_MM,
    DEFAULT_PITCH_STEP_DEG,
    DEFAULT_SCAN_DEGREES,
    DEFAULT_YAW_STEP_DEG,
    INTERPOLATION_MAX_NEIGHBORS,
    INTERPOLATION_NEIGHBOR_RADIUS,
    MAX_SCAN_DEGREES,
)


def normalize_scan_degrees(scan_degrees: float) -> float:
    normalized = float(scan_degrees)
    if normalized <= 0.0 or normalized > MAX_SCAN_DEGREES:
        raise ValueError(
            f"scan_degrees must be between 0 and {MAX_SCAN_DEGREES:.0f}."
        )
    return round(normalized, 2)


def format_scan_command(scan_degrees: float) -> str:
    return f"START_SCAN,{normalize_scan_degrees(scan_degrees):.2f}"


def parse_number(value: str | None) -> float:
    if value is None:
        raise ValueError("Missing numeric value.")
    return float(value)


def pose_key(yaw_deg: float, pitch_deg: float) -> tuple[int, int]:
    return (int(round(yaw_deg * 100.0)), int(round(pitch_deg * 100.0)))


def spherical_to_cartesian_mm(
    distance_mm: float, yaw_deg: float, pitch_deg: float
) -> tuple[float, float, float]:
    yaw_rad = math.radians(yaw_deg)
    pitch_rad = math.radians(pitch_deg)
    cos_pitch = math.cos(pitch_rad)
    return (
        distance_mm * cos_pitch * math.cos(yaw_rad),
        distance_mm * cos_pitch * math.sin(yaw_rad),
        distance_mm * math.sin(pitch_rad),
    )


def load_pose_point_map(csv_path: Path) -> dict[tuple[int, int], dict[str, float | int]]:
    if not csv_path.exists():
        raise FileNotFoundError(f"Capture file not found: {csv_path}")

    point_map: dict[tuple[int, int], dict[str, float | int]] = {}
    with csv_path.open("r", newline="", encoding="utf-8") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            try:
                yaw_deg = parse_number(row["yaw_deg"])
                pitch_deg = parse_number(row["pitch_deg"])
                point_map[pose_key(yaw_deg, pitch_deg)] = {
                    "frame_id": int(float(row["frame_id"])),
                    "point_index": int(row["point_index"]),
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
            yaw_deg = float(row["yaw_deg"])
            pitch_deg = float(row["pitch_deg"])
            point_map[pose_key(yaw_deg, pitch_deg)] = {
                "frame_id": int(row["frame_id"]),
                "point_index": int(row["point_index"]),
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


def infer_scan_degrees_from_samples(samples: list[dict[str, float | int | bool]]) -> float | None:
    yaw_values = sorted({round(float(sample["yaw_deg"]), 2) for sample in samples})
    if not yaw_values:
        return None
    inferred = max(1.0, yaw_values[-1] - yaw_values[0])
    try:
        return normalize_scan_degrees(inferred)
    except ValueError:
        return None


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


def infer_axis_grid(
    point_map: dict[tuple[int, int], dict[str, float | int]],
    axis_name: str,
    fallback_step: float,
) -> tuple[list[float], float]:
    values = sorted({round(float(sample[axis_name]), 2) for sample in point_map.values()})
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
    pitch_values, pitch_step = infer_axis_grid(
        point_map, "pitch_deg", DEFAULT_PITCH_STEP_DEG
    )
    if target_poses is None:
        target_poses = [(yaw_deg, pitch_deg) for yaw_deg in yaw_values for pitch_deg in pitch_values]
    else:
        target_poses = sorted(
            {(round(float(yaw_deg), 2), round(float(pitch_deg), 2)) for yaw_deg, pitch_deg in target_poses}
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
