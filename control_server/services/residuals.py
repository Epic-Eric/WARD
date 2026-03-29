from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

from .capture import open_capture_file
from .common import format_optional_float
from .config import (
    CLUSTER_DBSCAN_MIN_NEIGHBORS,
    CLUSTER_HASH_CELL_MM,
    CLUSTER_MAX_PITCH_GAP_DEG,
    CLUSTER_MAX_YAW_GAP_DEG,
    CLUSTER_MIN_LOCAL_SUPPORT,
    CLUSTER_OCCLUSION_DELTA_MM,
    CLUSTER_RADIUS_DISTANCE_SCALE,
    CLUSTER_RADIUS_MAX_MM,
    CLUSTER_RADIUS_MIN_MM,
    CLUSTER_SPLIT_MAX_PITCH_GAP_DEG,
    CLUSTER_SPLIT_MAX_RADIUS_MM,
    CLUSTER_SPLIT_MAX_YAW_GAP_DEG,
    CLUSTER_SPLIT_MIN_POINTS,
    CLUSTER_SPLIT_OCCLUSION_DELTA_MM,
    CLUSTER_SPLIT_RADIUS_SCALE,
    DEBRIS_MAX_DIAGONAL_MM,
    DEBRIS_MIN_MAX_OCCLUSION_MM,
    DEBRIS_MIN_MEAN_OCCLUSION_MM,
    DEBRIS_MIN_OCCUPANCY_RATIO,
    DEBRIS_MIN_POINTS,
    DEBRIS_MIN_UNIQUE_PITCHES,
    DEBRIS_MIN_UNIQUE_YAWS,
    DEFAULT_RESIDUAL_TOLERANCE_MM,
)
from .scan_math import (
    build_surface_samples,
    load_pose_point_map,
    point_map_from_samples,
    pose_key,
)


def write_residual_capture(
    output_dir: Path,
    baseline_path: Path,
    current_path: Path,
    tolerance_mm: float = DEFAULT_RESIDUAL_TOLERANCE_MM,
) -> tuple[Path, str, int, list[dict[str, Any]]]:
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
    collected_residuals: list[dict[str, float | int]] = []
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
            collected_residuals.append({
                "frame_id": int(current["frame_id"]),
                "point_index": int(current["point_index"]),
                "yaw_deg": float(current["yaw_deg"]),
                "pitch_deg": float(current["pitch_deg"]),
                "distance_mm": current_distance_mm,
                "x_mm": float(current["x_mm"]),
                "y_mm": float(current["y_mm"]),
                "z_mm": float(current["z_mm"]),
                "occlusion_distance_mm": occlusion_distance_mm,
            })
            written_points += 1
    finally:
        file_obj.flush()
        file_obj.close()

    debris_clusters, _ = cluster_residual_samples(collected_residuals, tolerance_mm)
    summary = f"Residual saved with {written_points} foreground rays from the current scan."
    return residual_path, summary, written_points, debris_clusters


def cluster_radius_mm(sample: dict[str, float | int]) -> float:
    distance_mm = float(sample.get("distance_mm", 0.0))
    return max(
        CLUSTER_RADIUS_MIN_MM,
        min(CLUSTER_RADIUS_MAX_MM, distance_mm * CLUSTER_RADIUS_DISTANCE_SCALE),
    )


def spatial_hash_cell(x_mm: float, y_mm: float, z_mm: float) -> tuple[int, int, int]:
    return (
        int(math.floor(x_mm / CLUSTER_HASH_CELL_MM)),
        int(math.floor(y_mm / CLUSTER_HASH_CELL_MM)),
        int(math.floor(z_mm / CLUSTER_HASH_CELL_MM)),
    )


def pair_is_cluster_compatible(
    left: dict[str, float | int],
    right: dict[str, float | int],
    pair_radius_mm: float,
    *,
    max_yaw_gap_deg: float = CLUSTER_MAX_YAW_GAP_DEG,
    max_pitch_gap_deg: float = CLUSTER_MAX_PITCH_GAP_DEG,
    max_occlusion_delta_mm: float = CLUSTER_OCCLUSION_DELTA_MM,
) -> bool:
    delta_x = float(right["x_mm"]) - float(left["x_mm"])
    delta_y = float(right["y_mm"]) - float(left["y_mm"])
    delta_z = float(right["z_mm"]) - float(left["z_mm"])
    if (delta_x * delta_x + delta_y * delta_y + delta_z * delta_z) > pair_radius_mm * pair_radius_mm:
        return False

    if abs(float(right["yaw_deg"]) - float(left["yaw_deg"])) > max_yaw_gap_deg:
        return False
    if abs(float(right["pitch_deg"]) - float(left["pitch_deg"])) > max_pitch_gap_deg:
        return False

    left_occlusion_mm = float(left.get("occlusion_distance_mm", 0.0))
    right_occlusion_mm = float(right.get("occlusion_distance_mm", 0.0))
    if abs(right_occlusion_mm - left_occlusion_mm) > max_occlusion_delta_mm:
        return False

    return True


def split_member_indexes(
    residual_samples: list[dict[str, float | int]],
    member_indexes: list[int],
) -> list[list[int]]:
    if len(member_indexes) < max(CLUSTER_SPLIT_MIN_POINTS, 2):
        return [member_indexes]

    member_set = set(member_indexes)
    components: list[list[int]] = []
    visited: set[int] = set()

    for index in member_indexes:
        if index in visited:
            continue

        stack = [index]
        component: list[int] = []
        visited.add(index)

        while stack:
            current_index = stack.pop()
            component.append(current_index)
            current_sample = residual_samples[current_index]
            current_radius_mm = min(
                CLUSTER_SPLIT_MAX_RADIUS_MM,
                max(CLUSTER_RADIUS_MIN_MM, cluster_radius_mm(current_sample) * CLUSTER_SPLIT_RADIUS_SCALE),
            )

            for candidate_index in member_indexes:
                if candidate_index == current_index or candidate_index in visited:
                    continue
                if candidate_index not in member_set:
                    continue
                candidate_sample = residual_samples[candidate_index]
                candidate_radius_mm = min(
                    CLUSTER_SPLIT_MAX_RADIUS_MM,
                    max(CLUSTER_RADIUS_MIN_MM, cluster_radius_mm(candidate_sample) * CLUSTER_SPLIT_RADIUS_SCALE),
                )
                pair_radius_mm = min(current_radius_mm, candidate_radius_mm)
                if not pair_is_cluster_compatible(
                    current_sample,
                    candidate_sample,
                    pair_radius_mm,
                    max_yaw_gap_deg=CLUSTER_SPLIT_MAX_YAW_GAP_DEG,
                    max_pitch_gap_deg=CLUSTER_SPLIT_MAX_PITCH_GAP_DEG,
                    max_occlusion_delta_mm=CLUSTER_SPLIT_OCCLUSION_DELTA_MM,
                ):
                    continue
                visited.add(candidate_index)
                stack.append(candidate_index)

        components.append(component)

    filtered_components = [
        component
        for component in components
        if len(component) >= CLUSTER_SPLIT_MIN_POINTS
    ]
    return filtered_components or [member_indexes]


def bbox_from_samples(samples: list[dict[str, float | int]]) -> dict[str, float]:
    xs = [float(sample["x_mm"]) for sample in samples]
    ys = [float(sample["y_mm"]) for sample in samples]
    zs = [float(sample["z_mm"]) for sample in samples]
    min_x = min(xs)
    max_x = max(xs)
    min_y = min(ys)
    max_y = max(ys)
    min_z = min(zs)
    max_z = max(zs)
    size_x = max_x - min_x
    size_y = max_y - min_y
    size_z = max_z - min_z
    return {
        "min_x_mm": min_x,
        "max_x_mm": max_x,
        "min_y_mm": min_y,
        "max_y_mm": max_y,
        "min_z_mm": min_z,
        "max_z_mm": max_z,
        "size_x_mm": size_x,
        "size_y_mm": size_y,
        "size_z_mm": size_z,
        "center_x_mm": (min_x + max_x) / 2.0,
        "center_y_mm": (min_y + max_y) / 2.0,
        "center_z_mm": (min_z + max_z) / 2.0,
        "diagonal_mm": math.sqrt(size_x * size_x + size_y * size_y + size_z * size_z),
    }


def score_debris_cluster(
    point_count: int,
    mean_occlusion_mm: float,
    max_occlusion_mm: float,
    occupancy_ratio: float,
    diagonal_mm: float,
) -> float:
    max_component = min(1.0, max_occlusion_mm / 250.0)
    mean_component = min(1.0, mean_occlusion_mm / 150.0)
    count_component = min(1.0, point_count / 24.0)
    occupancy_component = min(1.0, occupancy_ratio)
    compactness_component = max(0.0, 1.0 - min(1.0, diagonal_mm / DEBRIS_MAX_DIAGONAL_MM))
    return round(
        (
            0.34 * max_component
            + 0.24 * mean_component
            + 0.22 * count_component
            + 0.10 * occupancy_component
            + 0.10 * compactness_component
        )
        * 100.0,
        1,
    )


def cluster_residual_samples(
    residual_samples: list[dict[str, float | int]],
    tolerance_mm: float,
) -> tuple[list[dict[str, Any]], dict[int, str]]:
    if not residual_samples:
        return [], {}

    hash_map: dict[tuple[int, int, int], list[int]] = {}
    for index, sample in enumerate(residual_samples):
        hash_map.setdefault(
            spatial_hash_cell(
                float(sample["x_mm"]),
                float(sample["y_mm"]),
                float(sample["z_mm"]),
            ),
            [],
        ).append(index)

    neighbor_cache: dict[int, list[int]] = {}

    def neighbors_for(index: int) -> list[int]:
        cached = neighbor_cache.get(index)
        if cached is not None:
            return cached

        sample = residual_samples[index]
        x_mm = float(sample["x_mm"])
        y_mm = float(sample["y_mm"])
        z_mm = float(sample["z_mm"])
        radius_mm = cluster_radius_mm(sample)
        cell_x, cell_y, cell_z = spatial_hash_cell(x_mm, y_mm, z_mm)
        neighbors: list[int] = []
        for offset_x in (-1, 0, 1):
            for offset_y in (-1, 0, 1):
                for offset_z in (-1, 0, 1):
                    for candidate_index in hash_map.get(
                        (cell_x + offset_x, cell_y + offset_y, cell_z + offset_z),
                        [],
                    ):
                        candidate = residual_samples[candidate_index]
                        pair_radius_mm = max(radius_mm, cluster_radius_mm(candidate))
                        if pair_is_cluster_compatible(sample, candidate, pair_radius_mm):
                            neighbors.append(candidate_index)

        neighbor_cache[index] = neighbors
        return neighbors

    supported_indexes = {
        index
        for index in range(len(residual_samples))
        if len([neighbor for neighbor in neighbors_for(index) if neighbor != index]) >= CLUSTER_MIN_LOCAL_SUPPORT
    }

    labels = [-1] * len(residual_samples)
    visited = [False] * len(residual_samples)
    cluster_index = 0
    for index in range(len(residual_samples)):
        if index not in supported_indexes:
            continue
        if visited[index]:
            continue
        visited[index] = True
        seed_neighbors = [neighbor for neighbor in neighbors_for(index) if neighbor in supported_indexes]
        if len(seed_neighbors) < CLUSTER_DBSCAN_MIN_NEIGHBORS:
            continue
        labels[index] = cluster_index
        queue = list(seed_neighbors)
        while queue:
            neighbor_index = queue.pop()
            if neighbor_index not in supported_indexes:
                continue
            if not visited[neighbor_index]:
                visited[neighbor_index] = True
                expanded_neighbors = [
                    expanded_index
                    for expanded_index in neighbors_for(neighbor_index)
                    if expanded_index in supported_indexes
                ]
                if len(expanded_neighbors) >= CLUSTER_DBSCAN_MIN_NEIGHBORS:
                    for expanded_index in expanded_neighbors:
                        if expanded_index not in queue:
                            queue.append(expanded_index)
            if labels[neighbor_index] == -1:
                labels[neighbor_index] = cluster_index
        cluster_index += 1

    cluster_members: dict[int, list[int]] = {}
    for index, label in enumerate(labels):
        if label >= 0:
            cluster_members.setdefault(label, []).append(index)

    valid_clusters: list[dict[str, Any]] = []
    sample_to_cluster: dict[int, str] = {}
    min_max_occlusion_mm = max(DEBRIS_MIN_MAX_OCCLUSION_MM, tolerance_mm + 5.0)
    min_mean_occlusion_mm = max(DEBRIS_MIN_MEAN_OCCLUSION_MM, tolerance_mm)

    for original_member_indexes in cluster_members.values():
        member_groups = split_member_indexes(residual_samples, original_member_indexes)
        for member_indexes in member_groups:
            samples = [residual_samples[index] for index in member_indexes]
            if len(samples) < DEBRIS_MIN_POINTS:
                continue

            unique_yaws = {round(float(sample["yaw_deg"]), 2) for sample in samples}
            unique_pitches = {round(float(sample["pitch_deg"]), 2) for sample in samples}
            if len(unique_yaws) < DEBRIS_MIN_UNIQUE_YAWS or len(unique_pitches) < DEBRIS_MIN_UNIQUE_PITCHES:
                continue

            occlusions = [float(sample.get("occlusion_distance_mm", 0.0)) for sample in samples]
            mean_occlusion_mm = sum(occlusions) / len(occlusions)
            max_occlusion_mm = max(occlusions)
            if max_occlusion_mm < min_max_occlusion_mm or mean_occlusion_mm < min_mean_occlusion_mm:
                continue

            bbox = bbox_from_samples(samples)
            if bbox["diagonal_mm"] > DEBRIS_MAX_DIAGONAL_MM:
                continue

            point_count = len(samples)
            occupancy_ratio = point_count / max(1, len(unique_yaws) * len(unique_pitches))
            if occupancy_ratio < DEBRIS_MIN_OCCUPANCY_RATIO:
                continue
            centroid_x_mm = sum(float(sample["x_mm"]) for sample in samples) / point_count
            centroid_y_mm = sum(float(sample["y_mm"]) for sample in samples) / point_count
            centroid_z_mm = sum(float(sample["z_mm"]) for sample in samples) / point_count
            cluster_id = (
                f"debris-"
                f"{round(centroid_x_mm / 100.0)}-"
                f"{round(centroid_y_mm / 100.0)}-"
                f"{round(centroid_z_mm / 100.0)}"
            )

            centroid_yaw_deg = round(math.degrees(math.atan2(centroid_y_mm, centroid_x_mm)), 2)
            centroid_pitch_deg = round(math.degrees(math.atan2(centroid_z_mm, math.hypot(centroid_x_mm, centroid_y_mm))), 2)
            valid_clusters.append(
                {
                    "cluster_id": cluster_id,
                    "centroid_yaw_deg": centroid_yaw_deg,
                    "centroid_pitch_deg": centroid_pitch_deg,
                    "score": score_debris_cluster(
                        point_count,
                        mean_occlusion_mm,
                        max_occlusion_mm,
                        occupancy_ratio,
                        float(bbox["diagonal_mm"]),
                    ),
                    "point_count": point_count,
                    "mean_occlusion_mm": round(mean_occlusion_mm, 1),
                    "max_occlusion_mm": round(max_occlusion_mm, 1),
                    "unique_yaw_count": len(unique_yaws),
                    "unique_pitch_count": len(unique_pitches),
                    "occupancy_ratio": round(occupancy_ratio, 3),
                    "centroid_mm": [
                        round(centroid_x_mm, 1),
                        round(centroid_y_mm, 1),
                        round(centroid_z_mm, 1),
                    ],
                    "bbox": {key: round(value, 1) for key, value in bbox.items()},
                }
            )
            for sample_index in member_indexes:
                sample_to_cluster[sample_index] = cluster_id

    valid_clusters.sort(
        key=lambda cluster: (
            -float(cluster["score"]),
            -int(cluster["point_count"]),
            -float(cluster["max_occlusion_mm"]),
        )
    )
    return valid_clusters, sample_to_cluster
