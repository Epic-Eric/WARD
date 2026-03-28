from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from .common import timestamp_string


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
