from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta
from pathlib import Path


@dataclass
class CaptureCleanupSummary:
    removed_count: int = 0
    kept_count: int = 0


def prune_capture_files(
    capture_dir: Path,
    keep_latest: int,
    retention_days: int,
) -> CaptureCleanupSummary:
    capture_dir.mkdir(parents=True, exist_ok=True)
    summary = CaptureCleanupSummary()
    cutoff = datetime.now() - timedelta(days=max(0, retention_days))
    candidates = sorted(
        [
            path
            for path in capture_dir.glob("*.csv")
            if path.name.startswith(("point_cloud_", "residual_"))
        ],
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )

    for index, path in enumerate(candidates):
        should_keep = index < max(0, keep_latest)
        is_recent = datetime.fromtimestamp(path.stat().st_mtime) >= cutoff
        if should_keep or is_recent:
            summary.kept_count += 1
            continue
        try:
            path.unlink()
            summary.removed_count += 1
        except OSError:
            summary.kept_count += 1

    return summary
