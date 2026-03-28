from __future__ import annotations

import json
import socket
from datetime import datetime
from pathlib import Path
from typing import Any


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


def format_optional_float(value: float | None) -> str | None:
    if value is None:
        return None
    return f"{value:.2f}"
