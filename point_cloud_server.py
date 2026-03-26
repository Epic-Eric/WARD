#!/usr/bin/env python3
"""
TCP receiver for Nano RP2040 Connect point cloud frames.

Usage:
    python point_cloud_server.py --host 0.0.0.0 --port 9000 --output-dir captures
"""

from __future__ import annotations

import argparse
import csv
import socket
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

CLIENT_IDLE_TIMEOUT_SECONDS = 10.0


@dataclass
class SessionStats:
    frames: int = 0
    points: int = 0
    current_frame: int = 0
    current_frame_points: int = 0

    def reset(self) -> None:
        self.frames = 0
        self.points = 0
        self.current_frame = 0
        self.current_frame_points = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Receive point cloud frames over TCP.")
    parser.add_argument("--host", default="0.0.0.0", help="Address to bind to.")
    parser.add_argument("--port", type=int, default=9000, help="TCP port to listen on.")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path("captures"),
        help="Directory for captured CSV files.",
    )
    return parser.parse_args()


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


def open_capture_file(output_dir: Path) -> tuple[Path, csv.writer, object]:
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / f"point_cloud_{timestamp_string()}.csv"
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
        self.file_obj: object | None = None
        self.pending_reason: str | None = "connection start"
        self.rows_written = 0

    def start_new_capture(self, reason: str) -> None:
        self.close()
        self.csv_path, self.writer, self.file_obj = open_capture_file(self.output_dir)
        self.rows_written = 0
        print(f"Writing capture to {self.csv_path} ({reason})")

    def ensure_open(self) -> None:
        if self.writer is None or self.file_obj is None:
            self.start_new_capture(self.pending_reason or "implicit capture start")
            self.pending_reason = None

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
            self.file_obj.close()
        self.csv_path = None
        self.writer = None
        self.file_obj = None


def handle_line(line: str, recorder: CaptureRecorder, stats: SessionStats) -> None:
    if not line:
        return

    parts = line.split(",")
    message_type = parts[0]

    if message_type == "HELLO":
        stats.reset()
        recorder.mark_run_boundary("HELLO")
        print(f"Board says: {line}")
        return

    if message_type == "RESET":
        stats.reset()
        recorder.mark_run_boundary("RESET")
        print(f"Board reset capture: {line}")
        return

    if message_type == "FRAME_BEGIN" and len(parts) >= 3:
        recorder.ensure_open()
        stats.frames += 1
        stats.current_frame = int(parts[1])
        stats.current_frame_points = 0
        print(f"Frame {parts[1]} started at device millis={parts[2]}")
        return

    if message_type == "FRAME_END" and len(parts) >= 3:
        recorder.flush()
        print(f"Frame {parts[1]} finished with {parts[2]} points")
        return

    if message_type == "POINT" and len(parts) == 9:
        recorder.write_point(parts[1:])
        stats.points += 1
        stats.current_frame_points += 1
        recorder.flush()
        if stats.current_frame_points % 25 == 0:
            print(
                f"Frame {stats.current_frame}: {stats.current_frame_points} points "
                f"received, total={stats.points}"
            )
        return

    print(f"Ignoring malformed line: {line}")


def serve_once(client_socket: socket.socket, address: tuple[str, int], output_dir: Path) -> None:
    print(f"Accepted connection from {address[0]}:{address[1]}")
    recorder = CaptureRecorder(output_dir)
    stats = SessionStats()

    with client_socket:
        client_socket.settimeout(CLIENT_IDLE_TIMEOUT_SECONDS)
        buffer = ""
        while True:
            try:
                chunk = client_socket.recv(4096)
            except socket.timeout:
                print(
                    f"Connection idle for {CLIENT_IDLE_TIMEOUT_SECONDS:.0f}s, "
                    "closing session and waiting for reconnect."
                )
                break

            if not chunk:
                break

            buffer += chunk.decode("utf-8", errors="replace")
            while "\n" in buffer:
                raw_line, buffer = buffer.split("\n", 1)
                handle_line(raw_line.strip(), recorder, stats)

        if buffer.strip():
            handle_line(buffer.strip(), recorder, stats)

    recorder.close()
    print(f"Connection closed. Frames={stats.frames}, points={stats.points}")


def main() -> None:
    args = parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((args.host, args.port))
        server_socket.listen(1)

        print(f"Listening on {args.host}:{args.port}")
        print(f"Detected LAN IP: {detect_lan_ip()}")
        print(f"Detected hostname: {detect_hostname()}")
        print("Set Config::kServerHost in src/main.cpp to the detected hostname.")

        while True:
            client_socket, address = server_socket.accept()
            serve_once(client_socket, address, args.output_dir)


if __name__ == "__main__":
    main()
