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
import socket
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import FileResponse
    import uvicorn
except ModuleNotFoundError as exc:
    FastAPI = None
    HTTPException = RuntimeError
    FileResponse = None
    uvicorn = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
INDEX_HTML = STATIC_DIR / "index.html"
CAPTURE_DIR = BASE_DIR / "captures"

HTTP_HOST = "0.0.0.0"
HTTP_PORT = 8000
ROBOT_HOST = "0.0.0.0"
ROBOT_PORT = 9000
SOCKET_TIMEOUT_SECONDS = 1.0


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


def open_capture_file(output_dir: Path) -> tuple[Path, csv.writer, Any]:
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
        self._current_yaw_deg: float | None = None
        self._current_pitch_deg: float | None = None
        self._sensor_status: str | None = None
        self._sensor_timeout = False
        self._last_sensor_error: str | None = None
        self._last_event = "Waiting for robot connection."
        self._last_error: str | None = None
        self._last_command: str | None = None
        self._live_frame_id: int | None = None
        self._live_points: list[list[float]] = []

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
            if self._client_socket is None or not self._robot_connected:
                raise RuntimeError("Robot is not connected.")

            try:
                self._client_socket.sendall(encoded)
            except OSError as exc:
                self._last_error = f"Failed to send command: {exc}"
                raise RuntimeError("Failed to send command to robot.") from exc

            self._last_command = command
            self._last_event = f"Sent command {command}"

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            status = {
                "robot_connected": self._robot_connected,
                "robot_address": self._robot_address,
                "robot_protocol": self._robot_protocol,
                "robot_state": self._robot_state,
                "scan_in_progress": self._scan_in_progress,
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
                "robot_tcp_port": self.port,
                "http_port": HTTP_PORT,
                "detected_lan_ip": detect_lan_ip(),
                "detected_hostname": detect_hostname(),
            }
            status.update(self._recorder.snapshot())
            return status

    def live_points_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "frame_id": self._live_frame_id,
                "points": list(self._live_points),
                "scan_in_progress": self._scan_in_progress,
                "robot_state": self._robot_state,
                "current_yaw_deg": self._current_yaw_deg,
                "current_pitch_deg": self._current_pitch_deg,
                "sensor_status": self._sensor_status,
                "sensor_timeout": self._sensor_timeout,
            }

    def _serve_forever(self) -> None:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind((self.host, self.port))
        server_socket.listen(1)
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
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    continue

                self._handle_client(client_socket, address)
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

        with self._lock:
            self._client_socket = client_socket
            self._robot_connected = True
            self._robot_address = peer
            self._robot_state = "connected"
            self._scan_in_progress = False
            self._last_error = None
            self._last_event = f"Robot connected from {peer}"

        client_socket.settimeout(SOCKET_TIMEOUT_SECONDS)
        buffer = ""

        try:
            while not self._stop_event.is_set():
                try:
                    chunk = client_socket.recv(4096)
                except socket.timeout:
                    continue

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

            with self._lock:
                if self._client_socket is client_socket:
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
                self._last_event = "Robot disconnected."

            self._recorder.close()
            print("Robot disconnected.")

    def _handle_robot_line(self, line: str) -> None:
        if not line:
            return

        parts = line.split(",")
        message_type = parts[0]

        with self._lock:
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
                return

            if message_type == "STATE" and len(parts) >= 2:
                state = parts[1].strip().lower()
                self._robot_state = state
                self._scan_in_progress = state in {"scanning", "stopping", "hard_stopping", "releasing"}
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
                self._last_event = (
                    f"Frame {parts[1]} started at device millis={parts[2]}"
                )
                return

            if message_type == "FRAME_END" and len(parts) >= 3:
                self._recorder.flush()
                self._scan_in_progress = False
                self._robot_state = "idle"
                if self._sensor_status == "SCANNING":
                    self._sensor_status = "READY"
                self._last_event = f"Frame {parts[1]} finished with {parts[2]} points"
                return

            if message_type == "POINT" and len(parts) == 9:
                self._recorder.write_point(parts[1:])
                self._stats.points += 1
                self._stats.current_frame_points += 1
                try:
                    x_mm = float(parts[6])
                    y_mm = float(parts[7])
                    z_mm = float(parts[8])
                    distance_mm = float(parts[5])
                except ValueError:
                    x_mm = y_mm = z_mm = distance_mm = 0.0
                self._live_points.append([x_mm, y_mm, z_mm, distance_mm])
                if self._stats.current_frame_points % 25 == 0:
                    self._recorder.flush()
                return

            if message_type == "SENSOR_TIMEOUT":
                self._sensor_timeout = True
                self._sensor_status = "SENSOR_TIMEOUT"
                self._last_sensor_error = ",".join(parts[1:]) if len(parts) > 1 else "SENSOR_TIMEOUT"
                self._last_event = f"Sensor timeout: {self._last_sensor_error}"
                return

            if message_type == "ABORT":
                self._scan_in_progress = False
                if self._sensor_status == "SCANNING":
                    self._sensor_status = "ABORTED"
                self._last_event = f"Scan aborted: {line}"
                return

            self._last_event = f"Ignored message: {line}"


robot_hub = RobotTcpHub(ROBOT_HOST, ROBOT_PORT, CAPTURE_DIR)


if FastAPI is not None:
    app = FastAPI(title="WARD Point Cloud Control")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(INDEX_HTML)

    @app.get("/api/status")
    async def api_status() -> dict[str, Any]:
        return robot_hub.snapshot()

    @app.get("/api/live-points")
    async def api_live_points() -> dict[str, Any]:
        return robot_hub.live_points_snapshot()

    @app.post("/api/robot/start-scan")
    async def api_start_scan() -> dict[str, Any]:
        status = robot_hub.snapshot()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")

        try:
            robot_hub.send_command("START_SCAN")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return robot_hub.snapshot()

    @app.post("/api/robot/stop-scan")
    async def api_stop_scan() -> dict[str, Any]:
        status = robot_hub.snapshot()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if not status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is not scanning.")

        try:
            robot_hub.send_command("STOP_SCAN")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return robot_hub.snapshot()

    @app.post("/api/robot/hard-stop")
    async def api_hard_stop() -> dict[str, Any]:
        status = robot_hub.snapshot()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if not status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is not scanning.")

        try:
            robot_hub.send_command("HARD_STOP")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return robot_hub.snapshot()

    @app.post("/api/robot/release-motors")
    async def api_release_motors() -> dict[str, Any]:
        status = robot_hub.snapshot()
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")

        try:
            robot_hub.send_command("RELEASE_MOTORS")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return robot_hub.snapshot()

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
