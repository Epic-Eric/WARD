from __future__ import annotations

import socket
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from .capture import CaptureRecorder, SessionStats
from .common import detect_hostname, detect_lan_ip
from .config import HTTP_PORT, ROBOT_STALE_TIMEOUT_SECONDS, SOCKET_TIMEOUT_SECONDS


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
            self._apply_optimistic_command_state_locked(command)
            self._last_event = f"Sent command {command}"

    def _apply_optimistic_command_state_locked(self, command: str) -> None:
        normalized_command = command.strip().upper()
        if normalized_command.startswith("START_SCAN"):
            self._robot_state = "scanning"
            self._scan_in_progress = True
            if self._sensor_status in {None, "READY", "IDLE"}:
                self._sensor_status = "SCANNING"
            return
        if normalized_command == "STOP_SCAN":
            self._robot_state = "stopping"
            self._scan_in_progress = True
            return
        if normalized_command == "HARD_STOP":
            self._robot_state = "hard_stopping"
            self._scan_in_progress = True
            return
        if normalized_command == "RELEASE_MOTORS":
            self._robot_state = "releasing"
            self._scan_in_progress = False
            return
        if normalized_command in {"ZERO_TURRET", "RESET_TURRET"}:
            self._robot_state = "zeroing"
            self._scan_in_progress = False
            return
        if normalized_command.startswith("MOVE_TO,"):
            self._robot_state = "moving"
            self._scan_in_progress = False
            return

    def scan_result_token(self) -> int:
        with self._lock:
            return self._scan_result_sequence

    def wait_for_scan_completion(self, after_token: int, timeout_seconds: float) -> dict[str, Any]:
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
            self._disconnect_client(client_socket)

    def _disconnect_client(self, client_socket: socket.socket) -> None:
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
            if self._handle_connection_messages(message_type, parts, line):
                return
            if self._handle_scan_messages(message_type, parts, line):
                return
            self._last_event = f"Ignored message: {line}"

    def _handle_connection_messages(self, message_type: str, parts: list[str], line: str) -> bool:
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
            return True
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
            return True
        if message_type == "STATE" and len(parts) >= 2:
            state = parts[1].strip().lower()
            self._robot_state = state
            self._scan_in_progress = state in {"scanning", "stopping", "hard_stopping", "releasing"}
            self._last_event = f"Robot state: {state}"
            return True
        if message_type == "POSE" and len(parts) >= 3:
            try:
                self._current_yaw_deg = float(parts[1])
                self._current_pitch_deg = float(parts[2])
            except ValueError:
                self._current_yaw_deg = None
                self._current_pitch_deg = None
            return True
        if message_type == "SENSOR_STATUS" and len(parts) >= 2:
            self._sensor_status = ",".join(parts[1:])
            if self._sensor_status == "READY":
                self._sensor_timeout = False
                self._last_sensor_error = None
            if self._sensor_status == "RECOVERING_SENSOR":
                self._last_event = "Sensor recovery in progress."
            return True
        if message_type == "HEARTBEAT":
            if len(parts) >= 3:
                self._robot_state = parts[2].strip().lower()
                self._scan_in_progress = self._robot_state in {"scanning", "stopping", "hard_stopping", "releasing"}
            if len(parts) >= 5:
                try:
                    self._current_yaw_deg = float(parts[3])
                    self._current_pitch_deg = float(parts[4])
                except ValueError:
                    pass
            self._last_event = f"Heartbeat from robot at {parts[1] if len(parts) > 1 else '?'}"
            return True
        if message_type == "COMMAND_ACK" and len(parts) >= 2:
            self._last_command_status = "ack"
            self._last_command_detail = ",".join(parts[1:])
            self._last_event = f"Command ack: {self._last_command_detail}"
            return True
        if message_type == "COMMAND_FAIL" and len(parts) >= 2:
            command_name = parts[1].strip().upper()
            if command_name.startswith("START_SCAN"):
                self._scan_in_progress = False
                self._robot_state = "idle"
                if self._sensor_status == "SCANNING":
                    self._sensor_status = "READY"
            elif command_name == "STOP_SCAN":
                self._robot_state = "scanning" if self._live_frame_id is not None else "idle"
                self._scan_in_progress = self._robot_state == "scanning"
            elif command_name == "HARD_STOP":
                self._robot_state = "idle"
                self._scan_in_progress = False
            elif command_name in {"RELEASE_MOTORS", "ZERO_TURRET", "RESET_TURRET"}:
                self._robot_state = "idle"
                self._scan_in_progress = False
            self._last_command_status = "fail"
            self._last_command_detail = ",".join(parts[1:])
            self._last_error = f"Command rejected: {self._last_command_detail}"
            self._last_event = self._last_error
            return True
        return False

    def _handle_scan_messages(self, message_type: str, parts: list[str], line: str) -> bool:
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
            return True
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
                "capture_path": str(self._recorder.csv_path) if self._recorder.csv_path else None,
                "point_count": self._stats.current_frame_points,
            }
            self._scan_condition.notify_all()
            return True
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
                frame_id = point_index = 0
                yaw_deg = pitch_deg = distance_mm = x_mm = y_mm = z_mm = 0.0
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
            return True
        if message_type == "SENSOR_TIMEOUT":
            self._sensor_timeout = True
            self._sensor_status = "SENSOR_TIMEOUT"
            self._last_sensor_error = ",".join(parts[1:]) if len(parts) > 1 else "SENSOR_TIMEOUT"
            self._last_event = f"Sensor timeout: {self._last_sensor_error}"
            return True
        if message_type == "ABORT":
            self._scan_in_progress = False
            self._robot_state = "idle"
            if self._sensor_status == "SCANNING":
                self._sensor_status = "ABORTED"
            frame_id = None
            if len(parts) > 1:
                try:
                    frame_id = int(parts[1])
                except ValueError:
                    frame_id = self._stats.current_frame
            self._last_event = f"Scan aborted: {line}"
            self._scan_result_sequence += 1
            self._last_scan_result = {
                "status": "aborted",
                "frame_id": frame_id,
                "capture_path": str(self._recorder.csv_path) if self._recorder.csv_path else None,
                "point_count": self._stats.current_frame_points,
            }
            self._scan_condition.notify_all()
            return True
        return False

    def _note_robot_activity_locked(self) -> None:
        self._last_robot_message_monotonic = time.monotonic()
        self._last_robot_seen_at = datetime.now().isoformat(timespec="seconds")

    def _is_robot_stale_locked(self) -> bool:
        if not self._robot_connected or self._last_robot_message_monotonic is None:
            return False
        return (time.monotonic() - self._last_robot_message_monotonic) > ROBOT_STALE_TIMEOUT_SECONDS

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
