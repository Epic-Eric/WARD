from __future__ import annotations

from typing import Any

from .automation import ScanAutomationController
from .config import (
    DEFAULT_RESIDUAL_TOLERANCE_MM,
    DEFAULT_SCAN_DEGREES,
    INDEX_HTML,
    MAX_SCAN_DEGREES,
)
from .robot_hub import RobotTcpHub
from .scan_math import normalize_scan_degrees

try:
    from fastapi import FastAPI, HTTPException, Request
    from fastapi.responses import FileResponse
except ModuleNotFoundError as exc:
    FastAPI = None
    HTTPException = RuntimeError
    Request = Any
    FileResponse = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


def combined_status(
    robot_hub: RobotTcpHub, automation: ScanAutomationController
) -> dict[str, Any]:
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


async def read_baseline_import(
    request: Request,
) -> tuple[dict[str, Any], float | None, str | None]:
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
            source_name = str(
                payload.get("source_name")
                or bundle.get("imported_name")
                or "imported_bundle"
            )

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


def create_app(
    robot_hub: RobotTcpHub, automation: ScanAutomationController
) -> FastAPI | None:
    if FastAPI is None:
        return None

    app = FastAPI(title="WARD Point Cloud Control")

    @app.get("/")
    async def index() -> FileResponse:
        return FileResponse(INDEX_HTML)

    @app.get("/api/status")
    async def api_status() -> dict[str, Any]:
        return combined_status(robot_hub, automation)

    @app.get("/api/live-points")
    async def api_live_points(
        mode: str = "auto",
        tolerance_mm: float = DEFAULT_RESIDUAL_TOLERANCE_MM,
    ) -> dict[str, Any]:
        return automation.build_live_plot_payload(
            robot_hub.live_points_snapshot(),
            mode,
            tolerance_mm,
        )

    @app.post("/api/robot/start-scan")
    async def api_start_scan(request: Request) -> dict[str, Any]:
        scan_degrees = await read_scan_degrees(request)
        status = combined_status(robot_hub, automation)
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")
        if status["monitoring_active"]:
            raise HTTPException(
                status_code=409,
                detail="Stop monitoring before manual scans.",
            )
        if status["pending_operation"] is not None:
            raise HTTPException(
                status_code=409,
                detail="Wait for the current automation operation to finish.",
            )

        try:
            automation.set_configured_scan_degrees(scan_degrees)
            robot_hub.send_command(f"START_SCAN,{scan_degrees:.2f}")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status(robot_hub, automation)

    @app.post("/api/robot/stop-scan")
    async def api_stop_scan() -> dict[str, Any]:
        status = combined_status(robot_hub, automation)
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

        return combined_status(robot_hub, automation)

    @app.post("/api/robot/hard-stop")
    async def api_hard_stop() -> dict[str, Any]:
        status = combined_status(robot_hub, automation)
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")

        automation.stop_monitoring(request_robot_stop=False)
        try:
            robot_hub.send_command("HARD_STOP")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status(robot_hub, automation)

    @app.post("/api/robot/release-motors")
    async def api_release_motors() -> dict[str, Any]:
        status = combined_status(robot_hub, automation)
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")

        automation.stop_monitoring(request_robot_stop=False)
        try:
            robot_hub.send_command("RELEASE_MOTORS")
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc

        return combined_status(robot_hub, automation)

    @app.post("/api/robot/zero-turret")
    async def api_zero_turret() -> dict[str, Any]:
        status = combined_status(robot_hub, automation)
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

        return combined_status(robot_hub, automation)

    @app.post("/api/baseline/save")
    async def api_save_baseline() -> dict[str, Any]:
        status = combined_status(robot_hub, automation)
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")

        try:
            automation.save_baseline_from_latest_capture()
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status(robot_hub, automation)

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

        return combined_status(robot_hub, automation)

    @app.delete("/api/baseline")
    async def api_delete_baseline() -> dict[str, Any]:
        try:
            automation.delete_baseline()
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status(robot_hub, automation)

    @app.post("/api/monitoring/start")
    async def api_start_monitoring(request: Request) -> dict[str, Any]:
        scan_degrees = await read_scan_degrees(request)
        status = combined_status(robot_hub, automation)
        if not status["robot_connected"]:
            raise HTTPException(status_code=409, detail="Robot is not connected.")
        if status["scan_in_progress"]:
            raise HTTPException(status_code=409, detail="Robot is already scanning.")

        try:
            automation.start_monitoring(scan_degrees)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

        return combined_status(robot_hub, automation)

    @app.post("/api/monitoring/stop")
    async def api_stop_monitoring() -> dict[str, Any]:
        automation.stop_monitoring()
        return combined_status(robot_hub, automation)

    return app
