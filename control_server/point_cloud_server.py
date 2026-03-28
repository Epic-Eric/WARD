#!/usr/bin/env python3
"""
FastAPI control server for the Nano RP2040 Connect point cloud scanner.

HTTP UI:
    http://127.0.0.1:8000

Robot TCP port:
    9000

Usage:
    python control_server/point_cloud_server.py
"""

from __future__ import annotations

try:
    import uvicorn
except ModuleNotFoundError as exc:
    uvicorn = None
    UVICORN_IMPORT_ERROR = exc
else:
    UVICORN_IMPORT_ERROR = None

try:
    from services.api import IMPORT_ERROR as FASTAPI_IMPORT_ERROR, create_app
    from services.automation import ScanAutomationController
    from services.config import (
        BASELINE_META_PATH,
        BASELINE_PATH,
        CAPTURE_DIR,
        HTTP_HOST,
        HTTP_PORT,
        ROBOT_HOST,
        ROBOT_PORT,
    )
    from services.robot_hub import RobotTcpHub
except ModuleNotFoundError:
    from control_server.services.api import (
        IMPORT_ERROR as FASTAPI_IMPORT_ERROR,
        create_app,
    )
    from control_server.services.automation import ScanAutomationController
    from control_server.services.config import (
        BASELINE_META_PATH,
        BASELINE_PATH,
        CAPTURE_DIR,
        HTTP_HOST,
        HTTP_PORT,
        ROBOT_HOST,
        ROBOT_PORT,
    )
    from control_server.services.robot_hub import RobotTcpHub


robot_hub = RobotTcpHub(ROBOT_HOST, ROBOT_PORT, CAPTURE_DIR)
automation = ScanAutomationController(
    robot_hub,
    CAPTURE_DIR,
    BASELINE_PATH,
    BASELINE_META_PATH,
)
app = create_app(robot_hub, automation)


def main() -> None:
    if FASTAPI_IMPORT_ERROR is not None or UVICORN_IMPORT_ERROR is not None:
        print("FastAPI server dependencies are missing.")
        print("Install them with:")
        print("  pip install fastapi uvicorn")
        raise SystemExit(1)

    assert uvicorn is not None
    assert app is not None
    robot_hub.start()
    try:
        uvicorn.run(app, host=HTTP_HOST, port=HTTP_PORT)
    finally:
        robot_hub.stop()


if __name__ == "__main__":
    main()
