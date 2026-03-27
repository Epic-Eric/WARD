#!/usr/bin/env python3
"""Generate an end-to-end WARD tech stack diagram with diagrams/Graphviz."""

from pathlib import Path

from diagrams import Cluster, Diagram, Edge
from diagrams.custom import Custom
from diagrams.generic.storage import Storage
from diagrams.onprem.client import User
from diagrams.onprem.compute import Server
from diagrams.onprem.network import Internet
from diagrams.programming.framework import FastAPI, React
from diagrams.programming.language import Cpp, JavaScript, NodeJS, Python


ROOT = Path(__file__).resolve().parents[1]
OUTPUT_DIR = ROOT / "docs"
OUTPUT_BASE = OUTPUT_DIR / "ward_tech_stack_diagram"
ICON_DIR = ROOT / "assets" / "diagram-icons"


def hardware_node(icon_name: str, label: str) -> Custom:
    return Custom(label, (ICON_DIR / icon_name).as_posix())


def build_diagram() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    graph_attr = {
        "fontsize": "20",
        "pad": "0.35",
        "nodesep": "0.75",
        "ranksep": "1.0",
        "splines": "spline",
        "labelloc": "t",
        "labeljust": "l",
    }

    with Diagram(
        "WARD End-to-End Tech Stack",
        filename=OUTPUT_BASE.as_posix(),
        outformat=["png", "svg"],
        show=False,
        direction="LR",
        graph_attr=graph_attr,
    ):
        operator = User("Operator")
        scene = hardware_node("object_scene.png", "Physical scene\nforeground object / room")
        wifi_lan = Internet("Wi-Fi LAN")

        with Cluster("Robot Hardware"):
            nano = hardware_node("nano_rp2040.png", "Arduino Nano RP2040 Connect\nrobot controller")
            tof_sensor = hardware_node("tof_sensor.png", "VL53L1X ToF sensor\nI2C long-range samples")
            drv8833 = hardware_node("drv8833.png", "DRV8833 motor driver")
            yaw_motor = hardware_node("stepper_yaw.png", "Yaw stepper motor\n360 deg sweep via gear train")
            pitch_motor = hardware_node("stepper_pitch.png", "Pitch stepper motor\nvertical sweep")

        with Cluster("Firmware Stack (src/main.cpp)"):
            toolchain = Cpp(
                "PlatformIO + Arduino framework\nnanorp2040connect target"
            )
            firmware_libs = Cpp(
                "Embedded libraries\nWiFiNINA | Stepper | VL53L1X | Wire"
            )
            scan_controller = Cpp(
                "Scan controller\npose math | sweep logic | sensor recovery"
            )
            robot_protocol = Cpp(
                "Robot TCP client\nHELLO / HEARTBEAT / POSE / POINT\nSTART_SCAN / STOP_SCAN / HARD_STOP"
            )

        with Cluster("Host Software Stack"):
            host_machine = Server("Laptop / workstation")
            fastapi_app = FastAPI(
                "FastAPI + Uvicorn\npoint_cloud_server.py\nHTTP 8000"
            )
            robot_hub = Python(
                "Robot hub + capture engine\nTCP listener 9000\nbaseline / residual / monitoring"
            )
            capture_store = Storage(
                "Capture artifacts\ncaptures/*.csv\nbaseline_scan.csv\nbaseline meta JSON\nresidual captures"
            )
            control_ui = JavaScript(
                "Control dashboard\nstatic/index.html\n2D live plots + scan controls"
            )
            vite_toolchain = NodeJS(
                "viewer-react toolchain\nnpm | Vite dev/build"
            )
            react_viewer = React(
                "3D viewer\nReact 19 + Three.js\n@react-three/fiber + drei"
            )
            viewer_bundle = Storage(
                "Viewer bundle JSON\nraw / baseline / residual snapshots"
            )
            plot_utility = Python(
                "plot_lidar.py\nNumPy + Matplotlib\noptional pyserial serial mode"
            )
            plot_outputs = Storage(
                "Rendered plots\nlidar_map.png\nlidar_point_cloud_3d.png"
            )

        toolchain >> Edge(label="build + upload") >> firmware_libs
        firmware_libs >> Edge(label="used by") >> scan_controller
        scan_controller >> Edge(label="runs on") >> nano
        robot_protocol >> Edge(label="line protocol over Wi-Fi") >> nano
        scan_controller >> Edge(label="telemetry + scan points") >> robot_protocol

        scene >> Edge(label="distance target") >> tof_sensor
        tof_sensor >> Edge(label="I2C") >> nano
        nano >> Edge(label="step sequences") >> drv8833
        drv8833 >> Edge(label="coil drive") >> yaw_motor
        drv8833 >> Edge(label="coil drive") >> pitch_motor

        nano >> Edge(label="802.11 via WiFiNINA") >> wifi_lan
        wifi_lan >> Edge(label="TCP 9000") >> robot_hub
        robot_hub >> Edge(label="newline robot commands") >> wifi_lan

        host_machine >> fastapi_app
        host_machine >> robot_hub
        fastapi_app >> Edge(label="shared runtime state") >> robot_hub
        robot_hub >> Edge(label="writes captures / baseline / residuals") >> capture_store

        fastapi_app >> Edge(label="serves /") >> control_ui
        operator >> Edge(label="scan / baseline / monitoring control") >> control_ui
        control_ui >> Edge(
            label="GET /api/status\nGET /api/live-points\nPOST /api/robot/*\nPOST /api/baseline/*\nPOST /api/monitoring/*"
        ) >> fastapi_app

        vite_toolchain >> Edge(label="serves / builds") >> react_viewer
        operator >> Edge(label="3D inspection") >> react_viewer
        react_viewer >> Edge(
            label="GET /api/status\nGET /api/live-points\nPOST /api/baseline/import"
        ) >> fastapi_app
        react_viewer >> Edge(label="export / import snapshots") >> viewer_bundle

        capture_store >> Edge(label="offline CSV input") >> plot_utility
        nano >> Edge(label="optional USB serial path") >> plot_utility
        plot_utility >> Edge(label="renders") >> plot_outputs


if __name__ == "__main__":
    build_diagram()
    print(f"Generated {OUTPUT_BASE.with_suffix('.png')}")
    print(f"Generated {OUTPUT_BASE.with_suffix('.svg')}")
