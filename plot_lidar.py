"""
LIDAR Scanner — Serial reader & 3D point cloud plot.

Usage:
    python plot_lidar.py              # auto-detect serial port
    python plot_lidar.py /dev/cu.usbmodem14201   # specify port
    python plot_lidar.py --file data.csv         # plot from saved CSV
"""

import sys
import csv
import math
import time
import numpy as np
import matplotlib.pyplot as plt


def import_serial_modules():
    try:
        import serial
        import serial.tools.list_ports
    except ModuleNotFoundError as exc:
        print("Serial mode requires the `pyserial` package.")
        print("You installed the unrelated `serial` package.")
        print("Fix with:")
        print("  pip uninstall serial")
        print("  pip install pyserial")
        raise SystemExit(1) from exc

    return serial, serial.tools.list_ports


def find_port():
    """Auto-detect the Arduino serial port."""
    _, list_ports = import_serial_modules()
    ports = list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in ["arduino", "rp2040", "usb modem", "usbmodem"]):
            return p.device
    # Fallback: first port
    if ports:
        return ports[0].device
    return None


def run_scan(port):
    """Connect to Arduino, send 'scan', and collect CSV lines."""
    serial, _ = import_serial_modules()
    print(f"Connecting to {port} ...")
    ser = serial.Serial(port, 115200, timeout=2)
    time.sleep(2)  # wait for Arduino reset

    # Drain startup messages
    while ser.in_waiting:
        print(ser.readline().decode(errors="replace").strip())

    print("Sending 'scan' command...")
    ser.write(b"scan\n")

    data = []
    scanning = False

    while True:
        line = ser.readline().decode(errors="replace").strip()
        if not line:
            continue

        if line == "SCAN_START":
            scanning = True
            print("Scan started — collecting data...")
            continue
        if line == "SCAN_END":
            print(f"Scan complete. {len(data)} points collected.")
            break

        if scanning:
            parts = line.split(",")
            if len(parts) == 3:
                try:
                    yaw   = float(parts[0])
                    pitch = float(parts[1])
                    dist  = float(parts[2])
                    data.append((yaw, pitch, dist))
                    # Progress indicator
                    if len(data) % 50 == 0:
                        print(f"  {len(data)} points ...", end="\r")
                except ValueError:
                    pass

    ser.close()
    return data


def spherical_to_cartesian_meters(yaw_deg, pitch_deg, dist_mm):
    if dist_mm == 0:
        return None

    yaw = math.radians(yaw_deg)
    pitch = math.radians(pitch_deg)
    dist_m = dist_mm / 1000.0

    cos_pitch = math.cos(pitch)
    x_m = dist_m * cos_pitch * math.cos(yaw)
    y_m = dist_m * cos_pitch * math.sin(yaw)
    z_m = dist_m * math.sin(pitch)
    return (x_m, y_m, z_m, dist_m)


def convert_scan_samples_to_points(samples):
    points = []
    for yaw_deg, pitch_deg, dist_mm in samples:
        point = spherical_to_cartesian_meters(yaw_deg, pitch_deg, dist_mm)
        if point is not None:
            points.append(point)
    return points


def load_csv(path):
    """Load previously saved CSV data."""
    data = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)

        if reader.fieldnames is None:
            return data

        fieldnames = {name.strip() for name in reader.fieldnames}

        if {"frame_id", "point_index", "distance_mm", "x_mm", "y_mm", "z_mm"} <= fieldnames:
            for row in reader:
                try:
                    x_m = float(row["x_mm"]) / 1000.0
                    y_m = float(row["y_mm"]) / 1000.0
                    z_m = float(row["z_mm"]) / 1000.0
                    dist_m = float(row["distance_mm"]) / 1000.0
                except (TypeError, ValueError):
                    continue
                data.append((x_m, y_m, z_m, dist_m))
            return data

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("yaw"):
                continue
            parts = line.split(",")
            if len(parts) == 3:
                try:
                    yaw_deg = float(parts[0])
                    pitch_deg = float(parts[1])
                    dist_mm = float(parts[2])
                except ValueError:
                    continue

                point = spherical_to_cartesian_meters(yaw_deg, pitch_deg, dist_mm)
                if point is not None:
                    data.append(point)
    return data


def save_csv(data, path="scan_data.csv"):
    with open(path, "w") as f:
        f.write("yaw_deg,pitch_deg,distance_mm\n")
        for yaw, pitch, dist in data:
            f.write(f"{yaw},{pitch},{dist}\n")
    print(f"Data saved to {path}")


def plot_point_cloud_3d(data):
    """
    Plot a 3D point cloud from Cartesian (x, y, z, distance) points.
    """
    xs, ys, zs, colors = [], [], [], []

    for x_m, y_m, z_m, dist_m in data:
        xs.append(x_m)
        ys.append(y_m)
        zs.append(z_m)
        colors.append(dist_m)

    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)
    colors = np.array(colors)

    fig = plt.figure(figsize=(11, 9))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(xs, ys, zs, c=colors, cmap="viridis_r", s=6, alpha=0.85)
    cbar = plt.colorbar(sc, ax=ax, shrink=0.7)
    cbar.set_label("Distance (m)")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("LIDAR 3D Point Cloud")
    ax.scatter([0], [0], [0], c="red", marker="+", s=120, linewidths=2)
    ax.grid(True, alpha=0.3)
    ax.set_box_aspect((np.ptp(xs) or 1.0, np.ptp(ys) or 1.0, np.ptp(zs) or 1.0))
    ax.view_init(elev=25, azim=45)

    plt.tight_layout()
    plt.savefig("lidar_point_cloud_3d.png", dpi=150)
    print("Plot saved to lidar_point_cloud_3d.png")
    plt.show()


if __name__ == "__main__":
    # Parse args
    if "--file" in sys.argv:
        idx = sys.argv.index("--file")
        csv_path = sys.argv[idx + 1]
        data = load_csv(csv_path)
    else:
        port = sys.argv[1] if len(sys.argv) > 1 else find_port()
        if not port:
            print("No serial port found. Specify one: python plot_lidar.py /dev/cu.usbmodemXXXX")
            sys.exit(1)
        raw_data = run_scan(port)
        save_csv(raw_data)
        data = convert_scan_samples_to_points(raw_data)

    if not data:
        print("No data collected.")
        sys.exit(1)

    plot_point_cloud_3d(data)
