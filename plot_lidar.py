"""
LIDAR Scanner — Serial reader & 2D top-down plot.

Usage:
    python plot_lidar.py              # auto-detect serial port
    python plot_lidar.py /dev/cu.usbmodem14201   # specify port
    python plot_lidar.py --file data.csv         # plot from saved CSV
"""

import sys
import math
import time
import serial
import serial.tools.list_ports
import numpy as np
import matplotlib.pyplot as plt


def find_port():
    """Auto-detect the Arduino serial port."""
    ports = serial.tools.list_ports.comports()
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


def load_csv(path):
    """Load previously saved CSV data."""
    data = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or line.startswith("yaw"):
                continue
            parts = line.split(",")
            if len(parts) == 3:
                data.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return data


def save_csv(data, path="scan_data.csv"):
    with open(path, "w") as f:
        f.write("yaw_deg,pitch_deg,distance_mm\n")
        for yaw, pitch, dist in data:
            f.write(f"{yaw},{pitch},{dist}\n")
    print(f"Data saved to {path}")


def plot_topdown(data):
    """
    Project each (yaw, pitch, distance) point onto the horizontal plane
    and plot a top-down 2D map.

    Projection:
        horizontal_dist = distance * cos(pitch)
        x = horizontal_dist * sin(yaw)
        y = horizontal_dist * cos(yaw)
    """
    xs, ys, colors = [], [], []

    for yaw_deg, pitch_deg, dist_mm in data:
        if dist_mm == 0:
            continue  # skip invalid readings

        yaw   = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        dist_m = dist_mm / 1000.0  # convert to meters

        horiz = dist_m * math.cos(pitch)
        x = horiz * math.sin(yaw)
        y = horiz * math.cos(yaw)

        xs.append(x)
        ys.append(y)
        colors.append(dist_m)

    xs = np.array(xs)
    ys = np.array(ys)
    colors = np.array(colors)

    fig, ax = plt.subplots(figsize=(10, 10))
    sc = ax.scatter(xs, ys, c=colors, cmap="viridis_r", s=4, alpha=0.8)
    cbar = plt.colorbar(sc, ax=ax, shrink=0.7)
    cbar.set_label("Distance (m)")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("LIDAR Top-Down 2D Map")
    ax.set_aspect("equal")
    ax.plot(0, 0, "r+", markersize=15, markeredgewidth=2)  # sensor position
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig("lidar_map.png", dpi=150)
    print("Plot saved to lidar_map.png")
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
        data = run_scan(port)
        save_csv(data)

    if not data:
        print("No data collected.")
        sys.exit(1)

    plot_topdown(data)
