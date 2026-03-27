# Firmware Component

This directory contains the robot firmware for the Arduino Nano RP2040 Connect.

Main file:

- `main.cpp`

## Responsibilities

- connect the board to Wi-Fi
- connect the robot TCP client to the computer on port `9000`
- control the yaw and pitch steppers
- read the VL53L1X distance sensor
- stream scan points and robot state back to the server
- accept server commands such as `START_SCAN`

## Hardware Assumptions

- board: Arduino Nano RP2040 Connect
- sensor: VL53L1X
- motors: 2 steppers, one for yaw and one for pitch

Configured scan-home angles in the current code:

- yaw home: `0 deg`
- pitch home: `-60 deg`

## Important Config

The `Config` namespace in `main.cpp` defines:

- Wi-Fi SSID and password
- server hostname and fallback IP
- TCP port
- motor pins
- step geometry and gear ratios
- scan angle defaults
- sensor timeout and recovery behavior

## Robot Command Surface

Current recognized commands include:

- `START_SCAN`
- `START_SCAN,<degrees>`
- `STOP_SCAN`
- `HARD_STOP`
- `RELEASE_MOTORS`

The board accepts newline-delimited commands from the TCP connection.

## Robot Telemetry

The firmware emits line-based messages such as:

- `HELLO,...`
- `HEARTBEAT,...`
- `STATE,...`
- `POSE,...`
- `POINT,...`
- `FRAME_BEGIN,...`
- `FRAME_END,...`
- `COMMAND_ACK,...`
- `COMMAND_FAIL,...`
- `SENSOR_TIMEOUT,...`

## Sensor Recovery

The distance sensor has an explicit recovery path.

Current behavior:

- a sensor timeout or repeated invalid state can trigger recovery
- recovery retries use exponential backoff
- recovery probes the sensor before reporting success
- if recovery fails, the scan aborts instead of hanging forever

## Build And Upload

From the repo root:

```bash
/Users/haysoncheung/.platformio/penv/bin/pio run -e nanorp2040connect
```

Upload:

```bash
/Users/haysoncheung/.platformio/penv/bin/pio run -e nanorp2040connect -t upload
```

## Serial Monitor

```bash
/Users/haysoncheung/.platformio/penv/bin/pio device monitor -b 115200
```

Use the serial monitor when debugging:

- Wi-Fi connection failures
- hostname or fallback IP issues
- sensor initialization failures
- scan-time sensor recovery failures
