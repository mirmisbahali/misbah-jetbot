# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a differential drive robot (JetBot) control system using:
- **Jetson Nano** as the main compute board running ROS2 Iron
- **Raspberry Pi Pico** as the motor controller (PWM via UART bridge)
- **MX1508** motor driver for 2 DC motors

The Pico is used specifically because the Jetson Nano only has 2 hardware PWM pins, which is insufficient for direct motor control.

## Development Environment

Development uses VS Code Dev Containers with two separate container configurations:

- **Jetbot container** (`.devcontainer/jetbot/`): For onboard Jetson Nano code. Base: `dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04`, runs with `--privileged --network=host --runtime=nvidia`. Mounts `ros2_ws/jetbot_ws` → `/jetbot_ws`.
- **Host container** (`.devcontainer/host/`): For visualization/planning. Base: `osrf/ros:humble-desktop`, runs with `--privileged --network=host`. Mounts `ros2_ws/host_ws` → `/host_ws`.

**Important**: VS Code 1.85 is required for Jetson Nano compatibility (Jetpack 4.6).

## ROS2 Workspaces

### Jetbot workspace (`ros2_ws/jetbot_ws/`) — ROS Jazzy
```bash
# Build
colcon build

# Source workspace
source install/setup.bash

# Run nodes
ros2 run motor_controller uart_controller
ros2 run motor_controller simple_jetbot_controller
ros2 run pico_bridge bridge

# Lint/test (ament tooling)
colcon test
colcon test-result --verbose
```

### Host workspace (`ros2_ws/host_ws/`) — ROS Humble
```bash
colcon build
source install/setup.bash
```

## Architecture

### Communication Flow
```
Teleop keyboard → /cmd_vel (Twist) → pico_bridge node
                                          ↓ UART /dev/ttyTHS1 (115200 baud)
                                      Pico (main.py)
                                          ↓ PWM pins 2,3,4,5
                                      MX1508 driver → Motors
```

### ROS2 Packages (jetbot_ws)

**`motor_controller`** — Two motor control implementations:
- `simple_jetbot_controller`: Direct GPIO control using Jetson.GPIO (BOARD pin numbering). Left motor: pins 32, 35. Right motor: pins 33, 37.
- `uart_controller`: Sends verbose velocity strings over UART (legacy format, not compatible with Pico's parser).

**`pico_bridge`** — Primary Pico communication bridge:
- Subscribes to `/cmd_vel`, converts to differential drive, sends `"L=0.500 R=0.250\n"` over serial.
- ROS parameters: `port` (default: `/dev/ttyTHS1`), `baud` (default: 115200), `scale` (default: 1.0), `rate` (default: 30.0 Hz).
- Differential drive math: `left = linear_x - angular_z`, `right = linear_x + angular_z`, clamped to [-1.0, 1.0].
- Sends `"L=0 R=0\n"` stop command on shutdown.

### Pico Firmware (`pico/firmware/motor_driver/`)

Written in MicroPython (`.micropico` project). Main file is `main.py`:
- Parses `"L=0.500 R=0.250\n"` format from UART.
- Motor A (left): PWM pins 2, 3. Motor B (right): PWM pins 4, 5.
- PWM frequency: 1 kHz. Safety timeout: stops motors after 1 second with no command.
- Supporting files: `motor_test.py`, `uart.py`, `blink.py` for testing.

## Hardware Notes

- **UART port**: `/dev/ttyTHS1` (Jetson Nano UART2, physical pins 8 and 10)
- **Serial access**: User must be in `dialout` group (handled in Dockerfile)
- **GPU access**: User must be in `video` group (handled in Dockerfile)
- The `uart_controller` node sends a different string format than what the Pico expects — use `pico_bridge` for actual Pico communication.
