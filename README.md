# Jetson Capstone — Autonomous Trash-Picking Robot

Roomba/Wall-E style autonomous robot built on a Jetson Orin Nano. Patrols a room, detects trash using YOLO vision, approaches and collects objects, returns to deposit location. Accessible via web dashboard over WiFi.

---

## Hardware

| Component | Details |
|---|---|
| Main computer | Jetson Orin Nano, JetPack 6.2, Ubuntu 22.04 |
| Microcontroller | STM32 Nucleo-L476RG |
| Motors | 4x Pololu 131:1 37D gearmotors with 64 CPR encoders |
| Motor drivers | 2x Cytron MDD10A (dual channel) |
| Arm servos | Hiwonder HTD-45H serial bus servos, 2-DOF |
| Camera | USB webcam on /dev/video0 |
| Power | 3S LiPo (9.9V–12.6V) |
| Drive | Mecanum wheel (4-wheel holonomic) |

---

## Software Stack

- ROS2 Humble (Ubuntu 22.04)
- Python 3.10, CUDA 12.6, TensorRT 10.3.0
- Ultralytics YOLO — YOLOv2 6n exported to TensorRT FP16

---

## ROS2 Packages

### yolo_detector
Reads webcam, runs TensorRT inference at ~15fps, publishes detections and annotated image. Decoupled capture/inference threading prevents stutter — capture thread always overwrites latest frame, inference timer drops ticks if previous inference is still running.

### state_machine
Robot brain. States: IDLE, PATROL, APPROACH, COLLECT, DEPOSIT, FETCH, TELEOP. Patrol wanders randomly and interrupts to approach detected trash. Approach uses P-control on bounding box horizontal error. Teleop has a 0.5s watchdog that stops the robot if no command arrives (protects against browser disconnect).

### drive_node
Converts `/cmd_vel` Twist into wheel speed setpoints using mecanum inverse kinematics. Normalises against max linear speed so pure forward = 100% on all wheels. Proportionally scales down if combined commands exceed limits.

### stm32_bridge
Translates `/wheel_speeds` into UART binary packets to the STM32, and receives encoder odometry back. Node is written and ready — not yet added to launch (waiting on STM32 firmware flash + UART wiring).

### robot_dashboard
Web dashboard served on port 8888. Uses roslibjs + rosbridge (port 9090) for ROS communication, MJPEG stream from web_video_server (port 8080) for camera feed.

**Dashboard features:**
- Live camera feed
- State badge (colour-coded per state)
- Battery voltage + bar (needs STM32 wiring)
- E-stop button
- Command buttons: PATROL / FETCH / TELEOP / IDLE
- Teleop dpad + WASD keyboard, speed slider (5–100%), auto-switches to TELEOP on first keypress
- Vision card: detection FPS, object count, confidence, per-object list
- Wheel speed bars (centre-anchored, green/red)
- System graph: 60s rolling CPU/GPU/RAM chart
- Log: /rosout feed with warn/error colouring

### system_monitor
Publishes `/system_stats` JSON at 1Hz (CPU, RAM, GPU, temperature). GPU polled at 10Hz with EMA smoothing (α=0.15) to handle irregular sysfs update timing.

---

## Access

- **Dashboard**: http://nano.local:8888
- **SSH**: `ssh justin@nano.local`
- **Static IP**: 192.168.1.74 (home WiFi)

---

## Autostart

Systemd service (`robot.service`) launches everything on boot via `~/start_robot.sh`. Starts after `network-online.target` with a 10s delay to let networking settle.

---

## What's Done

- [x] YOLO TensorRT inference pipeline (decoupled threading, ~15fps)
- [x] State machine: PATROL, APPROACH, COLLECT, DEPOSIT, FETCH, TELEOP, IDLE
- [x] Mecanum drive node with proper normalisation
- [x] STM32 bridge node (UART protocol, pending hardware)
- [x] STM32 firmware (pending flash)
- [x] Web dashboard (camera, state, teleop, vision, wheel speeds, system stats, log)
- [x] Teleop watchdog safety feature
- [x] Responsive dashboard layout (CSS clamp)
- [x] Systemd autostart (boot ordering bug fixed)

## What's Next

- [ ] Flash STM32 firmware, wire UART
- [ ] Test motor control end-to-end, tune PWM scaling
- [ ] Wire battery voltage sense (STM32 ADC → UART → /battery_voltage)
- [ ] Wheel odometry node
- [ ] Field-oriented teleop (rotate commands into robot frame using heading)
- [ ] Arm node (HTD-45H servos)
- [ ] Nav2 / room mapping integration
