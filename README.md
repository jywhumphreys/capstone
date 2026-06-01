# jetson capstone — autonomous trash-picking robot

roomba/wall-e style robot on a jetson orin nano. patrols a room, detects trash with yolo vision, drives over and collects it with a small arm + gripper, drops it in an onboard hopper. mecanum drive for holonomic motion. accessible over wifi via a web dashboard.

two computers:
- **jetson** — vision, navigation, ros2, dashboard
- **stm32** — real-time motor/servo/actuator control (zephyr rtos), talks to the jetson over uart

---

## hardware

| component | details |
|---|---|
| main computer | jetson orin nano super dev kit (8gb), jetpack 6.2 |
| microcontroller | stm32 nucleo (f072rb; portable to l476rg / g474re) |
| drive | 4x pololu 131:1 37D gearmotors (64 CPR encoders), 2x cytron MDD10A, mecanum wheels |
| arm | 3x hiwonder HTD-45H serial bus servos (base/shoulder/elbow) |
| gripper | hiwonder HPS-2527 (270° pwm servo) |
| hopper | progressive automations PA-MC1 linear actuator via TB6612FNG |
| camera | logitech c920 (usb) |
| power | zeee 3s lipo, 5200mAh 50C 11.1V, xt60 |

---

## firmware — `zephyr/`

zephyr rtos app. the hardware mapping lives entirely in the devicetree overlay, so the same code runs on different nucleo boards by swapping the overlay — verified building on f072rb, l476rg and g474re.

subsystems (each bench-tested):
- **motors** — sign-magnitude mecanum drive, inverse kinematics, `motor_set` / `mecanum_drive`
- **gripper** — pwm servo, clamped to a calibrated open/closed band
- **hopper** — TB6612FNG dir-only control, timed move (no feedback, internal limit switches)
- **servo** — HTD-45H half-duplex bus protocol on usart3 (move / read position / set id / torque)
- **comms** — uart packet protocol to/from the jetson (drive in, odom out)

`main.c` has a `TEST_MODE` selector — per-subsystem bench tests plus a combined demo that runs all four in sequence with no jetson attached.

build/flash:
```
$env:ZEPHYR_BASE="<path>/zephyr"
west build -b nucleo_f072rb zephyr
west flash
```

---

## jetson — `ros2_ws/`

ros2 humble. packages:

- **yolo_detector** — c920 -> yolo (tensorrt fp16) at ~15fps. publishes `/detections` + annotated `/camera/image_raw`.
- **state_machine** — the brain: patrol / approach / collect / deposit / fetch / teleop / idle. subscribes `/detections`, `/state_cmd`, `/cmd_vel_teleop`; publishes `/cmd_vel`, `/robot_state`. teleop has a 0.5s watchdog.
- **drive_node** — `/cmd_vel` -> `/wheel_speeds` via mecanum inverse kinematics, normalised. params for wheel radius / wheelbase / max vel and per-wheel invert.
- **stm32_bridge** — `/wheel_speeds` -> uart packets on `/dev/ttyTHS1` @115200, encoders back on `/wheel_encoders`. written, not yet in the launch file.
- **robot_dashboard** — web ui on `:8888` (rosbridge `:9090`, web_video_server `:8080` for the mjpeg feed). `system_monitor` publishes `/system_stats`.

bring it all up (minus stm32_bridge):
```
ros2 launch robot_dashboard dashboard.launch.py
```

---

## autostart — `scripts/`

- **robot.service** — systemd unit, starts on boot after the network is up (10s settle, restart on failure)
- **start_robot.sh** — sources ros2 + the workspace and launches the dashboard

access: dashboard at `http://nano.local:8888`, ssh `justin@nano.local`.
