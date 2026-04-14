# capstone robot — master progress document

---

## overview

autonomous trash-picking robot (roomba/wall-e style) built on a jetson orin nano. the robot patrols a room, detects trash-class objects using a yolo vision model running on the jetson's gpu, approaches and collects them, and returns to a deposit location. a secondary "fetch" mode can be used for dead-reckoning drives to fixed locations. everything is built on ros2 humble and accessible via a web dashboard over wifi.

---

## hardware

| component | details |
|---|---|
| main computer | jetson orin nano, jetpack 6.2 (jp36.5), ubuntu 22.04 |
| microcontroller | stm32 nucleo-l476rg |
| motors | 4x pololu 131:1 37D gearmotors with 64 CPR encoders |
| motor drivers | 2x cytron MDD10A (dual channel) |
| arm servos | hiwonder HTD-45H serial bus servos, 2-DOF |
| camera | usb webcam on /dev/video0 |
| power | 3s lipo (12.6V full, 11.1V nominal, 9.9V cutoff) |
| drive | mecanum wheel drive (4-wheel holonomic) |

### stm32 pin assignments
- **pwm outputs**: TIM1_CH1–4 on PA8–PA11
- **motor direction**: PB0, PB1, PC4, PC5
- **encoders**: TIM2 (PA0/PA1), TIM3 (PA6/PA7), TIM4 (PB6/PB7), TIM8 (PC6/PC7)
- **uart to jetson**: USART3 on PC10 (TX) / PC11 (RX)
- **jetson uart**: /dev/ttyTHS1 — 40-pin header pins 8 (TX) and 10 (RX)

---

## networking & access

- **static ip**: 192.168.1.74 (home wifi, connection: SpectrumSetup-3925)
- **campus hotspot**: saved connection named `justin`, auto-connects when in range
- **mdns**: `nano.local` resolves on any network via avahi-daemon — no need to know the ip
- **dashboard**: http://nano.local:8888
- **ssh**: `ssh justin@nano.local`

---

## software environment

- ros2 humble (ubuntu 22.04)
- python 3.10
- cuda 12.6
- tensorrt 10.3.0
- ultralytics yolo (python package)

### ld_library_path required for cuda/tensorrt
```
/usr/local/cuda-12.6/targets/aarch64-linux/lib
/usr/lib/aarch64-linux-gnu/nvidia
/usr/lib/aarch64-linux-gnu/tegra
```
set via `SetEnvironmentVariable` in the ros2 launch file — prepended to existing path, not replaced.

---

## ros2 workspace — ~/ros2_ws

built with `colcon build --symlink-install`. all packages are in `~/ros2_ws/src/`.

**important**: for python packages (ament_python), changes must be applied to both the source file (`src/`) and the build copy (`build/`), since the installed entry points run from the build directory. alternatively, rebuild the affected package:
```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select <package_name>
```

everything launches from a single launch file:
```
~/ros2_ws/src/robot_dashboard/launch/dashboard.launch.py
```

---

## package: yolo_detector

**purpose**: reads the webcam, runs yolo inference on the jetson gpu via tensorrt, publishes detections and the annotated image.

**model**: yolo v2 6n exported to tensorrt fp16 engine at `/home/justin/yolo26n.engine`. the 6n is a nano-sized model optimised for edge inference.

**inference pipeline**:
the original naive implementation ran `cap.read()` and then inference in the same ros2 timer callback, causing stutter because the opencv buffer queued stale frames while inference was running. the fixed architecture decouples them:

1. **capture thread** (`_capture_loop`): runs continuously in a daemon thread, calls `cap.read()` in a tight loop, always overwrites `self._frame` with the latest frame under a lock. `cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)` keeps opencv's internal buffer minimal.

2. **inference timer** (`_inference_callback`): fires at the target fps via a ros2 timer. copies the latest frame under the lock, runs yolo inference, publishes results. a non-blocking lock guard (`self._inferring` flag) drops the tick if the previous inference is still running — this prevents callbacks from queuing up when inference takes longer than one timer period.

**performance**: runs at ~15fps, which is the natural throughput limit of the engine on this hardware (~65–135ms per frame depending on scene complexity). the timer is set to 15fps to match.

**parameters**:
- `engine_path`: path to .engine file
- `camera_index`: default 0
- `confidence_threshold`: default 0.5
- `fps`: default 15.0

**published topics**:
- `/detections` — `vision_msgs/Detection2DArray`: bounding boxes, class ids, confidence scores
- `/camera/image_raw` — `sensor_msgs/Image`: annotated frame with boxes drawn

---

## package: state_machine

**purpose**: the robot's brain. subscribes to detections and commands, decides what the robot does, publishes velocity commands.

**states**:

| state | behaviour |
|---|---|
| idle | stop, wait for command |
| patrol | random wander scanning for trash |
| approach | p-control drive toward detected target |
| collect | stop and wait (arm placeholder) |
| deposit | reverse slightly (return to bin placeholder) |
| fetch | dead-reckon to fixed location (beer run mode) |
| teleop | pass through /cmd_vel_teleop with watchdog |

**patrol**: alternates between driving straight (`patrol_speed = 0.3 m/s`) for 2.5s and turning in a random direction (`patrol_turn_speed = 0.8 rad/s`) for 1.5s. immediately interrupts to approach if a trash-class object is detected.

**approach**: selects the highest-confidence trash detection. uses p-control on horizontal error (`kp = 1.2`) to steer toward it while driving forward. error is normalised to ±1.0 based on how far the target is from frame center. forward speed is reduced proportionally to the steering error (`vx = approach_speed * (1 - 0.5 * |error|)`). transitions to collect when the bounding box area exceeds 12% of the frame area.

**fetch**: optional yaw turn first (configurable angle), then drives straight at `fetch_speed` for `fetch_duration` seconds. designed for going to a fixed known location (e.g. a bin) by dead-reckoning.

**teleop watchdog**: when in teleop state, if no `/cmd_vel_teleop` message arrives within 0.5 seconds, the robot stops (but stays in teleop mode). this prevents runaway if the browser disconnects or the tab closes mid-drive. entering teleop also resets the stored twist to zero so there's no stale command from a previous session.

**trash coco classes**: bottle (39), wine glass (40), cup (41), fork (42), knife (43), spoon (44), bowl (45), book (73), scissors (76)

**logging**: uses a throttled logger (`_tlog`) that logs a given message key at most once per configurable period. this gives useful status updates without spamming at 20hz. logs state transitions immediately (unthrottled), patrol phase changes and direction, approach target/distance/error every 0.5s, fetch phase progress every 1s, teleop inputs every 0.5s.

**topics**:
- subscribes: `/detections`, `/state_cmd` (String), `/cmd_vel_teleop` (Twist)
- publishes: `/cmd_vel` (Twist), `/robot_state` (String at 20hz)

**nav2 upgrade path**: the `_tick_patrol()` method uses direct `cmd_vel` commands for random wander. to upgrade to nav2, swap those calls for `NavigateThroughPoses` action goals — the state transitions are identical.

---

## package: drive_node

**purpose**: converts velocity commands into wheel speed setpoints for the mecanum drivetrain.

**mecanum inverse kinematics**:
```
v_FL = (vx - vy - wz*(lx+ly)) / R
v_FR = (vx + vy + wz*(lx+ly)) / R
v_RL = (vx + vy - wz*(lx+ly)) / R
v_RR = (vx - vy + wz*(lx+ly)) / R
```
where `lx` and `ly` are half the wheelbase in x and y directions, `R` is wheel radius.

**normalisation**: wheel speeds are normalised against `max_lin / R` so that pure forward at max linear speed outputs exactly ±1.0 on all wheels. if any combined command (translation + rotation) pushes a wheel past ±1.0, all wheels are scaled down proportionally to preserve the drive ratio.

**parameters**: `wheel_radius` (0.05m), `wheelbase_x` (0.15m), `wheelbase_y` (0.15m), `max_lin_vel` (0.5 m/s), `max_ang_vel` (2.0 rad/s), per-wheel `invert_fl/fr/rl/rr` bools for wiring correction.

**topics**:
- subscribes: `/cmd_vel` (Twist)
- publishes: `/wheel_speeds` (Float32MultiArray — FL, FR, RL, RR in order, ±1.0)

---

## package: stm32_bridge

**purpose**: translates `/wheel_speeds` ros topic into uart binary packets to the stm32, and receives encoder odometry back.

**uart protocol**:
- **drive packet** (12 bytes, jetson → stm32): header, 4x int16 wheel speeds scaled to pwm range, xor checksum
- **odom packet** (20 bytes, stm32 → jetson): header, 4x int32 encoder counts, xor checksum
- baud rate: 115200

**status**: node written and ready. not yet added to the launch file — waiting on stm32 firmware to be flashed and uart wired up.

---

## package: stm32 firmware (~/stm32_firmware/)

written for stm32cubeide (cubemx project). files: `comms_protocol.h`, `motor_control.h/c`, `robot_comms.h/c`, `main_additions.c`.

key details:
- pwm generated on TIM1 channels 1–4 at appropriate frequency for the cytron drivers
- 16-bit encoder timers (TIM3/4/8) use overflow tracking via `HAL_TIM_PeriodElapsedCallback` to handle rollover correctly
- uart receive runs in interrupt mode, packet parser checks header and xor checksum before acting
- motor direction controlled by separate gpio pins (not pwm polarity)

**not yet flashed** — waiting to be done with physical hardware present.

---

## package: robot_dashboard

**web server**: simple python ros2 node serving static files from `web/` on port 8888. the entire dashboard is a single html file using:
- **roslibjs** + **rosbridge websocket** (port 9090) for all ros topic communication
- **mjpeg stream** from web_video_server (port 8080) for the camera feed
- **chart.js** for rolling time-series graphs

### layout
three-column layout:
- **column 1** (flexible): camera feed
- **column 2** (fluid ~20vw): all controls and data cards
- **column 3** (fluid ~22vw): log

responsive layout using css `clamp()` for column widths, padding, gaps, and chart height. root font size scales with viewport width (`clamp(11px, 1.1vw, 16px)`) so all rem-based sizes adapt proportionally.

### cards

**status**
- websocket connection dot (green/red)
- state badge — colour-coded: patrol=green, approach=orange, collect/deposit=blue, fetch=purple, teleop=red, idle=grey
- battery voltage display + colour-coded bar (green >50%, orange >20%, red otherwise). subscribes to `/battery_voltage` (std_msgs/Float32). shows "— V" until stm32 is wired.
- e-stop button: publishes zero twist to `/cmd_vel_teleop` and sends `idle` to `/state_cmd`

**commands**: patrol / fetch / teleop / idle — publish to `/state_cmd`

**teleop**
- 3×3 dpad grid: ↑↓←→ (wasd) + ↺↻ (q/e rotate)
- keyboard listener on the page: w/s = forward/back, a/d = strafe left/right, q/e = rotate left/right
- speed slider: 5–100%, where 100% = 0.5 m/s (max_lin_vel). default 80%.
- on first keypress: automatically sends `teleop` state command
- setInterval at 100ms: computes vx/vy/wz from held keys, publishes to `/cmd_vel_teleop`
- on key release: publishes zero twist
- dpad buttons use mousedown/mouseup/mouseleave events, same held-key logic

**vision**: detection fps, object count, best confidence %, live per-object detection list with class names and confidence. fps computed from topic message rate in js.

**wheel speeds**: four bars (FL/FR/RL/RR). each bar is centre-anchored — positive speed grows right (green), negative grows left (red). uses css transitions on both `width` and `left` simultaneously so the bar appears to grow outward from centre. colour only updates on non-zero values so it persists while the bar shrinks to zero (avoids green flash on release). max bar width is 50% of container so 100% speed fills exactly half.

**system graph**: 60-second rolling line chart. cpu (green), gpu (blue), ram (orange). gpu uses an exponential moving average (α=0.15, ~2-3s smoothing window) because the jetson gpu sysfs file (`/sys/devices/platform/bus@0/17000000.gpu/load`) updates on its own schedule, causing visible oscillation if read raw. polled at 10hz in a background thread. cpu/ram via psutil. temperature (max across all thermal zones) shown below the chart.

**log**: subscribes to `/rosout`. filters out rosbridge, web_video_server, and yolo_detector (too verbose). colours warn entries orange, error entries red. newest messages at bottom, capped at 200 entries.

---

## system monitor node

separate ros2 node (`system_monitor`) publishing `/system_stats` as a json string at 1hz:
```json
{"cpu": 18.4, "ram": 36.1, "gpu": 24.7, "temp": 53.2}
```
gpu is polled in a background thread at 10hz and smoothed with ema before publishing. all other stats read inline at publish time.

---

## autostart

`/etc/systemd/system/robot.service`:
```
After=network-online.target
Wants=network-online.target
ExecStartPre=/bin/sleep 10
ExecStart=/home/justin/start_robot.sh
Restart=on-failure
RestartSec=10
```

`~/start_robot.sh`:
```bash
#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/justin/ros2_ws/install/setup.bash
exec ros2 launch robot_dashboard dashboard.launch.py
```

**boot issue that was fixed**: originally had `After=jtop.service` which created a circular dependency (`jtop.service` itself has `After=multi-user.target`, and `robot.service` is `WantedBy=multi-user.target`). systemd detected the cycle and silently dropped the robot service job. fixed by removing the jtop dependency entirely (jtop was already unused — gpu is read via sysfs directly).

---

## what's next

### immediate (needs hardware)
- flash stm32 firmware
- wire uart: jetson 40-pin header pins 8/10 → stm32 PC10/PC11
- add `stm32_bridge` node to `dashboard.launch.py`
- test motor control end-to-end, tune pwm scaling
- wire battery voltage sense → stm32 adc → uart → `/battery_voltage` topic

### soon after
- wheel odometry node: integrate encoder ticks from stm32 into position + heading estimate
- field-oriented teleop: rotate commanded velocity into robot frame using current heading so "forward" is always world-forward regardless of robot orientation
- tune approach parameters (kp, stop_area) with real hardware

### later
- urdf from solidworks model (sw2urdf exporter → drop package into ros2_ws/src)
- arm node for HTD-45H serial bus servos
- room mapping / nav2 integration (state machine already designed for this swap)
