# Alfred — Mecanum Line Follower Robot

## Overview

Alfred is a **4-wheeled mecanum robot** that can follow black lines on a white surface. It uses a **split-brain architecture**: an **ESP32-S3** microcontroller handles low-level motor control and sensor reading, while a **Raspberry Pi** runs the high-level decision-making (line-following algorithm + manual control GUI). They communicate over **UART at 115200 baud**.

## Hardware Layout

```
       [N]    ← center front IR sensor
      /   \
   [NW]   [NE]  ← diagonal IR sensors
   /         \
 [W]         [E]  ← side IR sensors (far left / far right)
  |           |
  A --------- B   ← front mecanum wheels (left / right)
  |           |
  C --------- D   ← rear mecanum wheels (left / right)
```

- **5 IR sensors** on a semicircular arc at the front, reading black-line presence (1 = line, 0 = no line)
- **4 mecanum wheels** enabling omnidirectional movement: forward/backward, strafing, tank rotation, differential curves, and side pivots
- **MCU:** ESP32-S3-DevKitM-1
- **SBC:** Raspberry Pi (connected via UART at 115200 baud)
- **Sensors:** 5 IR sensors on GPIO 5, 6, 7, 15, 45

## Project Structure

### 1. `src/main.cpp` — ESP32-S3 Firmware (PlatformIO/Arduino)

This is the **actuator layer**. It does three things:

- **Reads IR sensors** at 20 Hz and broadcasts `IR_STATUS:<bitmask>` over UART to the Pi. The bitmask is 5 bits: `bit0=W, bit1=NW, bit2=N, bit3=NE, bit4=E`.
- **Receives movement commands** over UART in `command:params` format and drives the motors accordingly.
- **Sends heartbeat pings** every 1.5s.

Supported commands:

| Command | Params | Action |
|---------|--------|--------|
| `mv_fwd` | speed | All wheels forward |
| `mv_rev` | speed | All wheels backward |
| `mv_left` | speed | Mecanum strafe left |
| `mv_right` | speed | Mecanum strafe right |
| `mv_turnleft` | speed | Tank rotate CCW (left side backward, right side forward) |
| `mv_turnright` | speed | Tank rotate CW |
| `mv_curve` | left,right | Differential steering — independent speed per side, negative = reverse |
| `mv_vector` | vx,vy,omega | **Mecanum inverse kinematics** — forward, strafe, and rotation in one command |
| `mv_sidepivot` | frontSpeed,rearPercent,direction | Front wheels oppose hard, rear wheels crawl — pivots around rear axle |
| `stop` | 0 | All motors stop |

Speed values are 0–100 percent, mapped internally to PWM range 50–200.

#### Mecanum Inverse Kinematics (`mv_vector`)

The `mv_vector` command takes three values (-100 to 100 each):
- `vx` = forward (+) / backward (-)
- `vy` = strafe right (+) / strafe left (-)
- `omega` = rotate CW (+) / rotate CCW (-)

Each wheel speed is computed as:
```
FL (Motor A) = vx + vy + omega
FR (Motor B) = vx - vy - omega
RL (Motor C) = vx - vy + omega
RR (Motor D) = vx + vy - omega
```
If any wheel exceeds 100, all are scaled down proportionally to preserve the motion vector.

### 2. `Minilab5/linefollower.py` — Raspberry Pi Controller (Python + Pygame)

This is the **brain**. It runs a pygame GUI (700x580, two-column layout) with two modes:

**Manual Mode (default):** WASD for forward/reverse/strafe, QE for rotation, 1/2 to adjust speed, Space to stop. All keys are combinable for true omnidirectional control.

**Auto Mode (press M):** A finite state machine (FSM) that follows a black line using proportional rotation steering with adaptive curve slowdown.

## The Line-Following Algorithm (FSM)

The auto mode has **6 states**:

### `STATE_FOLLOWING` — Normal line tracking (rotation + adaptive speed)

- Reads the 5 IR sensor bits and computes **`turn_var`** — weighted average of active sensors using `TURN_STRENGTHS = [-7, -4.5, 0, 4.5, 7]`. Negative = line is left, positive = line is right, zero = centered.
- **Omega steering**: `turn_var` is scaled by `OMEGA_GAIN` to produce rotation. On sharp curves this acts like pressing Q/E — the robot pivots in place while crawling forward.
- **Curve slowdown**: Forward speed is reduced based on turn severity using `CURVE_SLOW_FACTOR` and `CURVE_SLOW_EXPO`. On straights the robot runs at full speed; on sharp curves it drops to 20% forward speed with strong rotation.
- **No lateral strafing** during auto following (`vy = 0` always). This keeps the movement smooth and predictable — rotation-only steering is more reliable on tight curves than combined strafe+rotate.
- **Adaptive speed** automatically goes fast on straights (center sensor only = 5.0) and slows on curves (outer sensors = 2.0–3.0).
- `internal_speed` (0.0–5.0) ramps toward target using acceleration/deceleration constants.
- If **all 5 sensors** fire → transitions to `STATE_ENDPOINT`.
- If **no sensors** fire beyond `LOST_PSEUDO_DIST_MAX` → transitions to `STATE_LOST_REVERSE`.

### `STATE_ENDPOINT` — Junction / delivery zone handling

- Robot drives straight at reduced speed (`ENDPOINT_TARGET_SPEED = 2.0`).
- A **timer** tracks how long all 5 sensors stay on:
  - Short all-on (< 0.4s) = junction crossbar → drives through, returns to `STATE_FOLLOWING`.
  - Sustained all-on (>= 0.4s) = delivery zone → transitions to `STATE_PARKING`.
- If all sensors go dark → `STATE_LOST_REVERSE`.

### `STATE_PARKING` — Delivery zone parking

- Crawls forward at low speed (`PARKING_SPEED = 15`) for `PARKING_DRIVE_TIME` (0.8s) to center all wheels in the zone.
- Then stops permanently and disables auto mode.

### `STATE_LOST_REVERSE` — Backtrack recovery

- Drives backward at `RECOVERY_REVERSE_SPEED = 20` for a pseudo-distance threshold.
- Once line found or distance met → `STATE_LOST_PIVOT`.

### `STATE_LOST_PIVOT` — Re-center on line

- Uses `moveSidePivot()` to pivot around the rear axle in the last known turn direction until the line is recentered (turn strength within ±5.0).
- Then returns to `STATE_FOLLOWING`.

### `STATE_STOPPED` — Graceful halt

- Ramps `internal_speed` down to zero before locking motors, preventing abrupt stops.

## Communication Flow

```
Pi (linefollower.py)                    ESP32 (main.cpp)
       |                                      |
       |  <-- "IR_STATUS:12\n" (20 Hz) ----  |  (sensor broadcast)
       |                                      |
       |  ---- "mv_vector:30,5,12\n" ----->  |  (vector drive command)
       |                                      |
       |  <-- "Hello from ESP32\n" --------  |  (heartbeat)
```

## Threading Model (Pi side)

- **Main thread**: Pygame event loop at 60 FPS. Handles keyboard input, renders the debug GUI, and calls `line_follow_step()` every tick (rate-limited to 30 Hz internally).
- **UART thread**: Daemon thread continuously reading serial. Parses `IR_STATUS:` messages and updates the shared `ir_status` variable (protected by `ir_lock`). Also sends periodic pings.

## Key Tuning Parameters

### Steering & Curve Handling (most important)

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `OMEGA_GAIN` | 4.5 | How hard it rotates into curves | Robot slow to react to curves | Robot oscillates/overshoots |
| `CURVE_SLOW_FACTOR` | 0.20 | Min forward speed at max turn (20%) | Robot stops on curves | Robot too fast through curves |
| `CURVE_SLOW_EXPO` | 1.5 | How quickly speed drops with turn severity | Speed drops too late | Speed drops too early on gentle curves |

### Speed Tuning

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `DEFAULT_SPEED` | 35 | Master speed scalar |
| `ACCEL` | 0.10 | Speed ramp-up per tick |
| `DECEL` | 0.07 | Speed ramp-down per tick |

### Delivery Zone & Recovery

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `DELIVERY_ZONE_TIME_THRESHOLD` | 0.4s | How long all-5-ON must persist to count as delivery zone |
| `PARKING_DRIVE_TIME` | 0.8s | How far to crawl into zone after detection |
| `REVERSE_PSEUDO_DIST_MAX` | 0.18 | How far to reverse before switching to pivot |
| `LOST_PSEUDO_DIST_MAX` | 0.15 | How far to coast before declaring lost |

### Quick Start Presets

- **Conservative (stable tray):** `DEFAULT_SPEED=25, OMEGA_GAIN=3.0, CURVE_SLOW_FACTOR=0.15`
- **Balanced (good speed + stability):** `DEFAULT_SPEED=35, OMEGA_GAIN=4.5, CURVE_SLOW_FACTOR=0.20`
- **Aggressive (fastest time):** `DEFAULT_SPEED=50, OMEGA_GAIN=6.0, CURVE_SLOW_FACTOR=0.30`

## Controls (Raspberry Pi GUI)

| Key | Action |
|-----|--------|
| W/S | Forward / Reverse |
| A/D | Strafe left / right (manual only) |
| Q/E | Tank rotate left / right |
| M | Toggle auto (line following) / manual mode |
| 1/2 | Decrease / increase speed |
| Space | Stop |
| Esc | Quit |

## Setup

### ESP32 Firmware

1. Install [PlatformIO](https://platformio.org/)
2. Open this project folder
3. Build and upload:
   ```
   pio run --target upload
   ```

### Raspberry Pi

1. Install dependencies:
   ```
   pip install pyserial pygame
   ```
2. Run the controller:
   ```
   python Minilab5/linefollower.py
   ```

---

## Changelog

### V3 (2026-03-26) — Smooth Curve Handling + Enhanced GUI

**Movement — Smooth rotation-only steering:**
- Replaced PID + strafe correction with simple proportional omega steering — more reliable on sharp curves
- **Curve slowdown**: forward speed drops to 20% on sharp turns (configurable via `CURVE_SLOW_FACTOR` / `CURVE_SLOW_EXPO`), robot crawls + rotates hard like pressing W gently + Q/E hard
- No lateral strafing during auto mode (`vy=0`) — rotation-only is smoother and more predictable
- Recovery uses `moveSidePivot()` for reliable re-centering

**GUI — Two-column layout (700x580):**
- **Vector field visualization** — radar-style display showing live vx/vy arrow + rotation arc (CW/CCW), works in both manual and auto modes
- **Throttle gauge** — color-coded speed bar showing algo speed percentage, live in auto mode
- Sensor glow effects, pill-shaped mode/state indicators, card-based stats
- Smooth animated vector arrow (lerp between values)
- All debug values (vx, vy, omega) updated in every FSM state

### V2 (2026-03-26) — PID + Mecanum Vector Drive + Parking

**ESP32 Firmware:**
- Added `mv_vector:vx,vy,omega` command with mecanum inverse kinematics
- Added `mv_sidepivot` command for rear-axle pivot recovery

**Pi Controller:**
- Delivery zone parking — time-based detection (all-5-ON > 0.4s)
- Pseudo-distance integration for lost/endpoint detection
- Adaptive speed — fast on straights, slow on curves
- Enhanced GUI with color-coded FSM state

### V1 (2026-03-25) — Initial Release

- 5-sensor weighted fusion with differential steering
- Frame-based lost detection with debounce
- Reverse + pivot recovery
- Gate pattern detection (11011, 10001)
- Manual WASD+QE control with Pygame GUI
