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

This is the **brain**. It runs a pygame GUI with two modes:

**Manual Mode (default):** WASD for movement, QE for tank rotation, 1/2 to adjust speed, Space to stop.

**Auto Mode (press M):** A finite state machine (FSM) that follows a black line using PID control, mecanum strafe correction, and adaptive speed.

## The Line-Following Algorithm (FSM)

The auto mode has **7 states**:

### `STATE_FOLLOWING` — Normal line tracking (PID + strafe + vector drive)

- Reads the 5 IR sensor bits and computes **`turn_var`** — weighted average of active sensors using `TURN_STRENGTHS = [-7, -4.5, 0, 4.5, 7]`. Negative = line is left, positive = line is right, zero = centered.
- A **PID controller** processes `turn_var` to produce `omega` (rotation):
  - **P** (proportional) = immediate steering response
  - **I** (integral) = eliminates drift on sustained curves
  - **D** (derivative) = dampens oscillation, critical for tray stability
- **Strafe correction** adds a `vy` component for small errors — the robot slides sideways instead of rotating, keeping heading aligned and the tray stable. For larger errors, rotation takes over.
- **Adaptive speed** automatically goes fast on straights (center sensor only = 5.0) and slows on curves (outer sensors = 2.0–3.0).
- `internal_speed` (0.0–5.0) ramps toward target using acceleration/deceleration constants.
- An **omega rate limiter** caps how quickly rotation can change per tick, preventing sudden jerky turns.
- All three components (`vx`, `vy`, `omega`) are sent via `moveVector()` to use the full mecanum capability.
- If **all 5 sensors** fire → transitions to `STATE_ENDPOINT`.
- If **no sensors** fire for `LOST_DETECTION_DELAY` (0.5s) → transitions to `STATE_LOST_STRAFE`.

### `STATE_ENDPOINT` — Junction / delivery zone handling

- Robot drives straight at reduced speed (`ENDPOINT_TARGET_SPEED = 2.0`).
- A **timer** tracks how long all 5 sensors stay on:
  - Short all-on (< 0.4s) = junction crossbar → drives through, returns to `STATE_FOLLOWING`.
  - Sustained all-on (>= 0.4s) = delivery zone → transitions to `STATE_PARKING`.
- If all sensors go dark → `STATE_LOST_STRAFE`.

### `STATE_PARKING` — Delivery zone parking

- Crawls forward at low speed (`PARKING_SPEED = 15`) for `PARKING_DRIVE_TIME` (0.8s) to center all wheels in the zone.
- Then stops permanently and disables auto mode.

### `STATE_LOST_STRAFE` — Strafe recovery (first attempt)

- Strafes sideways in the last known line direction for 0.4s with slight forward motion.
- If the line is found → returns to `STATE_FOLLOWING`.
- If not found → falls back to `STATE_LOST_REVERSE`.
- Much gentler than immediately reversing — less tray sway.

### `STATE_LOST_REVERSE` — Backtrack recovery

- Drives backward at `RECOVERY_REVERSE_SPEED = 20` until any sensor detects the line again.
- Once line found → `STATE_LOST_PIVOT`.

### `STATE_LOST_PIVOT` — Re-center on line

- Pivots in place (using `last_turn_var` memory to pick direction) until the computed turn strength is within ±3.0 (roughly centered).
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

### PID Tuning (most important)

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `KP` | 18.0 | Main steering response | Robot slow to react to curves | Robot oscillates/wobbles |
| `KI` | 0.5 | Fixes steady drift on curves | Robot drifts to outside of curves | Robot overshoots after curves |
| `KD` | 12.0 | Dampens oscillation | Robot oscillates on straights | Robot sluggish entering curves |
| `PID_I_MAX` | 50.0 | Anti-windup clamp for I term | Unlikely to need change | I term causes overshoot |

**Tuning order:** Set `KI=0, KD=0`. Increase `KP` until it follows but wobbles. Add `KD` until wobble is gone. Add small `KI` to fix curve drift. If tray sways, increase `KD`.

### Strafe & Speed Tuning

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `STRAFE_GAIN` | 8.0 | How hard it strafes sideways for small corrections |
| `STRAFE_WEIGHT_THRESHOLD` | 3.0 | Error below this = pure strafe (no rotation) |
| `OMEGA_RATE_LIMIT` | 5.0 | Max rotation change per tick (tray stability) |
| `DEFAULT_SPEED` | 35 | Master speed scalar |
| `ACCEL / DECEL` | 0.20 / 0.12 | Ramp rates for smooth speed transitions |

### Delivery Zone & Recovery

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `DELIVERY_ZONE_TIME_THRESHOLD` | 0.4s | How long all-5-ON must persist to count as delivery zone |
| `PARKING_DRIVE_TIME` | 0.8s | How far to crawl into zone after detection |
| `LOST_DETECTION_DELAY` | 0.5s | Debounce before declaring line lost |
| `STRAFE_SEARCH_DURATION` | 0.4s | How long to strafe-search before falling back to reverse |

### Quick Start Presets

- **Conservative (stable tray):** `DEFAULT_SPEED=25, KP=12, KD=15, OMEGA_RATE_LIMIT=3`
- **Balanced (good speed + stability):** `DEFAULT_SPEED=35, KP=18, KD=12, OMEGA_RATE_LIMIT=5`
- **Aggressive (fastest time):** `DEFAULT_SPEED=50, KP=24, KD=10, OMEGA_RATE_LIMIT=8`

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

### V2 (2026-03-26) — PID + Mecanum Vector Drive + Parking

**Bug Fixes:**
- Fixed `lost_frame_counter` crash (was used but never initialized)
- Fixed 60Hz vs 30Hz lost detection mismatch (now time-based)
- Removed dead `LOST_DETECTION_FRAMES` code

**ESP32 Firmware:**
- Added `mv_vector:vx,vy,omega` command with mecanum inverse kinematics
- Added `vectorDrive()` and `driveWheel()` functions
- All existing commands unchanged

**Pi Controller — New Features:**
- **PID controller** replaces old 1.3x outer-wheel-boost differential steering
- **Strafe correction** — small errors use sideways sliding instead of rotation (tray stays stable)
- **Delivery zone parking** — time-based detection (all-5-ON > 0.4s), crawls to center, stops permanently
- **Strafe recovery** — tries lateral search before falling back to reverse+pivot
- **Adaptive speed** — automatically fast on straights, slow on curves
- **Omega rate limiter** — caps rotation change per tick for tray stability
- **Enhanced GUI** — color-coded FSM state, visual sensor boxes, PID breakdown, vector output display, delivery zone timer

### V1 (2026-03-25) — Initial Release

- 5-sensor weighted fusion with differential steering
- Frame-based lost detection with debounce
- Reverse + pivot recovery
- Gate pattern detection (11011, 10001)
- Manual WASD+QE control with Pygame GUI
