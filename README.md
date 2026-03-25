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

This is the **dumb actuator layer**. It does three things:

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
| `mv_sidepivot` | frontSpeed,rearPercent,direction | Front wheels oppose hard, rear wheels crawl — pivots around rear axle |
| `stop` | 0 | All motors stop |

Speed values are 0–100 percent, mapped internally to PWM range 50–200.

### 2. `Minilab5/linefollower.py` — Raspberry Pi Controller (Python + Pygame)

This is the **brain**. It runs a pygame GUI with two modes:

**Manual Mode (default):** WASD for movement, QE for tank rotation, 1/2 to adjust speed, Space to stop.

**Auto Mode (press M):** A finite state machine (FSM) that follows a black line using weighted sensor fusion and differential steering.

## The Line-Following Algorithm (FSM)

The auto mode has **5 states**:

### `STATE_FOLLOWING` — Normal line tracking

- Reads the 5 IR sensor bits and computes two values:
  - **`turn_var`** — weighted average of active sensors using `TURN_STRENGTHS = [-7, -4.5, 0, 4.5, 7]`. Negative = line is left, positive = line is right, zero = centered. This tells the robot *which direction to steer*.
  - **`target_max_speed`** — weighted average using `MOVE_STRENGTHS = [3.8, 4.2, 5.0, 4.2, 3.8]`. Center sensor gives max speed (5.0), side sensors give lower speed. This makes the robot *slow down on curves*.
- `internal_speed` (0.0–5.0) ramps toward `target_max_speed` using acceleration/deceleration constants (`ACCEL=0.20`, `DECEL=0.12`).
- **Differential steering**: `turn_ratio = |turn_var| / MAX_TURN_STRENGTH * 2` creates a speed difference (`delta_v`) between left and right wheels. The outer wheel gets a 1.3x boost for sharper turns. Final motor speeds are `internal_speed * multiplier` where `multiplier = current_speed / 5.0`.
- If **all 5 sensors** fire → transitions to `STATE_ENDPOINT`.
- If **no sensors** fire for `LOST_DETECTION_DELAY` (0.5s) worth of frames → transitions to `STATE_LOST_REVERSE`. During the delay, it keeps turning in the last known direction while slowing down.

### `STATE_ENDPOINT` — Junction / crossbar handling

- Robot goes straight at reduced speed (`ENDPOINT_TARGET_SPEED = 2.0`).
- If it sees gate patterns (`11011` or `10001`) at endpoint speed → hard stop (`STATE_STOPPED`).
- If sensor count drops back to 1–3 (normal track) → returns to `STATE_FOLLOWING`.
- If all sensors go dark → `STATE_LOST_REVERSE`.

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
       |  ---- "mv_curve:45,30\n" -------->  |  (motor command)
       |                                      |
       |  <-- "Hello from ESP32\n" --------  |  (heartbeat)
```

## Threading Model (Pi side)

- **Main thread**: Pygame event loop at 60 FPS. Handles keyboard input, renders the debug GUI, and calls `line_follow_step()` every tick (rate-limited to 30 Hz internally).
- **UART thread**: Daemon thread continuously reading serial. Parses `IR_STATUS:` messages and updates the shared `ir_status` variable (protected by `ir_lock`). Also sends periodic pings.

## Key Tuning Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `TURN_STRENGTHS` | [-7, -4.5, 0, 4.5, 7] | How aggressively each sensor pulls steering |
| `MOVE_STRENGTHS` | [3.8, 4.2, 5.0, 4.2, 3.8] | Target speed per sensor (center = fastest) |
| `ACCEL / DECEL` | 0.20 / 0.12 | Ramp rates for smooth speed transitions |
| `DEFAULT_SPEED` | 35 | Master speed scalar (multiplied with algo's 0–5 output) |
| `LOST_DETECTION_DELAY` | 0.5s | Debounce before declaring line lost |
| `MAX_TURN_STRENGTH` | 9.0 | Normalizer for differential steering ratio |

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
