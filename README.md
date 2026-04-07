# Alfred — Mecanum Line Follower Robot

> A 4-wheeled mecanum robot with omnidirectional movement, 5-sensor line following, and a real-time debug GUI. Built with ESP32-S3 + Raspberry Pi.

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--S3-orange?logo=platformio)](https://platformio.org/)
[![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python)](https://python.org/)
[![License](https://img.shields.io/badge/License-MIT-green)](#license)

---

## Features

- **Omnidirectional movement** — mecanum wheels enable forward, strafe, rotate, and any combination simultaneously
- **5-sensor weighted line following** — semicircular IR array with proportional rotation steering and adaptive curve slowdown
- **Finite state machine** — 6 states: follow, endpoint detection, delivery zone parking, lost recovery (reverse + pivot), and graceful stop
- **Real-time debug GUI** — Pygame dashboard with vector field visualization, sensor arc display, throttle gauge, and live FSM state
- **Split-brain architecture** — ESP32-S3 handles motors & sensors at 20 Hz; Raspberry Pi runs the decision-making algorithm
- **Manual + auto modes** — full WASD+QE manual control with seamless toggle to autonomous line following
- **Delivery zone detection** — time-based all-sensor-on detection with automatic parking sequence
- **Tunable parameters** — all algorithm constants at the top of the file with documented presets

---

## Hardware

```
       [N]    <- center front IR sensor
      /   \
   [NW]   [NE]  <- diagonal IR sensors
   /         \
 [W]         [E]  <- side IR sensors
  |           |
  A --------- B   <- front mecanum wheels
  |           |
  C --------- D   <- rear mecanum wheels
```

| Component | Details |
|-----------|---------|
| **MCU** | ESP32-S3-DevKitM-1 |
| **SBC** | Raspberry Pi (any model with UART) |
| **Motors** | 4x mecanum wheel motors (PWM controlled) |
| **Sensors** | 5x IR line sensors on GPIO 5, 6, 7, 15, 45 |
| **Communication** | UART at 115200 baud |

---

## Project Structure

```
alfred/
├── src/main.cpp              # ESP32-S3 firmware (PlatformIO/Arduino)
├── Minilab5/linefollower.py  # Raspberry Pi controller (Python + Pygame)
├── legacy/                   # Earlier versions for reference
│   ├── advancedmovement.py   # V1 advanced movement experiments
│   ├── advancedv2.py         # V2 with PID controller
│   ├── esp.cpp               # Earlier ESP32 firmware
│   └── Pi.py                 # Earlier Pi controller
├── CHANGELOG_V2.md           # Detailed V2→V3 changes and tuning guide
└── platformio.ini            # PlatformIO build configuration
```

---

## Quick Start

### ESP32 Firmware

```bash
# Install PlatformIO CLI or use the VS Code extension
pip install platformio

# Build and upload
pio run --target upload
```

### Raspberry Pi Controller

```bash
# Install dependencies
pip install pyserial pygame

# Run the controller
python Minilab5/linefollower.py
```

> **Note:** The Pi connects to ESP32 via UART on `/dev/ttyAMA2`. Change `SERIAL_PORT` in `linefollower.py` if your setup differs.

---

## How It Works

### Architecture

```
Raspberry Pi                          ESP32-S3
┌────────────────────┐     UART      ┌──────────────────┐
│  Line Following    │◄────────────► │  Motor Control   │
│  Algorithm (FSM)   │  115200 baud  │  Sensor Reading  │
│  Pygame Debug GUI  │               │  Heartbeat       │
└────────────────────┘               └──────────────────┘
        │                                    │
   Decision-making                    IR_STATUS:XX (20 Hz)
   mv_vector:vx,vy,omega             5-bit sensor bitmask
```

### ESP32 Commands

| Command | Params | Action |
|---------|--------|--------|
| `mv_fwd` | speed | All wheels forward |
| `mv_rev` | speed | All wheels backward |
| `mv_left` | speed | Mecanum strafe left |
| `mv_right` | speed | Mecanum strafe right |
| `mv_turnleft` | speed | Tank rotate CCW |
| `mv_turnright` | speed | Tank rotate CW |
| `mv_curve` | left,right | Differential steering per side |
| `mv_vector` | vx,vy,omega | Mecanum inverse kinematics (main command) |
| `mv_sidepivot` | front,rear%,dir | Pivot around rear axle |
| `stop` | 0 | All motors stop |

Speed values are 0–100%, mapped internally to PWM 50–200.

### Mecanum Inverse Kinematics (`mv_vector`)

Takes three values (-100 to 100): `vx` (forward/back), `vy` (strafe), `omega` (rotation).

```
FL = vx + vy + omega
FR = vx - vy - omega
RL = vx - vy + omega
RR = vx + vy - omega
```

All wheels are scaled proportionally if any exceeds 100.

---

## Line Following Algorithm

The autonomous mode runs a **6-state finite state machine**:

```
                    ┌─────────────┐
                    │  FOLLOWING   │◄──────────────────────┐
                    └──────┬──────┘                        │
                           │                               │
              ┌────────────┼────────────┐                  │
              ▼            ▼            ▼                  │
        ┌──────────┐ ┌──────────┐ ┌──────────┐           │
        │ ENDPOINT │ │  LOST    │ │ STOPPED  │           │
        └─────┬────┘ │ REVERSE  │ └──────────┘           │
              │       └─────┬────┘                        │
              ▼             ▼                              │
        ┌──────────┐ ┌──────────┐                         │
        │ PARKING  │ │  LOST    ├─────────────────────────┘
        └──────────┘ │  PIVOT   │
                     └──────────┘
```

| State | What it does |
|-------|-------------|
| **FOLLOWING** | Proportional omega steering with adaptive curve slowdown. Fast on straights, crawls through curves. |
| **ENDPOINT** | All 5 sensors on — drives through junctions, detects delivery zones (sustained > 0.4s). |
| **PARKING** | Crawls forward to center in delivery zone, then stops permanently. |
| **LOST_REVERSE** | Backs up to last known position using pseudo-distance. |
| **LOST_PIVOT** | Pivots around rear axle in last turn direction until line is recentered. |
| **STOPPED** | Graceful deceleration before locking motors. |

### Curve Slowdown (Key Innovation)

Forward speed drops on curves using a power function:

```
speed_scale = 1.0 - (1.0 - CURVE_SLOW_FACTOR) * (turn_ratio ^ CURVE_SLOW_EXPO)
```

| Turn severity | Forward speed | Behavior |
|:---:|:---:|---|
| 30% | 84% | Barely slows — gentle curve |
| 50% | 60% | Noticeable braking |
| 70% | 41% | Significant slowdown |
| 100% | 20% | Crawling + maximum rotation |

---

## Debug GUI

The Pygame dashboard (700x580) provides real-time visualization:

| Panel | Description |
|-------|-------------|
| **Sensor Arc** | 5 IR sensors with glow effects on active sensors |
| **Turn Bar** | Color-coded turn indicator (green/yellow/red) |
| **Stats Cards** | Speed, algorithm speed, pseudo-distance |
| **Vector Field** | Radar-style display with live vx/vy arrow + rotation arc |
| **Throttle Gauge** | Color-coded speed bar (green → yellow → red) |
| **FSM State** | Pill indicator showing current state with color coding |

### Controls

| Key | Action |
|-----|--------|
| **W/S** | Forward / Reverse |
| **A/D** | Strafe left / right (manual only) |
| **Q/E** | Rotate left / right |
| **M** | Toggle auto / manual mode |
| **1/2** | Decrease / increase speed |
| **Space** | Emergency stop |
| **Esc** | Quit |

All keys are combinable for true omnidirectional manual control.

---

## Tuning Guide

All parameters are at the top of `Minilab5/linefollower.py`.

### Steering (most impactful)

| Parameter | Default | Increase if... | Decrease if... |
|-----------|:-------:|----------------|----------------|
| `OMEGA_GAIN` | 4.5 | Robot slow to react to curves | Robot oscillates/overshoots |
| `CURVE_SLOW_FACTOR` | 0.20 | Robot stops on curves | Robot too fast through curves |
| `CURVE_SLOW_EXPO` | 1.5 | Speed drops too late | Speed drops too early |

### Presets

| Profile | `DEFAULT_SPEED` | `OMEGA_GAIN` | `CURVE_SLOW_FACTOR` | Use case |
|---------|:---:|:---:|:---:|---|
| Conservative | 25 | 3.0 | 0.15 | Carrying fragile payload |
| Balanced | 35 | 4.5 | 0.20 | General purpose |
| Aggressive | 50 | 6.0 | 0.30 | Speed competition |

### Tuning Order

1. Start with balanced preset
2. If the robot can't follow sharp curves → increase `OMEGA_GAIN` to 5.5–6.0
3. If it overshoots/oscillates → decrease `OMEGA_GAIN` or `CURVE_SLOW_FACTOR`
4. If gentle curves feel too slow → increase `CURVE_SLOW_EXPO` to 2.0

---

## Communication Protocol

```
Pi → ESP32:    "mv_vector:30,0,12\n"     (drive command)
ESP32 → Pi:    "IR_STATUS:12\n"           (sensor data, 20 Hz)
ESP32 → Pi:    "Hello from ESP32\n"       (heartbeat, 1.5s)
```

### Threading (Pi side)

- **Main thread** — Pygame loop at 60 FPS, keyboard input, GUI rendering, FSM step (rate-limited to 30 Hz)
- **UART thread** — Daemon thread reading serial, parsing `IR_STATUS:` messages, updating shared state with lock

---

## Changelog

### V3 — Smooth Curve Handling + Enhanced GUI
- Replaced PID + strafe with proportional omega steering — more reliable on sharp curves
- Curve slowdown system: forward speed drops to 20% on sharp turns
- Two-column GUI with vector field visualization and throttle gauge
- Recovery uses `moveSidePivot()` for reliable re-centering

### V2 — PID + Mecanum Vector Drive + Parking
- Added `mv_vector` and `mv_sidepivot` ESP32 commands
- Delivery zone parking with time-based detection
- Pseudo-distance integration for lost/endpoint detection

### V1 — Initial Release
- 5-sensor weighted fusion with differential steering
- Reverse + pivot recovery, gate pattern detection
- Manual WASD+QE control with Pygame GUI

---

## Contributing

Contributions are welcome! Some ideas:

- **Encoder support** — replace pseudo-distance with real odometry
- **Camera integration** — computer vision for intersection handling
- **PID auto-tuning** — Ziegler-Nichols or relay method for omega gain
- **Web dashboard** — Flask/FastAPI based remote monitoring
- **Path recording** — record and replay routes

---

## License

MIT License. See [LICENSE](LICENSE) for details.

---

Built for Minilab5 robotics coursework. Uses ESP32-S3 + Raspberry Pi with mecanum drive.
