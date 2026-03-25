# Alfred V2 — Changes & Tuning Guide

## What Changed

### Bug Fixes
1. **`lost_frame_counter` crash** — Was used but never initialized. Now replaced with time-based `lost_start_time`.
2. **60Hz vs 30Hz mismatch** — Lost detection used `* 60` but control runs at 30Hz. Now uses `time.time()` directly — no more tick-rate dependency.
3. **Dead code removed** — `LOST_DETECTION_FRAMES` was declared but never used. Removed.

### New: Mecanum Vector Drive (ESP32)
- Added `mv_vector:vx,vy,omega` command — sends forward speed, strafe speed, and rotation in one command.
- The ESP32 now does proper **mecanum inverse kinematics**: each wheel gets an independent speed calculated from the 3 velocity components.
- All old commands (`mv_fwd`, `mv_curve`, etc.) still work. Nothing was removed.

### New: PID Controller (replaces old differential steering)
- **Old way**: Weighted average → delta_v → 1.3x outer wheel boost. No memory of past errors.
- **New way**: PID controller with Proportional, Integral, and Derivative terms.
  - **P** reacts to current error (how far off the line)
  - **I** accumulates past error (fixes steady drift on curves)
  - **D** reacts to rate of change (stops oscillation, keeps tray stable)

### New: Strafe Correction (the mecanum advantage)
- **Small errors** (line slightly off-center): robot **slides sideways** instead of rotating. No heading change = no tray sway.
- **Medium errors**: blend of strafe + rotation.
- **Large errors** (line far off): pure rotation (like before, but PID-controlled).
- This is the #1 reason mecanum wheels are better than normal wheels for this task.

### New: Delivery Zone Parking
- **Old way**: Looked for gate patterns (11011, 10001) with exact float comparison.
- **New way**: Times how long all 5 sensors stay on.
  - Short all-on (< 0.4s) = junction crossbar → drive through.
  - Long all-on (>= 0.4s) = delivery zone → crawl forward to center, then stop permanently.

### New: Strafe Recovery
- **Old way**: Lost line → reverse → pivot. Slow and jerky.
- **New way**: Lost line → strafe sideways (toward last known line direction) for 0.4s first. If that doesn't find it, *then* reverse → pivot.
- Much gentler recovery = less tray sway.

### New: Adaptive Speed
- Robot automatically speeds up on straights and slows down on curves based on which sensors are active.
- Center sensor only = full speed. Outer sensors = slow down.

### New: Omega Rate Limiter
- Rotation speed can only change by a limited amount per tick. Prevents sudden jerky turns.

### New: Enhanced GUI
- Color-coded FSM state (green=following, yellow=endpoint, red=lost, blue=parking)
- Visual sensor boxes (green=line detected, gray=no line)
- PID P/I/D breakdown display
- Vector output (vx, vy, omega) display
- Delivery zone timer

---

## Tuning Guide

All parameters are at the top of `Minilab5/linefollower.py`. Here's what to change and when:

### PID Tuning (most important)

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `KP` | 18.0 | How hard it steers toward the line | Robot is slow to react to curves | Robot oscillates/wobbles on the line |
| `KI` | 0.5 | Fixes steady drift on sustained curves | Robot drifts to outside of curves | Robot overshoots after exiting curves |
| `KD` | 12.0 | Dampens oscillation | Robot oscillates on straights | Robot feels sluggish entering curves |
| `PID_I_MAX` | 50.0 | Caps how much I can build up | Unlikely to need change | I term causes overshoot after long curves |

**Tuning order:**
1. Set `KI=0`, `KD=0`. Increase `KP` until robot follows line but wobbles.
2. Increase `KD` until wobble is gone.
3. Add small `KI` (0.1-1.0) until curve drift is fixed.
4. If tray sways, increase `KD`.

### Strafe Tuning

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `STRAFE_GAIN` | 8.0 | How hard it strafes sideways | Robot slow to center on straights | Robot jitters sideways |
| `STRAFE_WEIGHT_THRESHOLD` | 3.0 | Error below this = pure strafe | Want more strafe, less rotation | Robot doesn't turn enough on curves |
| `STRAFE_BLEND_RANGE` | 2.0 | Transition zone from strafe to rotation | Want smoother transition | Want sharper switchover |

### Speed Tuning

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `DEFAULT_SPEED` | 35 | Master speed multiplier (0-5 scale to motor %) | Robot too slow overall | Tray unstable or overshooting |
| `ACCEL` | 0.20 | How fast speed ramps up per tick | Too slow to accelerate | Jerky acceleration |
| `DECEL` | 0.12 | How fast speed ramps down per tick | Too slow to brake for curves | Braking feels abrupt |
| `OMEGA_RATE_LIMIT` | 5.0 | Max rotation change per tick | Turns feel sluggish | Tray sways on corrections |

### Delivery Zone & Recovery

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `DELIVERY_ZONE_TIME_THRESHOLD` | 0.4s | How long all-5-ON must persist to count as delivery zone | Robot parks at junctions by mistake | Robot drives through delivery zone |
| `PARKING_DRIVE_TIME` | 0.8s | How far to crawl into zone after detection | Wheels not fully inside zone | Robot drives past zone |
| `PARKING_SPEED` | 15 | Crawl speed into delivery zone | Not reaching center in time | Moving too fast while parking |
| `LOST_DETECTION_DELAY` | 0.5s | How long no-line before declaring lost | False lost triggers on small gaps | Takes too long to start recovery |
| `STRAFE_SEARCH_DURATION` | 0.4s | How long to strafe-search before reversing | Need more search time | Wastes time before reversing |
| `STRAFE_SEARCH_SPEED` | 30 | Strafe speed during recovery search | Search not covering enough ground | Too aggressive for tray |

### Quick Start Settings
- **Conservative (stable tray):** `DEFAULT_SPEED=25, KP=12, KD=15, OMEGA_RATE_LIMIT=3`
- **Balanced (good speed + stability):** `DEFAULT_SPEED=35, KP=18, KD=12, OMEGA_RATE_LIMIT=5`
- **Aggressive (fastest time):** `DEFAULT_SPEED=50, KP=24, KD=10, OMEGA_RATE_LIMIT=8`
