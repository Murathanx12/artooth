# Alfred V3 — Changes & Tuning Guide

## What Changed from V2

### Simplified Steering: Rotation-Only (no strafing in auto)

- **Old way (V2)**: PID controller + strafe correction (`vy` component) for small errors. Complex omni-directional moves on curves caused the robot to lose the line on sharp turns.
- **New way (V3)**: Simple proportional omega steering with aggressive curve slowdown. On sharp curves the robot behaves like pressing **W gently + Q/E hard** — it crawls forward while rotating strongly. No lateral strafing (`vy = 0` always during auto).
- **Why**: Rotation-only steering is smoother and more predictable. The robot stays on the line through sharp curves instead of overshooting with complex vector moves.

### Curve Slowdown System

The key innovation in V3. Forward speed drops dramatically on curves using a power curve:

```
speed_scale = 1.0 - (1.0 - CURVE_SLOW_FACTOR) * (turn_ratio ^ CURVE_SLOW_EXPO)
```

With defaults (`CURVE_SLOW_FACTOR=0.20`, `CURVE_SLOW_EXPO=1.5`):
- 30% turn → 84% forward speed (barely slows)
- 50% turn → 60% forward speed (noticeable braking)
- 70% turn → 41% forward speed (significant slowdown)
- 100% turn → 20% forward speed (crawling + max rotation)

This means the robot is fast on straights but patient on curves — the #1 key to smooth line following.

### Recovery Simplified

- **Lost reverse**: Straight backward (no strafe), then pivot
- **Lost pivot**: Uses `moveSidePivot()` (pivots around rear axle) instead of vector rotation — more reliable for recentering

### Enhanced GUI (700x580, two-column layout)

- **Left column**: Sensor arc, turn indicator bar, stats cards (speed/algo/dist), vector numeric readout
- **Right column**: Vector field visualization (radar-style with live arrow + rotation arc), throttle gauge
- **Key improvements**:
  - Vector visualization and throttle now work in auto mode (debug values updated in all FSM states)
  - Smooth animated vector arrow (lerp between values)
  - Sensor glow effects on active sensors
  - Pill-shaped mode/state indicators in header bar
  - All elements properly sized for readability

---

## Tuning Guide

All parameters are at the top of `Minilab5/linefollower.py`.

### Steering & Curve Handling (most important)

| Parameter | Default | What it does | Increase if... | Decrease if... |
|-----------|---------|-------------|----------------|----------------|
| `OMEGA_GAIN` | 4.5 | Rotation strength per unit of turn error | Robot slow to react to curves | Robot oscillates/overshoots on the line |
| `CURVE_SLOW_FACTOR` | 0.20 | Min forward speed at max turn (as fraction) | Robot stalls on tight curves | Robot too fast through sharp curves |
| `CURVE_SLOW_EXPO` | 1.5 | How quickly speed drops (>1 = gentle curves stay fast) | Speed drops too early on gentle curves | Speed drops too late on sharp curves |

**Tuning order:**
1. Start with `OMEGA_GAIN=4.5`. If the robot can't follow sharp curves, increase to 5.5–6.0.
2. If the robot overshoots/oscillates, decrease `OMEGA_GAIN` or decrease `CURVE_SLOW_FACTOR` (slower on curves).
3. If gentle curves feel too slow, increase `CURVE_SLOW_EXPO` (e.g., 2.0).

### Speed Tuning

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `DEFAULT_SPEED` | 35 | Master speed multiplier (0-5 scale to motor %) |
| `ACCEL` | 0.10 | How fast speed ramps up per tick |
| `DECEL` | 0.07 | How fast speed ramps down per tick |

### Delivery Zone & Recovery

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `DELIVERY_ZONE_TIME_THRESHOLD` | 0.4s | How long all-5-ON must persist to count as delivery zone |
| `PARKING_DRIVE_TIME` | 0.8s | How far to crawl into zone after detection |
| `PARKING_SPEED` | 15 | Crawl speed into delivery zone |
| `REVERSE_PSEUDO_DIST_MAX` | 0.18 | How far to reverse before switching to pivot |
| `LOST_PSEUDO_DIST_MAX` | 0.15 | How far to coast before declaring line lost |
| `RECOVERY_PIVOT_SPEED` | 80 | Pivot motor speed during recovery |

### Quick Start Settings

- **Conservative (stable tray):** `DEFAULT_SPEED=25, OMEGA_GAIN=3.0, CURVE_SLOW_FACTOR=0.15`
- **Balanced (good speed + stability):** `DEFAULT_SPEED=35, OMEGA_GAIN=4.5, CURVE_SLOW_FACTOR=0.20`
- **Aggressive (fastest time):** `DEFAULT_SPEED=50, OMEGA_GAIN=6.0, CURVE_SLOW_FACTOR=0.30`
