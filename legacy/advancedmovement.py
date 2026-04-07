import time
import serial
import threading
import signal
import sys
import pygame

# ===========================================================================
# TUNABLE PARAMETERS — Algorithmic Navigation
# ===========================================================================

# -- Sensor Configuration --
REVERSE_SENSOR_ORDER = False
LOST_DETECTION_DELAY = 0.5      # Seconds of no-line before declaring LOST

# -- Algorithm Weights (W, NW, N, NE, E) --
TURN_STRENGTHS = [-7, -4.5, 0.0, 4.5, 7]
MOVE_STRENGTHS = [3.8, 4.2, 5.0, 4.2, 3.8]

# -- Speed & Physics Limits --
MIN_SPEED       = 10       # lowest usable motor speed
MAX_SPEED       = 150      # highest motor speed
DEFAULT_SPEED   = 35       # Base multiplier setting

ACCEL = 0.10  # Speed gained per tick
DECEL = 0.07  # Speed lost per tick
MAX_TURN_STRENGTH = 9.0

# -- Vector Steering Gains (replaces PID) --
OMEGA_GAIN = 4.0           # How aggressively to rotate for a given turn_var
OMEGA_BOOST = 1.3          # Extra boost on the turning side (from working code)

# -- Recovery & Junction Parameters --
RECOVERY_REVERSE_SPEED = 20
RECOVERY_PIVOT_SPEED   = 80     # Front wheel speed for sidepivot recovery
RECOVERY_PIVOT_REAR_PERCENT = 15  # Rear wheels as % of front during pivot recovery
ENDPOINT_TARGET_SPEED  = 2.0   # Out of 5.0

# -- Pseudo-distance thresholds (speed_units x seconds, no encoder) --
REVERSE_PSEUDO_DIST_MAX  = 0.18   # How far to reverse before switching to pivot
ENDPOINT_PSEUDO_DIST_MAX = 0.4
LOST_PSEUDO_DIST_MAX     = 0.15

# -- Delivery zone parking --
DELIVERY_ZONE_TIME_THRESHOLD = 0.4  # Seconds of sustained all-5-ON = delivery zone
PARKING_DRIVE_TIME = 0.8            # Seconds to drive forward into zone
PARKING_SPEED = 15                  # Slow crawl into the zone

# -- UART --
SERIAL_PORT     = '/dev/ttyAMA2'
BAUD_RATE       = 115200
PING_INTERVAL   = 5

# ===========================================================================
# Globals & State Machine
# ===========================================================================
running   = True
auto_mode = False

ir_status = 0
ir_lock   = threading.Lock()

current_speed = DEFAULT_SPEED   # Master scalar for the 0-5 algorithmic speeds

# FSM States
STATE_FOLLOWING    = 0
STATE_ENDPOINT     = 1
STATE_LOST_REVERSE = 2
STATE_LOST_PIVOT   = 3
STATE_STOPPED      = 4
STATE_PARKING      = 5

auto_state     = STATE_FOLLOWING
internal_speed = 0.0  # Tracks the 0.0 to 5.0 algorithmic speed
last_turn_var  = 0.0  # Memory for which way we were turning before getting lost
turn_var       = 0.0

# -- Pseudo-distance accumulator --
pseudo_dist      = 0.0
last_step_time   = 0.0   # Wall-clock time of the previous line_follow_step call

# -- Delivery zone parking --
all_on_start_time  = None
parking_start_time = None

# Debug values for GUI
debug_vx = 0
debug_vy = 0
debug_omega = 0

STATE_NAMES = {
    STATE_FOLLOWING:    "FOLLOW",
    STATE_ENDPOINT:     "ENDPOINT",
    STATE_LOST_REVERSE: "LOST_REV",
    STATE_LOST_PIVOT:   "LOST_PIVOT",
    STATE_STOPPED:      "STOPPED",
    STATE_PARKING:      "PARKING",
}

# ---------------------------------------------------------------------------
# UART & Movement Functions
# ---------------------------------------------------------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def sendSerialCommand(command_name, params):
    param_str   = ",".join(map(str, params))
    command_str = f"{command_name}:{param_str}\n"
    ser.write(command_str.encode())

def moveForward(speed):  sendSerialCommand('mv_fwd',       [speed])
def moveReverse(speed):  sendSerialCommand('mv_rev',       [speed])
def moveLeft(speed):     sendSerialCommand('mv_left',      [speed])
def moveRight(speed):    sendSerialCommand('mv_right',     [speed])
def moveTurnLeft(speed): sendSerialCommand('mv_turnleft',  [speed])
def moveTurnRight(speed):sendSerialCommand('mv_turnright', [speed])
def stopAll():           sendSerialCommand('stop',         [0])
def moveCurve(left_speed, right_speed): sendSerialCommand('mv_curve', [left_speed, right_speed])
def moveVector(vx, vy, omega): sendSerialCommand('mv_vector', [int(vx), int(vy), int(omega)])

def moveSidePivot(front_speed, rear_percent, direction):
    sendSerialCommand('mv_sidepivot', [front_speed, rear_percent, direction])

# ---------------------------------------------------------------------------
# IR Sensor Helpers
#   Sensor output: 1 = black line detected, 0 = white / no line
#
#         [N]  North (center front)
#        /   \
#     [NW]   [NE]
#     /         \
#   [W]         [E]
#
#   bit0 = GPIO5  = W   (West / far left)
#   bit1 = GPIO6  = NW  (Northwest)
#   bit2 = GPIO7  = N   (North / center front)
#   bit3 = GPIO15 = NE  (Northeast)
#   bit4 = GPIO45 = E   (East / far right)
# ---------------------------------------------------------------------------
IDX_W, IDX_NW, IDX_N, IDX_NE, IDX_E = 0, 1, 2, 3, 4

def get_ir_bits():
    with ir_lock:
        v = ir_status
    if REVERSE_SENSOR_ORDER:
        return [(v >> (4 - i)) & 1 for i in range(5)]
    return [(v >> i) & 1 for i in range(5)]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _enter_state(new_state):
    """Transition to a new FSM state, resetting the pseudo-distance accumulator."""
    global auto_state, pseudo_dist
    auto_state  = new_state
    pseudo_dist = 0.0

def _accumulate_pseudo_dist(dt, speed_fraction):
    global pseudo_dist
    pseudo_dist += abs(speed_fraction) * dt
    return pseudo_dist

# ---------------------------------------------------------------------------
# ALGORITHMIC LINE FOLLOWING FSM
# ---------------------------------------------------------------------------
def line_follow_step():
    global auto_state, internal_speed, last_turn_var, current_speed, turn_var
    global last_step_time, pseudo_dist
    global all_on_start_time, parking_start_time, auto_mode
    global debug_vx, debug_vy, debug_omega

    # --- Per-tick timing (used for pseudo-distance integration) ---
    now = time.monotonic()
    dt  = now - last_step_time if last_step_time > 0 else 0.0
    last_step_time = now

    bits         = get_ir_bits()
    active_count = sum(bits)
    pattern      = (bits[IDX_E] << 4) | (bits[IDX_NE] << 3) | (bits[IDX_N] << 2) | (bits[IDX_NW] << 1) | bits[IDX_W]

    # Speed multiplier (maps 0.0-5.0 scale to actual motor speed)
    multiplier = current_speed / 5.0

    # -----------------------------------------------------------------------
    # 1. STATE: STOPPED / BRAKING
    # -----------------------------------------------------------------------
    if auto_state == STATE_STOPPED:
        if internal_speed > 0:
            internal_speed = max(0.0, internal_speed - DECEL * 5)
            vx = int(round(internal_speed * multiplier))
            moveVector(vx, 0, 0)
        else:
            stopAll()
        return

    # -----------------------------------------------------------------------
    # 2. STATE: PARKING (delivery zone)
    # -----------------------------------------------------------------------
    if auto_state == STATE_PARKING:
        elapsed = time.monotonic() - parking_start_time
        if elapsed < PARKING_DRIVE_TIME:
            moveVector(PARKING_SPEED, 0, 0)
        else:
            stopAll()
            auto_mode = False
            print(">>> PARKED. ATTEMPT COMPLETE.")
        return

    # -----------------------------------------------------------------------
    # 3. STATE: LOST — REVERSING TO FIND LINE
    # -----------------------------------------------------------------------
    if auto_state == STATE_LOST_REVERSE:
        rev_frac = RECOVERY_REVERSE_SPEED / MAX_SPEED
        _accumulate_pseudo_dist(dt, rev_frac)

        if pseudo_dist >= REVERSE_PSEUDO_DIST_MAX:
            print(f">>> LOST_REV → PIVOT (dist={pseudo_dist:.2f})")
            _enter_state(STATE_LOST_PIVOT)
        elif active_count > 0:
            print(f">>> LOST_REV → PIVOT (line reappeared early, dist={pseudo_dist:.2f})")
            _enter_state(STATE_LOST_PIVOT)
        else:
            moveVector(-RECOVERY_REVERSE_SPEED, 0, 0)
        return

    # -----------------------------------------------------------------------
    # 4. STATE: LOST — PIVOTING TO RECENTER
    # -----------------------------------------------------------------------
    if auto_state == STATE_LOST_PIVOT:
        if active_count > 0:
            curr_turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            if abs(curr_turn_var) < 5.0:
                print(f">>> LOST_PIVOT → FOLLOW (turn_var={curr_turn_var:.2f})")
                _enter_state(STATE_FOLLOWING)
                return

        direction = 1 if last_turn_var > 0 else -1
        moveSidePivot(RECOVERY_PIVOT_SPEED, RECOVERY_PIVOT_REAR_PERCENT, direction)
        return

    # -----------------------------------------------------------------------
    # 5. STATE: ENDPOINT HANDLING
    # -----------------------------------------------------------------------
    if auto_state == STATE_ENDPOINT:
        # Accumulate distance driven while in the ENDPOINT state
        speed_frac = internal_speed / 5.0
        _accumulate_pseudo_dist(dt, speed_frac)

        if active_count == 5:
            # Still all-on — check if this is a delivery zone (sustained)
            if all_on_start_time is None:
                all_on_start_time = time.monotonic()
            elapsed_all_on = time.monotonic() - all_on_start_time
            if elapsed_all_on >= DELIVERY_ZONE_TIME_THRESHOLD:
                auto_state = STATE_PARKING
                parking_start_time = time.monotonic()
                all_on_start_time = None
                print(">>> DELIVERY ZONE DETECTED — PARKING")
                return

        # Trigger hard stop once the distance threshold is covered
        if pseudo_dist >= ENDPOINT_PSEUDO_DIST_MAX:
            _enter_state(STATE_STOPPED)
            all_on_start_time = None
            print(f">>> ENDPOINT DIST MET ({pseudo_dist:.2f}) -> HARD STOP")
            return

        if 0 < active_count < 5 and pattern not in [0b11011, 0b10001]:
            _enter_state(STATE_FOLLOWING)
            all_on_start_time = None
            print(">>> ENDPOINT CLEARED")
            return

        if active_count == 0:
            _enter_state(STATE_LOST_REVERSE)
            all_on_start_time = None
            return

        # Drive straight slowly through endpoint
        target_max_speed = ENDPOINT_TARGET_SPEED
        turn_var = 0.0

    # -----------------------------------------------------------------------
    # 6. STATE: NORMAL FOLLOWING
    # -----------------------------------------------------------------------
    if auto_state == STATE_FOLLOWING:
        if active_count == 0:
            # Accumulate distance while no line is detected
            speed_frac = internal_speed / 5.0
            _accumulate_pseudo_dist(dt, speed_frac)
            if pseudo_dist >= LOST_PSEUDO_DIST_MAX:
                _enter_state(STATE_LOST_REVERSE)
                internal_speed = 0.0
                print(f">>> LOST LINE -> RECOVERY after dist {pseudo_dist:.2f}")
            else:
                # Coast with last known steering while within distance threshold
                internal_speed = max(0.0, internal_speed - DECEL)
                coast_omega = (1 if last_turn_var > 0 else -1) * int(internal_speed * multiplier * 0.5)
                moveVector(int(internal_speed * multiplier), 0, coast_omega)
            return
        else:
            # Line visible — reset the pseudo_dist accumulator
            pseudo_dist = 0.0

        if active_count == 5:
            _enter_state(STATE_ENDPOINT)
            all_on_start_time = time.monotonic()
            print(">>> ALL SENSORS ON: ENTERING ENDPOINT")
            target_max_speed = ENDPOINT_TARGET_SPEED
            turn_var = 0.0
        else:
            turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            last_turn_var = turn_var

            if turn_var == 0.0:
                target_max_speed = 5.0
            else:
                target_max_speed = sum(b * m for b, m in zip(bits, MOVE_STRENGTHS)) / active_count

    # -----------------------------------------------------------------------
    # Shared Physics & Vector Steering  (Following & Endpoint)
    # -----------------------------------------------------------------------

    # Smooth acceleration / deceleration
    if internal_speed < target_max_speed:
        internal_speed = min(target_max_speed, internal_speed + ACCEL)
    elif internal_speed > target_max_speed:
        internal_speed = max(target_max_speed, internal_speed - DECEL)

    # Convert weighted steering to vector commands
    # vx = forward speed, omega = rotation from turn_var
    vx_raw = internal_speed * multiplier

    # Omega: proportional to turn_var, with boost factor for sharper turns
    turn_ratio = abs(turn_var) / MAX_TURN_STRENGTH * 2
    omega_raw = turn_var * OMEGA_GAIN * multiplier / 5.0

    # Boost the forward speed slightly when turning (like the 1.3 factor in working code)
    if abs(turn_var) > 0:
        vx_raw *= (1.0 + (OMEGA_BOOST - 1.0) * turn_ratio)

    # Reduce forward speed proportionally during sharp turns to tighten the arc
    vx_raw *= max(0.3, 1.0 - turn_ratio * 0.5)

    vx = max(-MAX_SPEED, min(MAX_SPEED, int(round(vx_raw))))
    omega = max(-MAX_SPEED, min(MAX_SPEED, int(round(omega_raw))))

    debug_vx, debug_vy, debug_omega = vx, 0, omega
    moveVector(vx, 0, omega)


# ---------------------------------------------------------------------------
# UART Thread & Pygame Boilerplate
# ---------------------------------------------------------------------------
def uart_thread():
    global running, ir_status
    last_ping = time.monotonic()
    while running:
        line = ser.readline()
        text = line.decode(errors='ignore').strip()
        if text:
            if text.startswith("IR_STATUS:"):
                try:
                    _, value_str = text.split(":", 1)
                    with ir_lock:
                        ir_status = int(value_str) & 0x1F
                except ValueError:
                    pass

        now = time.monotonic()
        if now - last_ping >= PING_INTERVAL:
            ser.write(b"hello from pi\n")
            last_ping = now

def handle_sigint(sig, frame):
    global running
    running = False
    stopAll()
    sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)

    t_uart = threading.Thread(target=uart_thread, daemon=True)
    t_uart.start()

    pygame.init()
    screen = pygame.display.set_mode((480, 420))
    pygame.display.set_caption("Alfred Line Follower v2")
    font = pygame.font.SysFont(None, 26)
    font_sm = pygame.font.SysFont(None, 22)

    pressed = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False}

    def update_movement():
        if auto_mode: return
        w, s = pressed['w'], pressed['s']
        a, d = pressed['a'], pressed['d']
        q, e = pressed['q'], pressed['e']

        # Vector-based manual control: combine forward/strafe/rotation simultaneously
        vx = 0
        vy = 0
        omega = 0

        if w: vx += current_speed
        if s: vx -= current_speed
        if a: vy -= current_speed   # strafe left
        if d: vy += current_speed   # strafe right
        if q: omega -= current_speed  # rotate CCW
        if e: omega += current_speed  # rotate CW

        if vx == 0 and vy == 0 and omega == 0:
            stopAll()
        else:
            moveVector(vx, vy, omega)

    clock = pygame.time.Clock()

    while running:
        screen.fill((30, 30, 30))
        mode_color = (0, 220, 100) if auto_mode else (220, 180, 0)
        mode_label = "AUTO (Vector)" if auto_mode else "MANUAL (WASD+QE)"

        bits = get_ir_bits()
        active = sum(bits)
        turn_display = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active if active > 0 else 0.0

        # State color coding
        state_colors = {
            STATE_FOLLOWING: (0, 220, 100),    # Green
            STATE_ENDPOINT: (220, 220, 0),     # Yellow
            STATE_LOST_REVERSE: (220, 50, 50), # Red
            STATE_LOST_PIVOT: (220, 100, 50),  # Orange
            STATE_STOPPED: (150, 150, 150),    # Gray
            STATE_PARKING: (50, 100, 220),     # Blue
        }
        st_color = state_colors.get(auto_state, (180, 220, 180))

        y = 10
        screen.blit(font.render(f"MODE: {mode_label}", True, mode_color), (20, y)); y += 30
        screen.blit(font.render(f"State: {STATE_NAMES.get(auto_state, '?')}", True, st_color), (20, y)); y += 30
        screen.blit(font.render(f"Speed: {current_speed}  Algo: {internal_speed:.2f}", True, (200,200,200)), (20, y)); y += 28

        # Sensor display — visual boxes
        sensor_names = ['W', 'NW', 'N', 'NE', 'E']
        sx = 20
        for i, name in enumerate(sensor_names):
            color = (0, 200, 0) if bits[i] else (80, 80, 80)
            pygame.draw.rect(screen, color, (sx, y, 40, 25))
            screen.blit(font_sm.render(name, True, (255,255,255)), (sx + 5, y + 3))
            sx += 50
        y += 35

        screen.blit(font_sm.render(f"Turn Var: {turn_display:+.2f}", True, (100,180,255)), (20, y)); y += 22

        # Vector output
        screen.blit(font_sm.render(f"Vector  vx:{debug_vx}  vy:{debug_vy}  omega:{debug_omega}", True, (180,255,180)), (20, y)); y += 22

        # Pseudo-distance
        screen.blit(font_sm.render(f"Pseudo-dist: {pseudo_dist:.2f}", True, (180,180,100)), (20, y)); y += 22

        # Delivery zone timer
        if all_on_start_time is not None:
            zone_t = time.monotonic() - all_on_start_time
            screen.blit(font_sm.render(f"All-ON timer: {zone_t:.2f}s / {DELIVERY_ZONE_TIME_THRESHOLD}s", True, (50,100,220)), (20, y))
        y += 22

        screen.blit(font_sm.render("Keys: M=auto  WASD+QE=manual  1/2=speed  Esc=quit", True, (120,120,120)), (20, y)); y += 20
        screen.blit(font_sm.render("Combos: W+A=diag-fwd-left  W+Q=fwd+rotate  etc.", True, (120,120,120)), (20, y))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    auto_mode = not auto_mode
                    pressed   = {k: False for k in pressed}
                    stopAll()
                    if auto_mode:
                        _enter_state(STATE_FOLLOWING)
                        internal_speed  = 0.0
                        last_step_time  = 0.0
                elif event.key == pygame.K_w:      pressed['w'] = True;  update_movement()
                elif event.key == pygame.K_s:      pressed['s'] = True;  update_movement()
                elif event.key == pygame.K_a:      pressed['a'] = True;  update_movement()
                elif event.key == pygame.K_d:      pressed['d'] = True;  update_movement()
                elif event.key == pygame.K_q:      pressed['q'] = True;  update_movement()
                elif event.key == pygame.K_e:      pressed['e'] = True;  update_movement()
                elif event.key == pygame.K_SPACE:  pressed = {k: False for k in pressed}; stopAll()
                elif event.key == pygame.K_ESCAPE: running = False
                elif event.key == pygame.K_1:
                    current_speed = max(MIN_SPEED, current_speed - 5); update_movement()
                elif event.key == pygame.K_2:
                    current_speed = min(MAX_SPEED, current_speed + 5); update_movement()
            elif event.type == pygame.KEYUP:
                if   event.key == pygame.K_w: pressed['w'] = False; update_movement()
                elif event.key == pygame.K_s: pressed['s'] = False; update_movement()
                elif event.key == pygame.K_a: pressed['a'] = False; update_movement()
                elif event.key == pygame.K_d: pressed['d'] = False; update_movement()
                elif event.key == pygame.K_q: pressed['q'] = False; update_movement()
                elif event.key == pygame.K_e: pressed['e'] = False; update_movement()

        if auto_mode:
            line_follow_step()

        clock.tick(60)

    stopAll()
    pygame.quit()
