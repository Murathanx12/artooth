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
LOST_DETECTION_DELAY = 0.5

# -- Algorithm Weights (W, NW, N, NE, E) --
TURN_STRENGTHS = [-7, -4.5, 0.0, 4.5, 7]
MOVE_STRENGTHS = [3.8, 4.2, 5.0, 4.2, 3.8]

# -- Speed & Physics Limits --
MIN_SPEED       = 10       # lowest usable motor speed
MAX_SPEED       = 120     # highest motor speed
DEFAULT_SPEED   = 35      # Base multiplier setting (Algorithm outputs 0-5. 5 * 10 = 50 motor speed)

ACCEL = 0.20  # Speed gained per tick
DECEL = 0.12  # Speed lost per tick
MAX_TURN_STRENGTH = 9.0 

# Add a rate limiter
last_control_time = 0
CONTROL_INTERVAL = 0.033  # 30 Hz instead of 60 Hz

# -- PID Controller --
KP = 18.0         # Proportional: main steering response
KI = 0.5          # Integral: eliminates curve drift (keep small)
KD = 12.0         # Derivative: dampens oscillation (critical for tray)
PID_I_MAX = 50.0  # Anti-windup clamp

# -- Strafe Correction (mecanum advantage) --
STRAFE_WEIGHT_THRESHOLD = 3.0  # Below this |turn_var|, use mostly strafe
STRAFE_BLEND_RANGE = 2.0       # Transition range to pure rotation
STRAFE_GAIN = 8.0              # Strafe correction strength

# -- Omega Rate Limiter (tray stability) --
OMEGA_RATE_LIMIT = 5.0  # Max rotation change per tick

# -- Recovery & Junction Parameters --
RECOVERY_REVERSE_SPEED = 20
RECOVERY_PIVOT_SPEED   = 18
ENDPOINT_TARGET_SPEED  = 2.0  # Out of 5.0

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

current_speed = DEFAULT_SPEED # Acts as the master scalar for the 0-5 algorithmic speeds

# FSM States
STATE_FOLLOWING    = 0
STATE_ENDPOINT     = 1
STATE_LOST_REVERSE = 2
STATE_LOST_PIVOT   = 3
STATE_STOPPED      = 4
STATE_PARKING      = 5
STATE_LOST_STRAFE  = 6

auto_state     = STATE_FOLLOWING
internal_speed = 0.0 # Tracks the 0.0 to 5.0 algorithmic speed
last_turn_var  = 0.0 # Memory for which way we were turning before getting lost
turn_var = 0.0
lost_start_time = None

# PID state
pid_integral   = 0.0
pid_last_error = 0.0
pid_last_time  = 0.0
last_omega     = 0.0  # For omega rate limiting

# Delivery zone parking
all_on_start_time  = None
parking_start_time = None
DELIVERY_ZONE_TIME_THRESHOLD = 0.4  # Seconds of sustained all-5-ON = delivery zone
PARKING_DRIVE_TIME = 0.8            # Seconds to drive forward into zone
PARKING_SPEED = 15                  # Slow crawl into the zone

# Strafe recovery
strafe_search_start = None
STRAFE_SEARCH_DURATION = 0.4  # Seconds to try strafing before falling back
STRAFE_SEARCH_SPEED = 30      # Strafe speed during search
STRAFE_SEARCH_FWD = 10        # Slight forward during strafe search

# Debug values for GUI
debug_pid_p = 0.0
debug_pid_i = 0.0
debug_pid_d = 0.0
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
    STATE_LOST_STRAFE:  "LOST_STRAFE",
}

# ---------------------------------------------------------------------------
# UART & Movement Functions (Kept identical for hardware compatibility)
# ---------------------------------------------------------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def sendSerialCommand(command_name, params):
    param_str   = ",".join(map(str, params))
    command_str = f"{command_name}:{param_str}\n"
    ser.write(command_str.encode())

def moveForward(speed): sendSerialCommand('mv_fwd', [speed])
def moveReverse(speed): sendSerialCommand('mv_rev', [speed])
def moveLeft(speed): sendSerialCommand('mv_left', [speed])
def moveRight(speed): sendSerialCommand('mv_right', [speed])
def moveTurnLeft(speed): sendSerialCommand('mv_turnleft', [speed])
def moveTurnRight(speed): sendSerialCommand('mv_turnright', [speed])
def stopAll(): sendSerialCommand('stop', [0])
def moveCurve(left_speed, right_speed): sendSerialCommand('mv_curve', [left_speed, right_speed])
def moveVector(vx, vy, omega): sendSerialCommand('mv_vector', [int(vx), int(vy), int(omega)])

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
# PID, Strafe, Speed & Rate Limiting Functions
# ---------------------------------------------------------------------------

def reset_pid():
    global pid_integral, pid_last_error, pid_last_time, last_omega
    pid_integral = 0.0
    pid_last_error = 0.0
    pid_last_time = time.time()
    last_omega = 0.0

def pid_compute(error, dt):
    global pid_integral, pid_last_error
    global debug_pid_p, debug_pid_i, debug_pid_d
    if dt <= 0:
        return 0.0
    P = KP * error
    pid_integral += error * dt
    pid_integral = max(-PID_I_MAX, min(PID_I_MAX, pid_integral))
    I = KI * pid_integral
    D = KD * (error - pid_last_error) / dt
    pid_last_error = error
    debug_pid_p, debug_pid_i, debug_pid_d = P, I, D
    return P + I + D

def compute_strafe_correction(turn_var):
    abs_tv = abs(turn_var)
    if abs_tv < STRAFE_WEIGHT_THRESHOLD:
        strafe_factor = 1.0
    elif abs_tv < STRAFE_WEIGHT_THRESHOLD + STRAFE_BLEND_RANGE:
        strafe_factor = 1.0 - (abs_tv - STRAFE_WEIGHT_THRESHOLD) / STRAFE_BLEND_RANGE
    else:
        strafe_factor = 0.0
    return turn_var * STRAFE_GAIN * strafe_factor

def compute_target_speed(bits, turn_var):
    active = sum(bits)
    abs_turn = abs(turn_var)
    if active == 0:
        return 0.0
    # Only center sensor -> perfectly straight -> MAX
    if bits == [0, 0, 1, 0, 0]:
        return 5.0
    # Center + one neighbor -> gentle curve -> HIGH
    if active <= 2 and bits[IDX_N] == 1:
        return 4.5
    # Outer sensor only -> sharp correction -> SLOW (check before medium)
    if abs_turn >= 5.0:
        return 2.0
    # Diagonal sensors -> moderate curve -> MEDIUM
    if abs_turn >= 3.0:
        return 3.0
    # Default: weighted average
    return sum(b * m for b, m in zip(bits, MOVE_STRENGTHS)) / active

def rate_limit_omega(new_omega):
    global last_omega
    delta = new_omega - last_omega
    if abs(delta) > OMEGA_RATE_LIMIT:
        new_omega = last_omega + OMEGA_RATE_LIMIT * (1 if delta > 0 else -1)
    last_omega = new_omega
    return new_omega

# ---------------------------------------------------------------------------
# ALGORITHMIC LINE FOLLOWING FSM
# ---------------------------------------------------------------------------
def line_follow_step():
    global auto_state, internal_speed, last_turn_var, current_speed, turn_var
    global lost_start_time, all_on_start_time, parking_start_time, strafe_search_start
    global pid_last_time, auto_mode
    global last_control_time
    global debug_vx, debug_vy, debug_omega

    current_time = time.time()
    if current_time - last_control_time < CONTROL_INTERVAL:
        return
    last_control_time = current_time

    bits = get_ir_bits()
    active_count = sum(bits)

    # Speed multiplier (maps 0.0-5.0 scale to actual motor speed)
    multiplier = current_speed / 5.0

    # 1. STATE: STOPPED / BRAKING
    if auto_state == STATE_STOPPED:
        if internal_speed > 0:
            internal_speed = max(0.0, internal_speed - DECEL * 5)
            vx = int(round(internal_speed * multiplier))
            moveVector(vx, 0, 0)
        else:
            stopAll()
        return

    # 2. STATE: PARKING (delivery zone)
    if auto_state == STATE_PARKING:
        elapsed = current_time - parking_start_time
        if elapsed < PARKING_DRIVE_TIME:
            moveVector(PARKING_SPEED, 0, 0)
        else:
            stopAll()
            auto_mode = False
            print(">>> PARKED. ATTEMPT COMPLETE.")
        return

    # 3. STATE: LOST - STRAFE SEARCH (try strafing before reversing)
    if auto_state == STATE_LOST_STRAFE:
        if active_count > 0:
            auto_state = STATE_FOLLOWING
            reset_pid()
            strafe_search_start = None
            return
        elapsed = current_time - strafe_search_start
        if elapsed < STRAFE_SEARCH_DURATION:
            strafe_dir = 1 if last_turn_var > 0 else -1
            moveVector(STRAFE_SEARCH_FWD, strafe_dir * STRAFE_SEARCH_SPEED, 0)
        else:
            auto_state = STATE_LOST_REVERSE
            strafe_search_start = None
        return

    # 4. STATE: LOST - REVERSING TO FIND LINE
    if auto_state == STATE_LOST_REVERSE:
        if active_count > 0:
            auto_state = STATE_LOST_PIVOT
        else:
            moveVector(-RECOVERY_REVERSE_SPEED, 0, 0)
        return

    # 5. STATE: LOST - PIVOTING TO RECENTER
    if auto_state == STATE_LOST_PIVOT:
        if active_count > 0:
            curr_turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            if abs(curr_turn_var) <= 3.0:
                auto_state = STATE_FOLLOWING
                reset_pid()
                return
        pivot_dir = 1 if last_turn_var > 0 else -1
        moveVector(0, 0, pivot_dir * RECOVERY_PIVOT_SPEED)
        return

    # 6. STATE: ENDPOINT / JUNCTION HANDLING
    if auto_state == STATE_ENDPOINT:
        if active_count == 5:
            # Still all-on — check if this is a delivery zone (sustained)
            if all_on_start_time is None:
                all_on_start_time = current_time
            elapsed_all_on = current_time - all_on_start_time
            if elapsed_all_on >= DELIVERY_ZONE_TIME_THRESHOLD:
                auto_state = STATE_PARKING
                parking_start_time = current_time
                all_on_start_time = None
                print(">>> DELIVERY ZONE DETECTED — PARKING")
                return
            # Not yet confirmed as delivery zone — drive straight slowly
            target_max_speed = ENDPOINT_TARGET_SPEED
            if internal_speed < target_max_speed:
                internal_speed = min(target_max_speed, internal_speed + ACCEL)
            elif internal_speed > target_max_speed:
                internal_speed = max(target_max_speed, internal_speed - DECEL)
            moveVector(int(internal_speed * multiplier), 0, 0)
            return

        # Exited all-on zone
        all_on_start_time = None
        if 0 < active_count < 5:
            auto_state = STATE_FOLLOWING
            reset_pid()
            print(">>> ENDPOINT CLEARED")
            return
        if active_count == 0:
            auto_state = STATE_LOST_STRAFE
            strafe_search_start = current_time
            return

    # 7. STATE: NORMAL FOLLOWING (PID + strafe + vector drive)
    if auto_state == STATE_FOLLOWING:
        if active_count == 0:
            # Time-based lost detection
            if lost_start_time is None:
                lost_start_time = current_time
            elapsed_lost = current_time - lost_start_time
            if elapsed_lost >= LOST_DETECTION_DELAY:
                auto_state = STATE_LOST_STRAFE
                strafe_search_start = current_time
                internal_speed = 0.0
                lost_start_time = None
                print(f">>> LOST LINE: TRYING STRAFE RECOVERY")
            else:
                # Slow down and drift in last direction while waiting
                internal_speed = max(0.0, internal_speed - DECEL)
                drift_omega = (1 if last_turn_var > 0 else -1) * int(internal_speed * multiplier * 0.5)
                moveVector(int(internal_speed * multiplier), 0, drift_omega)
            return
        else:
            lost_start_time = None

        # All 5 sensors on -> entering endpoint/junction
        if active_count == 5:
            auto_state = STATE_ENDPOINT
            all_on_start_time = current_time
            print(">>> ALL SENSORS ON: ENTERING ENDPOINT")
            return

        # --- Compute steering ---
        turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
        last_turn_var = turn_var

        # Adaptive target speed
        target_max_speed = compute_target_speed(bits, turn_var)

        # Smooth acceleration / deceleration
        if internal_speed < target_max_speed:
            internal_speed = min(target_max_speed, internal_speed + ACCEL)
        elif internal_speed > target_max_speed:
            internal_speed = max(target_max_speed, internal_speed - DECEL)

        # PID for rotation (omega)
        dt = current_time - pid_last_time
        pid_last_time = current_time
        omega_raw = pid_compute(turn_var, dt)

        # Strafe correction (mecanum advantage: small errors -> strafe, large -> rotate)
        vy_raw = compute_strafe_correction(turn_var)

        # Convert to motor commands
        vx = int(internal_speed * multiplier)
        vy = max(-MAX_SPEED, min(MAX_SPEED, int(vy_raw * multiplier / 5.0)))
        omega = max(-MAX_SPEED, min(MAX_SPEED, int(omega_raw * multiplier / 5.0)))
        omega = int(rate_limit_omega(omega))

        debug_vx, debug_vy, debug_omega = vx, vy, omega
        moveVector(vx, vy, omega)


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
                except ValueError: pass

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
        if   pressed['w']: moveForward(current_speed)
        elif pressed['s']: moveReverse(current_speed)
        elif pressed['q']: moveTurnLeft(current_speed)
        elif pressed['e']: moveTurnRight(current_speed)
        elif pressed['a']: moveLeft(current_speed)
        elif pressed['d']: moveRight(current_speed)
        else:              stopAll()

    clock = pygame.time.Clock()

    while running:
        screen.fill((30, 30, 30))
        mode_color = (0, 220, 100) if auto_mode else (220, 180, 0)
        mode_label = "AUTO (PID+Vector)" if auto_mode else "MANUAL (WASD+QE)"

        bits = get_ir_bits()
        active = sum(bits)
        turn_display = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active if active > 0 else 0.0

        # State color coding
        state_colors = {
            STATE_FOLLOWING: (0, 220, 100),    # Green
            STATE_ENDPOINT: (220, 220, 0),     # Yellow
            STATE_LOST_REVERSE: (220, 50, 50), # Red
            STATE_LOST_PIVOT: (220, 100, 50),  # Orange
            STATE_LOST_STRAFE: (220, 150, 50), # Light orange
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

        # PID debug
        screen.blit(font_sm.render(f"PID  P:{debug_pid_p:+.1f}  I:{debug_pid_i:+.1f}  D:{debug_pid_d:+.1f}", True, (180,180,255)), (20, y)); y += 22

        # Vector output
        screen.blit(font_sm.render(f"Vector  vx:{debug_vx}  vy:{debug_vy}  omega:{debug_omega}", True, (180,255,180)), (20, y)); y += 22

        # Delivery zone timer
        if all_on_start_time is not None:
            zone_t = time.time() - all_on_start_time
            screen.blit(font_sm.render(f"All-ON timer: {zone_t:.2f}s / {DELIVERY_ZONE_TIME_THRESHOLD}s", True, (50,100,220)), (20, y))
        y += 22

        screen.blit(font_sm.render("Keys: M=auto  WASD+QE=manual  1/2=speed  Esc=quit", True, (120,120,120)), (20, y))

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
                        auto_state = STATE_FOLLOWING
                        internal_speed = 0.0
                        reset_pid()
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