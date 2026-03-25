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

# -- Distance Integration Thresholds --
# Instead of counting frames or time, we integrate speed*dt to estimate
# distance traveled while a condition holds. Units are arbitrary
# (internal_speed * seconds).
LOST_DISTANCE_THRESHOLD     = 1.5   # integrated distance with no line before recovery
ENDPOINT_DISTANCE_THRESHOLD = 0.8   # integrated distance with all sensors on before gate check

# -- Algorithm Weights (W, NW, N, NE, E) --
TURN_STRENGTHS = [-7, -4.5, 0.0, 4.5, 7]
MOVE_STRENGTHS = [3.8, 4.6, 5.0, 4.6, 3.8]

# -- Speed & Physics Limits --
MIN_SPEED       = 10       # lowest usable motor speed
MAX_SPEED       = 120     # highest motor speed
DEFAULT_SPEED   = 35      # Base multiplier setting

ACCEL = 0.20  # Speed gained per tick
DECEL = 0.12  # Speed lost per tick
MAX_TURN_STRENGTH = 9.0

# Add a rate limiter
last_control_time = 0
CONTROL_INTERVAL = 0.033  # 30 Hz instead of 60 Hz

# -- Recovery & Junction Parameters --
RECOVERY_REVERSE_SPEED = 20
RECOVERY_SIDEPIVOT_FRONT_SPEED = 25  # Front wheel speed for side pivot
RECOVERY_SIDEPIVOT_REAR_PCT    = 15  # Rear wheels at 15% of front speed
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

auto_state     = STATE_FOLLOWING
internal_speed = 0.0 # Tracks the 0.0 to 5.0 algorithmic speed
last_turn_var  = 0.0 # Memory for which way we were turning before getting lost
turn_var = 0.0

# Distance integration accumulators
lost_distance_accum     = 0.0  # accumulated distance while line is lost
endpoint_distance_accum = 0.0  # accumulated distance while on endpoint (all sensors)
prev_tick_time          = 0.0  # timestamp of previous tick for dt calculation

STATE_NAMES = {
    STATE_FOLLOWING:    "FOLLOW",
    STATE_ENDPOINT:     "ENDPOINT",
    STATE_LOST_REVERSE: "LOST_REV",
    STATE_LOST_PIVOT:   "LOST_PIVOT",
    STATE_STOPPED:      "STOPPED",
}

# ---------------------------------------------------------------------------
# UART & Movement Functions
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

def moveSidePivot(front_speed, rear_percent, direction):
    """Side pivot: front wheels oppose at high speed, rear crawl or stop.
    direction: 1 = pivot right, -1 = pivot left"""
    sendSerialCommand('mv_sidepivot', [front_speed, rear_percent, direction])

# ---------------------------------------------------------------------------
# IR Sensor Helpers
# ---------------------------------------------------------------------------
IDX_W, IDX_NW, IDX_N, IDX_NE, IDX_E = 0, 1, 2, 3, 4

def get_ir_bits():
    with ir_lock:
        v = ir_status
    if REVERSE_SENSOR_ORDER:
        return [(v >> (4 - i)) & 1 for i in range(5)]
    return [(v >> i) & 1 for i in range(5)]

# ---------------------------------------------------------------------------
# ALGORITHMIC LINE FOLLOWING FSM
# ---------------------------------------------------------------------------
def line_follow_step():
    global auto_state, internal_speed, last_turn_var, current_speed, turn_var
    global lost_distance_accum, endpoint_distance_accum, prev_tick_time
    global last_control_time

    current_time = time.time()
    if current_time - last_control_time < CONTROL_INTERVAL:
        return
    last_control_time = current_time

    # Calculate dt for distance integration
    if prev_tick_time == 0.0:
        dt = CONTROL_INTERVAL  # first tick, assume one interval
    else:
        dt = current_time - prev_tick_time
    prev_tick_time = current_time

    bits = get_ir_bits()
    active_count = sum(bits)
    pattern = (bits[IDX_E] << 4) | (bits[IDX_NE] << 3) | (bits[IDX_N] << 2) | (bits[IDX_NW] << 1) | bits[IDX_W]

    # Speed multiplier (maps 0.0-5.0 scale to actual motor speed)
    multiplier = current_speed / 5.0

    # 1. STATE: STOPPED / BRAKING
    if auto_state == STATE_STOPPED:
        if internal_speed > 0:
            internal_speed = max(0.0, internal_speed - DECEL*5)
            left_motor  = int(round(internal_speed * multiplier))
            right_motor = int(round(internal_speed * multiplier))
            moveCurve(left_motor, right_motor)
        else:
            stopAll()
        return

    # 2. STATE: LOST - REVERSING TO FIND LINE
    if auto_state == STATE_LOST_REVERSE:
        if active_count > 0:
            auto_state = STATE_LOST_PIVOT
            lost_distance_accum = 0.0
        else:
            moveCurve(-RECOVERY_REVERSE_SPEED, -RECOVERY_REVERSE_SPEED)
        return

    # 3. STATE: LOST - SIDE PIVOT TO RECENTER
    if auto_state == STATE_LOST_PIVOT:
        if active_count > 0:
            curr_turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            if abs(curr_turn_var) <= 3.0:
                auto_state = STATE_FOLLOWING
                lost_distance_accum = 0.0
                return

        # Side pivot: front wheels fast in opposing directions, rear wheels slow
        direction = 1 if last_turn_var > 0 else -1
        moveSidePivot(RECOVERY_SIDEPIVOT_FRONT_SPEED, RECOVERY_SIDEPIVOT_REAR_PCT, direction)
        return

    # 4. STATE: ENDPOINT HANDLING (distance-integrated)
    if auto_state == STATE_ENDPOINT:
        target_max_speed = ENDPOINT_TARGET_SPEED
        turn_var = 0.0

        # Integrate distance while in endpoint state
        endpoint_distance_accum += internal_speed * dt

        # Check for End/Gate patterns only after traveling enough distance
        if endpoint_distance_accum >= ENDPOINT_DISTANCE_THRESHOLD and (pattern == 0b11011 or pattern == 0b10001):
            auto_state = STATE_STOPPED
            endpoint_distance_accum = 0.0
            print(f">>> GATE DETECTED: HARD STOP (dist={endpoint_distance_accum:.2f})")
            return

        # If we crossed a junction and are back to a normal track
        if 0 < active_count < 5 and pattern not in [0b11011, 0b10001]:
            auto_state = STATE_FOLLOWING
            endpoint_distance_accum = 0.0
            print(">>> ENDPOINT CLEARED")
            return

        # If we completely lose the line in a junction
        if active_count == 0:
            auto_state = STATE_LOST_REVERSE
            endpoint_distance_accum = 0.0
            return

    # 5. STATE: NORMAL FOLLOWING (distance-integrated lost detection)
    if auto_state == STATE_FOLLOWING:
        if active_count == 0:
            # Integrate distance traveled while lost
            # Use max(internal_speed, 1.0) so even stopped robots eventually trigger recovery
            effective_speed = max(internal_speed, 1.0)
            lost_distance_accum += effective_speed * dt

            if lost_distance_accum >= LOST_DISTANCE_THRESHOLD:
                auto_state = STATE_LOST_REVERSE
                internal_speed = 0.0
                print(f">>> LOST LINE: ENTERING RECOVERY after dist={lost_distance_accum:.2f}")
                lost_distance_accum = 0.0
            else:
                # Still accumulating — slow down and drift in last known direction
                internal_speed = max(0.0, internal_speed - DECEL)
                if last_turn_var > 0:
                    moveCurve(int(internal_speed * multiplier), -int(internal_speed * multiplier))
                else:
                    moveCurve(-int(internal_speed * multiplier), int(internal_speed * multiplier))
                return
        else:
            # Reset distance accumulator when we see line again
            lost_distance_accum = 0.0

        # Continue with normal following logic if we have line
        if active_count == 5:
            auto_state = STATE_ENDPOINT
            endpoint_distance_accum = 0.0
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

    # --- Shared Physics & Steering ---

    if internal_speed < target_max_speed:
        internal_speed = min(target_max_speed, internal_speed + ACCEL)
    elif internal_speed > target_max_speed:
        internal_speed = max(target_max_speed, internal_speed - DECEL)

    turn_ratio = abs(turn_var) / MAX_TURN_STRENGTH * 2
    delta_v = internal_speed * turn_ratio

    if turn_var > 0:
        left_algo  = (internal_speed + delta_v) * 1.3
        right_algo = internal_speed - delta_v
    elif turn_var < 0:
        left_algo  = internal_speed - delta_v
        right_algo = (internal_speed + delta_v) * 1.3
    else:
        left_algo = right_algo = internal_speed

    left_motor  = max(-MAX_SPEED, min(MAX_SPEED, int(round(left_algo * multiplier))))
    right_motor = max(-MAX_SPEED, min(MAX_SPEED, int(round(right_algo * multiplier))))

    moveCurve(left_motor, right_motor)


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
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Algorithmic Line Follower")
    font = pygame.font.SysFont(None, 28)

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
        mode_label = "AUTO (Algorithmic)" if auto_mode else "MANUAL (WASD+QE)"

        bits = get_ir_bits()
        active = sum(bits)
        turn_display = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active if active > 0 else 0.0

        screen.blit(font.render(f"MODE: {mode_label}", True, mode_color), (20, 20))
        screen.blit(font.render(f"Max Speed Scalar: {current_speed}", True, (200,200,200)), (20, 60))
        screen.blit(font.render(f"Internal Algo Spd: {internal_speed:.2f}", True, (200,200,200)), (20, 90))
        screen.blit(font.render(f"Turn Strength: {(turn_var / MAX_TURN_STRENGTH * 2):.2f}", True, (200,200,200)), (20, 120))
        screen.blit(font.render(f"W={bits[0]} NW={bits[1]} N={bits[2]} NE={bits[3]} E={bits[4]}", True, (100,180,255)), (20, 160))
        screen.blit(font.render(f"Lost dist: {lost_distance_accum:.2f}  EP dist: {endpoint_distance_accum:.2f}", True, (100,180,255)), (20, 190))
        screen.blit(font.render(f"State: {STATE_NAMES.get(auto_state, '?')}", True, (180,220,180)), (20, 230))

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
                        lost_distance_accum = 0.0
                        endpoint_distance_accum = 0.0
                        prev_tick_time = 0.0
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
