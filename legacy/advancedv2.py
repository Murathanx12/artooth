import time
import serial
import threading
import signal
import sys
import math
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

# -- Curve Handling --
# On sharp curves: slow way down + strong rotation (like pressing Q/E + gentle W)
OMEGA_GAIN       = 4.5     # Rotation strength for heading correction
CURVE_SLOW_FACTOR = 0.20   # At max turn, forward speed drops to 20% (slow crawl through curves)
CURVE_SLOW_EXPO   = 1.5    # Exponent for speed reduction curve (>1 = stays fast longer on gentle curves)

# -- Recovery & Junction Parameters --
RECOVERY_REVERSE_SPEED = 20
RECOVERY_PIVOT_SPEED   = 80
RECOVERY_PIVOT_REAR_PERCENT = 15
ENDPOINT_TARGET_SPEED  = 2.0   # Out of 5.0

# -- Pseudo-distance thresholds (speed_units x seconds, no encoder) --
REVERSE_PSEUDO_DIST_MAX  = 0.18
ENDPOINT_PSEUDO_DIST_MAX = 0.4
LOST_PSEUDO_DIST_MAX     = 0.15

# -- Delivery zone parking --
DELIVERY_ZONE_TIME_THRESHOLD = 0.4
PARKING_DRIVE_TIME = 0.8
PARKING_SPEED = 15

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

current_speed = DEFAULT_SPEED

# FSM States
STATE_FOLLOWING    = 0
STATE_ENDPOINT     = 1
STATE_LOST_REVERSE = 2
STATE_LOST_PIVOT   = 3
STATE_STOPPED      = 4
STATE_PARKING      = 5

auto_state     = STATE_FOLLOWING
internal_speed = 0.0
last_turn_var  = 0.0
turn_var       = 0.0

# -- Pseudo-distance accumulator --
pseudo_dist      = 0.0
last_step_time   = 0.0

# -- Delivery zone parking --
all_on_start_time  = None
parking_start_time = None

# Debug values for GUI (updated in ALL states so viz always works)
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

    now = time.monotonic()
    dt  = now - last_step_time if last_step_time > 0 else 0.0
    last_step_time = now

    bits         = get_ir_bits()
    active_count = sum(bits)
    pattern      = (bits[IDX_E] << 4) | (bits[IDX_NE] << 3) | (bits[IDX_N] << 2) | (bits[IDX_NW] << 1) | bits[IDX_W]

    multiplier = current_speed / 5.0

    # -----------------------------------------------------------------------
    # 1. STATE: STOPPED / BRAKING
    # -----------------------------------------------------------------------
    if auto_state == STATE_STOPPED:
        if internal_speed > 0:
            internal_speed = max(0.0, internal_speed - DECEL * 5)
            vx = int(round(internal_speed * multiplier))
            debug_vx, debug_vy, debug_omega = vx, 0, 0
            moveVector(vx, 0, 0)
        else:
            debug_vx, debug_vy, debug_omega = 0, 0, 0
            stopAll()
        return

    # -----------------------------------------------------------------------
    # 2. STATE: PARKING (delivery zone)
    # -----------------------------------------------------------------------
    if auto_state == STATE_PARKING:
        elapsed = time.monotonic() - parking_start_time
        if elapsed < PARKING_DRIVE_TIME:
            debug_vx, debug_vy, debug_omega = PARKING_SPEED, 0, 0
            moveVector(PARKING_SPEED, 0, 0)
        else:
            debug_vx, debug_vy, debug_omega = 0, 0, 0
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
            print(f">>> LOST_REV -> PIVOT (dist={pseudo_dist:.2f})")
            _enter_state(STATE_LOST_PIVOT)
        elif active_count > 0:
            print(f">>> LOST_REV -> PIVOT (line reappeared, dist={pseudo_dist:.2f})")
            _enter_state(STATE_LOST_PIVOT)
        else:
            debug_vx, debug_vy, debug_omega = -RECOVERY_REVERSE_SPEED, 0, 0
            moveVector(-RECOVERY_REVERSE_SPEED, 0, 0)
        return

    # -----------------------------------------------------------------------
    # 4. STATE: LOST — PIVOTING TO RECENTER
    # -----------------------------------------------------------------------
    if auto_state == STATE_LOST_PIVOT:
        if active_count > 0:
            curr_turn_var = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active_count
            if abs(curr_turn_var) < 5.0:
                print(f">>> LOST_PIVOT -> FOLLOW (turn_var={curr_turn_var:.2f})")
                _enter_state(STATE_FOLLOWING)
                return

        direction = 1 if last_turn_var > 0 else -1
        debug_vx, debug_vy, debug_omega = 0, 0, direction * RECOVERY_PIVOT_SPEED
        moveSidePivot(RECOVERY_PIVOT_SPEED, RECOVERY_PIVOT_REAR_PERCENT, direction)
        return

    # -----------------------------------------------------------------------
    # 5. STATE: ENDPOINT HANDLING
    # -----------------------------------------------------------------------
    if auto_state == STATE_ENDPOINT:
        speed_frac = internal_speed / 5.0
        _accumulate_pseudo_dist(dt, speed_frac)

        if active_count == 5:
            if all_on_start_time is None:
                all_on_start_time = time.monotonic()
            elapsed_all_on = time.monotonic() - all_on_start_time
            if elapsed_all_on >= DELIVERY_ZONE_TIME_THRESHOLD:
                auto_state = STATE_PARKING
                parking_start_time = time.monotonic()
                all_on_start_time = None
                print(">>> DELIVERY ZONE DETECTED — PARKING")
                return

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

        target_max_speed = ENDPOINT_TARGET_SPEED
        turn_var = 0.0

    # -----------------------------------------------------------------------
    # 6. STATE: NORMAL FOLLOWING
    # -----------------------------------------------------------------------
    if auto_state == STATE_FOLLOWING:
        if active_count == 0:
            speed_frac = internal_speed / 5.0
            _accumulate_pseudo_dist(dt, speed_frac)
            if pseudo_dist >= LOST_PSEUDO_DIST_MAX:
                _enter_state(STATE_LOST_REVERSE)
                internal_speed = 0.0
                debug_vx, debug_vy, debug_omega = 0, 0, 0
                print(f">>> LOST LINE -> RECOVERY after dist {pseudo_dist:.2f}")
            else:
                # Coast with last known rotation
                internal_speed = max(0.0, internal_speed - DECEL)
                coast_vx = int(internal_speed * multiplier)
                coast_omega = (1 if last_turn_var > 0 else -1) * int(internal_speed * multiplier * 0.5)
                debug_vx, debug_vy, debug_omega = coast_vx, 0, coast_omega
                moveVector(coast_vx, 0, coast_omega)
            return
        else:
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
    # Shared: Smooth rotation + speed control  (Following & Endpoint)
    #
    # Philosophy: on straights go fast, on curves SLOW DOWN and ROTATE hard
    #             like pressing W gently + Q/E hard. No strafing (vy=0).
    # -----------------------------------------------------------------------

    # Smooth acceleration / deceleration
    if internal_speed < target_max_speed:
        internal_speed = min(target_max_speed, internal_speed + ACCEL)
    elif internal_speed > target_max_speed:
        internal_speed = max(target_max_speed, internal_speed - DECEL)

    # How sharp is the curve? 0.0 = straight, 1.0 = hardest turn
    turn_ratio = min(1.0, abs(turn_var) / MAX_TURN_STRENGTH)

    # OMEGA: strong rotation into the curve (like pressing Q or E)
    omega_raw = turn_var * OMEGA_GAIN * multiplier / 5.0

    # VX: slow way down on curves — stays fast on gentle curves, crawls on sharp ones
    #   turn_ratio^1.5 means: at 30% turn -> 84% speed, at 70% turn -> 41% speed, at 100% -> 20%
    speed_scale = 1.0 - (1.0 - CURVE_SLOW_FACTOR) * (turn_ratio ** CURVE_SLOW_EXPO)
    vx_raw = internal_speed * multiplier * speed_scale

    vx = max(-MAX_SPEED, min(MAX_SPEED, int(round(vx_raw))))
    omega = max(-MAX_SPEED, min(MAX_SPEED, int(round(omega_raw))))

    debug_vx, debug_vy, debug_omega = vx, 0, omega
    moveVector(vx, 0, omega)


# ---------------------------------------------------------------------------
# UART Thread & Pygame
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
    W, H = 700, 580
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption("Alfred Line Follower v3")
    font_lg = pygame.font.SysFont(None, 32)
    font    = pygame.font.SysFont(None, 26)
    font_sm = pygame.font.SysFont(None, 22)
    font_xs = pygame.font.SysFont(None, 18)

    pressed = {'w': False, 's': False, 'a': False, 'd': False, 'q': False, 'e': False}

    def update_movement():
        global debug_vx, debug_vy, debug_omega
        if auto_mode: return
        w, s = pressed['w'], pressed['s']
        a, d = pressed['a'], pressed['d']
        q, e = pressed['q'], pressed['e']

        vx = 0; vy = 0; omega = 0
        if w: vx += current_speed
        if s: vx -= current_speed
        if a: vy -= current_speed
        if d: vy += current_speed
        if q: omega -= current_speed
        if e: omega += current_speed

        debug_vx, debug_vy, debug_omega = vx, vy, omega
        if vx == 0 and vy == 0 and omega == 0:
            stopAll()
        else:
            moveVector(vx, vy, omega)

    def draw_arrow(surface, color, start, end, width=2, head_size=8):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.sqrt(dx * dx + dy * dy)
        if length < 3:
            return
        pygame.draw.line(surface, color, start, end, width)
        angle = math.atan2(dy, dx)
        left  = (end[0] - head_size * math.cos(angle - 0.5),
                 end[1] - head_size * math.sin(angle - 0.5))
        right = (end[0] - head_size * math.cos(angle + 0.5),
                 end[1] - head_size * math.sin(angle + 0.5))
        pygame.draw.polygon(surface, color, [end, left, right])

    clock = pygame.time.Clock()
    anim_vx = 0.0
    anim_vy = 0.0
    anim_omega = 0.0

    while running:
        screen.fill((16, 18, 24))

        bits = get_ir_bits()
        active = sum(bits)
        turn_display = sum(b * t for b, t in zip(bits, TURN_STRENGTHS)) / active if active > 0 else 0.0

        mode_color = (0, 220, 100) if auto_mode else (220, 180, 0)
        mode_label = "AUTO" if auto_mode else "MANUAL"

        state_colors = {
            STATE_FOLLOWING:    (0, 220, 100),
            STATE_ENDPOINT:     (220, 220, 0),
            STATE_LOST_REVERSE: (220, 50, 50),
            STATE_LOST_PIVOT:   (220, 100, 50),
            STATE_STOPPED:      (120, 120, 130),
            STATE_PARKING:      (50, 100, 220),
        }
        st_color = state_colors.get(auto_state, (180, 220, 180))

        # Smooth animation lerp
        lerp = 0.25
        anim_vx    += (debug_vx    - anim_vx)    * lerp
        anim_vy    += (debug_vy    - anim_vy)    * lerp
        anim_omega += (debug_omega - anim_omega) * lerp

        # Layout constants
        LEFT_W = 360
        RIGHT_X = 375

        # =====================================================================
        # HEADER BAR
        # =====================================================================
        pygame.draw.rect(screen, (32, 36, 46), (0, 0, W, 48))
        pygame.draw.line(screen, (50, 55, 70), (0, 48), (W, 48))
        screen.blit(font_lg.render("ALFRED", True, (70, 160, 255)), (18, 12))

        # Mode pill
        pill_text = font.render(mode_label, True, mode_color)
        pill_w = pill_text.get_width() + 24
        pill_rect = pygame.Rect(140, 10, pill_w, 28)
        pygame.draw.rect(screen, mode_color, pill_rect, 1, border_radius=14)
        screen.blit(pill_text, (152, 13))

        # State pill
        state_name = STATE_NAMES.get(auto_state, '?')
        st_text = font.render(state_name, True, st_color)
        st_w = st_text.get_width() + 24
        st_rect = pygame.Rect(W - st_w - 16, 10, st_w, 28)
        pygame.draw.rect(screen, st_color, st_rect, 1, border_radius=14)
        screen.blit(st_text, (W - st_w - 4, 13))

        y = 58

        # =====================================================================
        # LEFT COLUMN — Sensors, Turn bar, Stats
        # =====================================================================

        # -- Sensor arc --
        panel_h = 100
        pygame.draw.rect(screen, (26, 30, 38), (10, y, LEFT_W, panel_h), border_radius=10)
        pygame.draw.rect(screen, (40, 45, 58), (10, y, LEFT_W, panel_h), 1, border_radius=10)
        screen.blit(font_xs.render("IR SENSORS", True, (80, 85, 100)), (20, y + 6))

        sensor_names = ['W', 'NW', 'N', 'NE', 'E']
        arc_positions = [(55, 62), (115, 34), (180, 22), (245, 34), (305, 62)]
        for i, (sx, sy) in enumerate(arc_positions):
            on = bits[i]
            cx, cy = sx, y + sy
            if on:
                glow = pygame.Surface((48, 48), pygame.SRCALPHA)
                pygame.draw.circle(glow, (0, 200, 60, 40), (24, 24), 22)
                screen.blit(glow, (cx - 24, cy - 18))
            box_color    = (0, 200, 60) if on else (42, 46, 56)
            border_color = (0, 255, 80) if on else (55, 60, 72)
            pygame.draw.rect(screen, box_color, (cx - 24, cy - 13, 48, 28), border_radius=6)
            pygame.draw.rect(screen, border_color, (cx - 24, cy - 13, 48, 28), 1, border_radius=6)
            lbl = font_sm.render(sensor_names[i], True, (255, 255, 255) if on else (90, 90, 100))
            screen.blit(lbl, (cx - lbl.get_width() // 2, cy - 7))
        y += panel_h + 8

        # -- Turn indicator bar --
        bar_h = 38
        pygame.draw.rect(screen, (26, 30, 38), (10, y, LEFT_W, bar_h), border_radius=8)
        pygame.draw.rect(screen, (40, 45, 58), (10, y, LEFT_W, bar_h), 1, border_radius=8)
        bar_cx = 10 + LEFT_W // 2
        bar_w = 240
        pygame.draw.rect(screen, (42, 46, 56), (bar_cx - bar_w // 2, y + 15, bar_w, 8), border_radius=4)
        pygame.draw.rect(screen, (80, 85, 100), (bar_cx - 1, y + 10, 2, 18))
        screen.blit(font_xs.render("L", True, (80, 85, 100)), (bar_cx - bar_w // 2 - 14, y + 12))
        screen.blit(font_xs.render("R", True, (80, 85, 100)), (bar_cx + bar_w // 2 + 5, y + 12))

        norm_turn = max(-1.0, min(1.0, turn_display / 7.0))
        dot_x = bar_cx + int(norm_turn * bar_w // 2)
        dot_color = (255, 70, 70) if abs(norm_turn) > 0.6 else (255, 190, 50) if abs(norm_turn) > 0.3 else (60, 210, 110)
        pygame.draw.circle(screen, dot_color, (dot_x, y + 19), 8)
        screen.blit(font_xs.render(f"{turn_display:+.1f}", True, (140, 150, 170)), (18, y + 10))
        y += bar_h + 8

        # -- Stats cards --
        card_h = 60
        cards = [
            ("SPEED",  f"{current_speed}",       (100, 180, 255)),
            ("ALGO",   f"{internal_speed:.1f}",   (180, 220, 130)),
            ("DIST",   f"{pseudo_dist:.2f}",      (220, 180, 80)),
        ]
        card_w = (LEFT_W - 16) // 3
        for ci, (label, value, color) in enumerate(cards):
            cx = 10 + ci * (card_w + 4)
            pygame.draw.rect(screen, (26, 30, 38), (cx, y, card_w, card_h), border_radius=8)
            pygame.draw.rect(screen, (40, 45, 58), (cx, y, card_w, card_h), 1, border_radius=8)
            screen.blit(font_xs.render(label, True, (80, 85, 100)), (cx + 10, y + 8))
            screen.blit(font.render(value, True, color), (cx + 10, y + 28))
        y += card_h + 8

        # -- Vector numeric readout --
        vec_h = 42
        pygame.draw.rect(screen, (26, 30, 38), (10, y, LEFT_W, vec_h), border_radius=8)
        pygame.draw.rect(screen, (40, 45, 58), (10, y, LEFT_W, vec_h), 1, border_radius=8)
        screen.blit(font_xs.render("OUTPUT", True, (80, 85, 100)), (20, y + 4))
        vxc = (100, 220, 130) if debug_vx > 0 else (220, 100, 100) if debug_vx < 0 else (110, 110, 120)
        vyc = (100, 180, 255) if debug_vy != 0 else (110, 110, 120)
        omc = (220, 170, 60)  if debug_omega != 0 else (110, 110, 120)
        screen.blit(font_sm.render(f"vx:{debug_vx:+d}", True, vxc), (20, y + 20))
        screen.blit(font_sm.render(f"vy:{debug_vy:+d}", True, vyc), (130, y + 20))
        screen.blit(font_sm.render(f"\u03c9:{debug_omega:+d}", True, omc), (240, y + 20))
        y += vec_h + 8

        # -- Delivery zone progress (conditional) --
        if all_on_start_time is not None:
            zone_t = time.monotonic() - all_on_start_time
            pct = min(1.0, zone_t / DELIVERY_ZONE_TIME_THRESHOLD)
            dz_h = 30
            pygame.draw.rect(screen, (26, 30, 38), (10, y, LEFT_W, dz_h), border_radius=6)
            pygame.draw.rect(screen, (40, 45, 58), (10, y, LEFT_W, dz_h), 1, border_radius=6)
            bar_full = LEFT_W - 20
            pygame.draw.rect(screen, (42, 46, 56), (20, y + 18, bar_full, 6), border_radius=3)
            pygame.draw.rect(screen, (50, 100, 220), (20, y + 18, int(bar_full * pct), 6), border_radius=3)
            screen.blit(font_xs.render(f"DELIVERY ZONE {zone_t:.1f}s", True, (50, 100, 220)), (20, y + 3))
            y += dz_h + 8

        # =====================================================================
        # RIGHT COLUMN — Vector Visualization + Throttle
        # =====================================================================
        viz_w = W - RIGHT_X - 12
        viz_h = 280
        viz_cx = RIGHT_X + viz_w // 2
        viz_cy = 58 + viz_h // 2
        viz_r = min(viz_w, viz_h) // 2 - 16

        # Panel
        pygame.draw.rect(screen, (26, 30, 38), (RIGHT_X, 58, viz_w, viz_h), border_radius=10)
        pygame.draw.rect(screen, (40, 45, 58), (RIGHT_X, 58, viz_w, viz_h), 1, border_radius=10)
        screen.blit(font_xs.render("VECTOR FIELD", True, (80, 85, 100)), (RIGHT_X + 10, 62))

        # Grid circles
        for r_frac in [0.33, 0.66, 1.0]:
            r = int(viz_r * r_frac)
            pygame.draw.circle(screen, (34, 38, 48), (viz_cx, viz_cy), r, 1)
        # Crosshair
        pygame.draw.line(screen, (34, 38, 48), (viz_cx - viz_r, viz_cy), (viz_cx + viz_r, viz_cy))
        pygame.draw.line(screen, (34, 38, 48), (viz_cx, viz_cy - viz_r), (viz_cx, viz_cy + viz_r))

        # Labels
        screen.blit(font_xs.render("FWD", True, (55, 60, 75)), (viz_cx - 10, viz_cy - viz_r - 14))
        screen.blit(font_xs.render("REV", True, (55, 60, 75)), (viz_cx - 10, viz_cy + viz_r + 3))
        screen.blit(font_xs.render("L",   True, (55, 60, 75)), (viz_cx - viz_r - 10, viz_cy - 6))
        screen.blit(font_xs.render("R",   True, (55, 60, 75)), (viz_cx + viz_r + 4,  viz_cy - 6))

        # Robot body
        bot = 16
        pygame.draw.rect(screen, (55, 60, 75), (viz_cx - bot, viz_cy - bot, bot * 2, bot * 2), border_radius=4)
        pygame.draw.rect(screen, (75, 82, 100), (viz_cx - bot, viz_cy - bot, bot * 2, bot * 2), 1, border_radius=4)
        pygame.draw.rect(screen, (100, 180, 255), (viz_cx - 4, viz_cy - bot - 4, 8, 5), border_radius=2)

        # Vector arrow  (vx=forward=screen-up, vy=right=screen-right)
        arrow_scale = viz_r / MAX_SPEED
        ax = anim_vy * arrow_scale
        ay = -anim_vx * arrow_scale
        arrow_len = math.sqrt(ax * ax + ay * ay)
        if arrow_len > 3:
            if anim_vx > 0:
                arrow_color = (80, 220, 130)
            elif anim_vx < -3:
                arrow_color = (220, 80, 80)
            else:
                arrow_color = (80, 160, 255)
            draw_arrow(screen, arrow_color, (viz_cx, viz_cy),
                       (viz_cx + int(ax), viz_cy + int(ay)), width=3, head_size=11)

        # Rotation arc
        if abs(anim_omega) > 3:
            arc_r = bot + 10
            arc_color = (220, 170, 60)
            arc_start = -0.5 if anim_omega > 0 else 2.6
            arc_span = min(abs(anim_omega) / MAX_SPEED * 3.14, 2.8)
            steps = max(8, int(arc_span * 12))
            points = []
            for s in range(steps + 1):
                a = arc_start + (arc_span * s / steps) * (1 if anim_omega > 0 else -1)
                points.append((viz_cx + int(arc_r * math.cos(a)),
                               viz_cy + int(arc_r * math.sin(a))))
            if len(points) > 1:
                pygame.draw.lines(screen, arc_color, False, points, 2)
                if len(points) >= 2:
                    draw_arrow(screen, arc_color, points[-2], points[-1], width=2, head_size=7)
            rot_label = "CW" if anim_omega > 0 else "CCW"
            screen.blit(font_xs.render(rot_label, True, arc_color), (viz_cx + arc_r + 6, viz_cy - 22))

        # -- Throttle gauge --
        gauge_y = 58 + viz_h + 10
        gauge_h = 56
        pygame.draw.rect(screen, (26, 30, 38), (RIGHT_X, gauge_y, viz_w, gauge_h), border_radius=8)
        pygame.draw.rect(screen, (40, 45, 58), (RIGHT_X, gauge_y, viz_w, gauge_h), 1, border_radius=8)
        screen.blit(font_xs.render("THROTTLE", True, (80, 85, 100)), (RIGHT_X + 10, gauge_y + 6))

        gauge_bar_w = viz_w - 24
        gauge_fill = max(0.0, min(1.0, internal_speed / 5.0))
        pygame.draw.rect(screen, (42, 46, 56), (RIGHT_X + 12, gauge_y + 26, gauge_bar_w, 12), border_radius=6)
        fill_color = (220, 70, 70) if gauge_fill > 0.8 else (220, 180, 50) if gauge_fill > 0.5 else (60, 200, 120)
        if gauge_fill > 0.01:
            pygame.draw.rect(screen, fill_color, (RIGHT_X + 12, gauge_y + 26, int(gauge_bar_w * gauge_fill), 12), border_radius=6)
        pct_label = f"{gauge_fill * 100:.0f}%"
        screen.blit(font_sm.render(pct_label, True, fill_color), (RIGHT_X + gauge_bar_w - 20, gauge_y + 42))

        # =====================================================================
        # KEY HELP (bottom)
        # =====================================================================
        help_y = H - 52
        pygame.draw.rect(screen, (26, 30, 38), (10, help_y, W - 20, 42), border_radius=8)
        help_color = (70, 75, 90)
        screen.blit(font_xs.render("W/S fwd/rev    A/D strafe    Q/E rotate    M auto    1/2 speed    SPACE stop    ESC quit",
                                   True, help_color), (22, help_y + 6))
        screen.blit(font_xs.render("All keys combinable for omnidirectional movement",
                                   True, (55, 60, 75)), (22, help_y + 24))

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_m:
                    auto_mode = not auto_mode
                    pressed   = {k: False for k in pressed}
                    stopAll()
                    debug_vx, debug_vy, debug_omega = 0, 0, 0
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
                elif event.key == pygame.K_SPACE:
                    pressed = {k: False for k in pressed}
                    debug_vx, debug_vy, debug_omega = 0, 0, 0
                    stopAll()
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
