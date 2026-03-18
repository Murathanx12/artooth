import time
import serial
import threading
import signal
import sys
import pygame

# ===========================================================================
# TUNABLE PARAMETERS — adjust these to change robot behaviour
# ===========================================================================

# -- Sensor --
REVERSE_SENSOR_ORDER = True   # True = flip bit order (bit0↔bit4, bit1↔bit3)

# -- Speed limits (1-100 scale sent to ESP32) --
MIN_SPEED       = 5       # lowest usable motor speed
MAX_SPEED       = 100     # highest motor speed
DEFAULT_SPEED   = 50      # starting cruise speed (keys 1/2 adjust at runtime)

# -- Steering: [fast_side, slow_side] for each correction level --
#    Positive = forward, 0 = stop, negative = reverse
#    These are actual speed values (not ratios). The fast side gets this speed,
#    the slow side gets the other. Left/right is mirrored automatically.
STRAIGHT_SPEED    = 45    # both sides equal on a straight line
GENTLE_FAST       = 35    # fast side speed during gentle correction
GENTLE_SLOW       = 12    # slow side speed during gentle correction
HARD_FAST         = 28    # fast side speed during hard correction
HARD_SLOW         = -5    # slow side (negative = reverse for tighter turn)
CURVE_FAST        = 25    # fast side speed during sharp 3-sensor curve
CURVE_SLOW        = -12   # slow side reversed hard for sharp curves
UNKNOWN_FAST      = 15    # both sides for unknown patterns
UNKNOWN_SLOW      = 15

# -- Adaptive speed (how fast the robot ramps up on straights) --
RAMP_UP_STEP      = 1     # speed increases by this each tick on straights

# -- Recovery behaviour --
LOST_LINE_TIMEOUT   = 0.5   # seconds of no reading before recovery starts
RECOVERY_FWD_TIME   = 0.1   # seconds to creep forward during recovery
RECOVERY_BACK_TIME  = 0.5   # seconds to reverse if still lost
RECOVERY_SPEED      = 15    # speed during recovery creep/reverse

# -- End condition (all 5 sensors = finish line) --
END_COAST_TIME  = 0.5     # seconds to coast forward after all sensors trigger
END_COAST_SPEED = 20      # speed during end coast

# -- UART --
SERIAL_PORT     = '/dev/ttyAMA2'
BAUD_RATE       = 115200
PING_INTERVAL   = 5       # seconds between heartbeat pings

# ===========================================================================
# Globals (not tunable)
# ===========================================================================
running   = True
auto_mode = False

ir_status = 0
ir_lock   = threading.Lock()

current_speed = DEFAULT_SPEED

# Auto state machine
STATE_FOLLOWING     = 0
STATE_ENDING        = 1
STATE_RECOVERY_FWD  = 2
STATE_RECOVERY_BACK = 3
STATE_STOPPED       = 4

auto_state     = STATE_FOLLOWING
state_start    = 0.0
adaptive_speed = DEFAULT_SPEED
lost_since     = 0.0          # when we first saw no sensors (0 = not lost)

# ---------------------------------------------------------------------------
# UART
# ---------------------------------------------------------------------------
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

def sendSerialCommand(command_name, params):
    param_str   = ",".join(map(str, params))
    command_str = f"{command_name}:{param_str}\n"
    ser.write(command_str.encode())
    print(f"Sent: {command_str.strip()}")

# ---------------------------------------------------------------------------
# Movement functions
# ---------------------------------------------------------------------------
def moveForward(speed):
    sendSerialCommand('mv_fwd', [speed])

def moveReverse(speed):
    sendSerialCommand('mv_rev', [speed])

def moveLeft(speed):
    # STRAFE left (mecanum lateral) — manual A key only, NOT used in auto
    sendSerialCommand('mv_left', [speed])

def moveRight(speed):
    # STRAFE right (mecanum lateral) — manual D key only, NOT used in auto
    sendSerialCommand('mv_right', [speed])

def moveTurnLeft(speed):
    # TANK ROTATE CCW — both sides opposite — Q key + auto mode
    sendSerialCommand('mv_turnleft', [speed])

def moveTurnRight(speed):
    # TANK ROTATE CW — both sides opposite — E key + auto mode
    sendSerialCommand('mv_turnright', [speed])

def stopAll():
    sendSerialCommand('stop', [0])

def moveCurve(left_speed, right_speed):
    sendSerialCommand('mv_curve', [left_speed, right_speed])

# ---------------------------------------------------------------------------
# IR sensor helpers  (5-sensor arc on front semicircle)
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
IDX_W  = 0
IDX_NW = 1
IDX_N  = 2
IDX_NE = 3
IDX_E  = 4

def get_ir_bits():
    with ir_lock:
        v = ir_status
    if REVERSE_SENSOR_ORDER:
        return [(v >> (4 - i)) & 1 for i in range(5)]
    return [(v >> i) & 1 for i in range(5)]

# ---------------------------------------------------------------------------
# FSM pattern sets (5 sensors — pattern bits: E(b4) NE(b3) N(b2) NW(b1) W(b0))
# 1 = sees black line, 0 = sees white / no line
# ---------------------------------------------------------------------------
STRAIGHT_PATTERNS = {
    0b00100,  # N only — perfectly centred
    0b01110,  # NE+N+NW — wide line, centred
    0b01010,  # NE+NW — symmetric, centred
}

GENTLE_LEFT_PATTERNS = {
    0b00110,  # N+NW — line drifting left
    0b00010,  # NW only
}

HARD_LEFT_PATTERNS = {
    0b00001,  # W only — line far left
    0b00011,  # NW+W
}

CURVE_LEFT_PATTERNS = {
    0b00111,  # W+NW+N — sharp left curve (3 sensors)
}

GENTLE_RIGHT_PATTERNS = {
    0b01100,  # NE+N — line drifting right
    0b01000,  # NE only
}

HARD_RIGHT_PATTERNS = {
    0b10000,  # E only — line far right
    0b11000,  # NE+E
}

CURVE_RIGHT_PATTERNS = {
    0b11100,  # N+NE+E — sharp right curve (3 sensors)
}

END_PATTERN = 0b11111  # all 5 sensors = finish line

STATE_NAMES = {
    STATE_FOLLOWING:     "FOLLOW",
    STATE_ENDING:        "ENDING",
    STATE_RECOVERY_FWD:  "RECOV_F",
    STATE_RECOVERY_BACK: "RECOV_B",
    STATE_STOPPED:       "STOPPED",
}

# ---------------------------------------------------------------------------
# Line following FSM — smooth differential steering, adaptive speed
# Uses moveCurve(left, right) instead of tank rotations for stability
# ---------------------------------------------------------------------------
def line_follow_step():
    global auto_state, state_start, adaptive_speed, lost_since

    bits = get_ir_bits()
    w  = bits[IDX_W]
    nw = bits[IDX_NW]
    n  = bits[IDX_N]
    ne = bits[IDX_NE]
    e  = bits[IDX_E]

    pattern = (e << 4) | (ne << 3) | (n << 2) | (nw << 1) | w
    now = time.monotonic()

    cruise = max(MIN_SPEED, min(MAX_SPEED, current_speed))

    # ---- STATE: ENDING ----
    if auto_state == STATE_ENDING:
        if now - state_start >= END_COAST_TIME:
            stopAll()
            auto_state = STATE_STOPPED
        return

    # ---- STATE: STOPPED ----
    if auto_state == STATE_STOPPED:
        return

    # ---- STATE: RECOVERY FORWARD ----
    if auto_state == STATE_RECOVERY_FWD:
        if pattern != 0b00000:
            auto_state = STATE_FOLLOWING
            lost_since = 0.0
            return
        if now - state_start >= RECOVERY_FWD_TIME:
            auto_state = STATE_RECOVERY_BACK
            state_start = now
            moveCurve(-RECOVERY_SPEED, -RECOVERY_SPEED)
        return

    # ---- STATE: RECOVERY BACK ----
    if auto_state == STATE_RECOVERY_BACK:
        if pattern != 0b00000:
            auto_state = STATE_FOLLOWING
            lost_since = 0.0
            return
        if now - state_start >= RECOVERY_BACK_TIME:
            stopAll()
            auto_state = STATE_FOLLOWING
            lost_since = 0.0
        return

    # ---- STATE: FOLLOWING ----

    # End condition — all 5 sensors
    if pattern == END_PATTERN:
        auto_state = STATE_ENDING
        state_start = now
        lost_since = 0.0
        moveCurve(END_COAST_SPEED, END_COAST_SPEED)
        print(">>> END LINE DETECTED")
        return

    # Lost line — debounced: must see nothing for LOST_LINE_TIMEOUT seconds
    if pattern == 0b00000:
        if lost_since == 0.0:
            lost_since = now
        if now - lost_since >= LOST_LINE_TIMEOUT:
            auto_state = STATE_RECOVERY_FWD
            state_start = now
            moveCurve(RECOVERY_SPEED, RECOVERY_SPEED)
            print(">>> LINE LOST — recovery started")
        return
    else:
        lost_since = 0.0

    # Adaptive speed: ramp on straights, use tunable values on corrections
    if pattern in STRAIGHT_PATTERNS:
        adaptive_speed = min(cruise, adaptive_speed + RAMP_UP_STEP)
        moveCurve(adaptive_speed, adaptive_speed)

    elif pattern in GENTLE_LEFT_PATTERNS:
        adaptive_speed = GENTLE_FAST
        moveCurve(GENTLE_SLOW, GENTLE_FAST)

    elif pattern in HARD_LEFT_PATTERNS:
        adaptive_speed = HARD_FAST
        moveCurve(HARD_SLOW, HARD_FAST)

    elif pattern in CURVE_LEFT_PATTERNS:
        adaptive_speed = CURVE_FAST
        moveCurve(CURVE_SLOW, CURVE_FAST)

    elif pattern in GENTLE_RIGHT_PATTERNS:
        adaptive_speed = GENTLE_FAST
        moveCurve(GENTLE_FAST, GENTLE_SLOW)

    elif pattern in HARD_RIGHT_PATTERNS:
        adaptive_speed = HARD_FAST
        moveCurve(HARD_FAST, HARD_SLOW)

    elif pattern in CURVE_RIGHT_PATTERNS:
        adaptive_speed = CURVE_FAST
        moveCurve(CURVE_FAST, CURVE_SLOW)

    else:
        adaptive_speed = UNKNOWN_FAST
        moveCurve(UNKNOWN_FAST, UNKNOWN_SLOW)

    print(f"IR W={w} NW={nw} N={n} NE={ne} E={e} | pat={pattern:05b} | spd={adaptive_speed} | {STATE_NAMES[auto_state]}")

# ---------------------------------------------------------------------------
# UART receive + ping thread
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
                    value = int(value_str) & 0x1F
                    with ir_lock:
                        ir_status = value
                    print(f"IR raw={value:05b}")
                except ValueError:
                    print(f"Bad IR_STATUS: {text}")
            else:
                print(f"UART: {text}")

        now = time.monotonic()
        if now - last_ping >= PING_INTERVAL:
            ser.write(b"hello from raspberry pi\n")
            last_ping = now

# ---------------------------------------------------------------------------
# Ctrl+C handler
# ---------------------------------------------------------------------------
def handle_sigint(sig, frame):
    global running
    running = False
    stopAll()
    sys.exit(0)

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    signal.signal(signal.SIGINT, handle_sigint)

    t_uart = threading.Thread(target=uart_thread, daemon=True)
    t_uart.start()

    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("Mecanum Line Follower")
    font = pygame.font.SysFont(None, 28)

    pressed = {'w': False, 's': False, 'a': False,
               'd': False, 'q': False, 'e': False}

    print("M=toggle auto | W/S/A/D=manual | Q/E=turn | Space=stop | 1/2=speed | Esc=quit")
    print(f"Speed: {current_speed}")

    def update_movement():
        if auto_mode:
            return
        speed = max(MIN_SPEED, min(MAX_SPEED, current_speed))
        if   pressed['w']: moveForward(speed)
        elif pressed['s']: moveReverse(speed)
        elif pressed['q']: moveTurnLeft(speed)     # Q = tank rotate left
        elif pressed['e']: moveTurnRight(speed)    # E = tank rotate right
        elif pressed['a']: moveLeft(speed)         # A = strafe left (manual only)
        elif pressed['d']: moveRight(speed)        # D = strafe right (manual only)
        else:              stopAll()

    clock = pygame.time.Clock()

    while running:
        # ---- HUD ----
        screen.fill((30, 30, 30))
        mode_color = (0, 220, 100) if auto_mode else (220, 180, 0)
        mode_label = "AUTO  (line following)" if auto_mode else "MANUAL  (WASD+QE)"
        bits = get_ir_bits()
        pat  = (bits[IDX_E] << 4) | (bits[IDX_NE] << 3) | (bits[IDX_N] << 2) | (bits[IDX_NW] << 1) | bits[IDX_W]

        screen.blit(font.render(f"MODE: {mode_label}",                         True, mode_color),    (20, 20))
        screen.blit(font.render(f"Speed: {current_speed}  Adapt: {adaptive_speed}",  True, (200,200,200)), (20, 60))
        screen.blit(font.render(f"W={bits[IDX_W]} NW={bits[IDX_NW]} N={bits[IDX_N]} NE={bits[IDX_NE]} E={bits[IDX_E]}", True, (100,180,255)), (20, 100))
        screen.blit(font.render(f"Pattern: {pat:05b}  ({pat})",                True, (100,180,255)), (20, 135))
        state_label = STATE_NAMES.get(auto_state, "?") if auto_mode else "-"
        screen.blit(font.render(f"State: {state_label}",                       True, (180,220,180)), (20, 170))
        screen.blit(font.render("M=auto  1/2=speed  Esc=quit",                 True, (120,120,120)), (20, 240))
        pygame.display.flip()

        # ---- Events ----
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
                        adaptive_speed = current_speed
                        lost_since = 0.0
                    print("AUTO ON" if auto_mode else "MANUAL")

                elif event.key == pygame.K_w:      pressed['w'] = True;  update_movement()
                elif event.key == pygame.K_s:      pressed['s'] = True;  update_movement()
                elif event.key == pygame.K_a:      pressed['a'] = True;  update_movement()
                elif event.key == pygame.K_d:      pressed['d'] = True;  update_movement()
                elif event.key == pygame.K_q:      pressed['q'] = True;  update_movement()
                elif event.key == pygame.K_e:      pressed['e'] = True;  update_movement()
                elif event.key == pygame.K_SPACE:  pressed = {k: False for k in pressed}; stopAll()
                elif event.key == pygame.K_ESCAPE: running = False

                elif event.key == pygame.K_1:
                    current_speed = max(MIN_SPEED, current_speed - 5)
                    print(f"Speed -> {current_speed}");  update_movement()
                elif event.key == pygame.K_2:
                    current_speed = min(MAX_SPEED, current_speed + 5)
                    print(f"Speed -> {current_speed}");  update_movement()

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
