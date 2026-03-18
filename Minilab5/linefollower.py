import time
import serial
import threading
import signal
import sys
from dataclasses import dataclass
import pygame

# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------
running   = True
auto_mode = False

ir_status = 0
ir_lock   = threading.Lock()

PING_INTERVAL = 5  # seconds

# ---------------------------------------------------------------------------
# Speed config
# ---------------------------------------------------------------------------
@dataclass
class SetConfig:
    min_speed:     int   = 5
    max_speed:     int   = 100
    default_speed: int   = 50
    gentle_factor: float = 0.6

setConfig     = SetConfig()
current_speed = setConfig.default_speed

# ---------------------------------------------------------------------------
# UART
# ---------------------------------------------------------------------------
ser = serial.Serial('/dev/ttyAMA2', 115200, timeout=1)

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
    return [(v >> i) & 1 for i in range(5)]

# ---------------------------------------------------------------------------
# FSM pattern sets (5 sensors: bit4=E, bit3=NE, bit2=N, bit1=NW, bit0=W)
# 1 = sees black line, 0 = sees white / no line
#
# Pattern bits:  E  NE  N  NW  W
#               b4  b3 b2  b1 b0
# ---------------------------------------------------------------------------
#                          E NE  N NW  W
STRAIGHT_PATTERNS = {
    0b00100,  # N only — perfectly centred
    0b01110,  # NE+N+NW — wide line, centred
    0b01010,  # NE+NW — symmetric, centred
    0b11111,  # all sensors — intersection / wide marker
}

GENTLE_LEFT_PATTERNS = {
    0b00110,  # N+NW — drifted slightly right → correct left
    0b00010,  # NW only — drifted right
}

HARD_LEFT_PATTERNS = {
    0b00001,  # W only — drifted far right → hard left
    0b00011,  # NW+W — drifted far right
}

GENTLE_RIGHT_PATTERNS = {
    0b01100,  # NE+N — drifted slightly left → correct right
    0b01000,  # NE only — drifted left
}

HARD_RIGHT_PATTERNS = {
    0b10000,  # E only — drifted far left → hard right
    0b11000,  # NE+E — drifted far left
}

STOP_PATTERNS = {
    0b00000,  # no sensor sees line → lost
}

# ---------------------------------------------------------------------------
# Line following FSM
# ALL corrections use moveTurnLeft / moveTurnRight (tank rotation)
# NEVER uses moveLeft / moveRight (those are mecanum strafe)
# ---------------------------------------------------------------------------
def line_follow_step():
    bits = get_ir_bits()
    w  = bits[IDX_W]    # GPIO5
    nw = bits[IDX_NW]   # GPIO6
    n  = bits[IDX_N]    # GPIO7
    ne = bits[IDX_NE]   # GPIO15
    e  = bits[IDX_E]    # GPIO45

    pattern = (e << 4) | (ne << 3) | (n << 2) | (nw << 1) | w

    speed = max(setConfig.min_speed,
                min(setConfig.max_speed, current_speed))

    gentle_speed = max(setConfig.min_speed,
                       int(speed * setConfig.gentle_factor))

    print(f"IR W={w} NW={nw} N={n} NE={ne} E={e} | pattern={pattern:05b} | spd={speed}")

    # ---- FSM ----

    if pattern in STRAIGHT_PATTERNS:
        moveForward(speed)

    elif pattern in GENTLE_LEFT_PATTERNS:
        # Drifted right → turn left to correct
        moveTurnLeft(gentle_speed)

    elif pattern in HARD_LEFT_PATTERNS:
        # Drifted far right → hard left
        moveTurnLeft(speed)

    elif pattern in GENTLE_RIGHT_PATTERNS:
        # Drifted left → turn right to correct
        moveTurnRight(gentle_speed)

    elif pattern in HARD_RIGHT_PATTERNS:
        # Drifted far left → hard right
        moveTurnRight(speed)

    elif pattern in STOP_PATTERNS:
        stopAll()

    else:
        # Unknown pattern → stop for safety
        stopAll()

    # ---- FSM END ----

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
        speed = max(setConfig.min_speed, min(setConfig.max_speed, current_speed))
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
        screen.blit(font.render(f"Speed: {current_speed}",                     True, (200,200,200)), (20, 60))
        screen.blit(font.render(f"W={bits[IDX_W]} NW={bits[IDX_NW]} N={bits[IDX_N]} NE={bits[IDX_NE]} E={bits[IDX_E]}", True, (100,180,255)), (20, 100))
        screen.blit(font.render(f"Pattern: {pat:05b}  ({pat})",                True, (100,180,255)), (20, 135))
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
                    current_speed = max(setConfig.min_speed, current_speed - 5)
                    print(f"Speed -> {current_speed}");  update_movement()
                elif event.key == pygame.K_2:
                    current_speed = min(setConfig.max_speed, current_speed + 5)
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
