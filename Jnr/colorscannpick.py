# ============================================================
# WRO JUNIOR — TWIN BARRIER GRABBERS: SCAN + PICK 2 + DROP   (v4)
#
# Bot has LEFT and RIGHT boom-barrier grabbers, 130 mm apart =
# slot pitch. Pick: centre between two slots, nose in, close both.
# Drop: four cases, decided from what is already placed.
#
#   drop_both  : both targets adjacent AND in the same left/right
#                order as the grabbers -> one nose-in, open both.
#   drop plain : the grabber whose neighbour target (where the
#                OTHER grabber sits) is EMPTY drops first, normal
#                nose-in. Second object then drops plain too
#                (an empty open barrier clears a placed block).
#   drop push  : both neighbours occupied -> release SHORT of the
#                box, back off, close barrier, push the block in.
#
# Layout (shared x-axis, mm): pick slots x = 0/130/260/390,
# drop targets RED -65, GREEN 65, BLACK 195, BLUE 325, YELLOW 455.
# Rows CROSS_MM apart. Plain spot turns only. Grabbers absolute.
# ============================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# ================= GEOMETRY =================
SLOT_PITCH = 130
PICK_X     = [0, 130, 260, 390]
DROP_X0    = -65
DROP_ORDER = ["RED", "GREEN", "BLACK", "BLUE", "YELLOW"]
DROP_X     = {c: DROP_X0 + SLOT_PITCH * i for i, c in enumerate(DROP_ORDER)}
CROSS_MM   = 550

SCAN_FIRST_BLOCK_MM = 50
SCAN_LENGTH_MM      = 580

# Grabber x-offset from bot centre WHEN FACING A ROW (+ = toward +x).
# Pick row: slot 3 (higher x) goes in the LEFT grabber  -> left is +65.
# Drop row is faced from the other side, so left/right mirror -> left is -65.
HALF = SLOT_PITCH / 2
PICK_DX = {"L": +HALF, "R": -HALF}
DROP_DX = {"L": -HALF, "R": +HALF}      # flip both signs if drops mirror

# ================= DIRECTION SIGNS =================
PICK_SIDE_TURN = -90   # heading -x on pick line -> face slots
CROSS_TURN     = +90   # used twice per crossing
DROP_SIDE_TURN = -90   # heading +x on drop line -> face targets

# ================= OVERALL TRIMS =================
SCAN_END_X_TRIM = 0
CROSS_TRIM      = 0
DROP_X_TRIM_ALL = 0

# ================= INDIVIDUAL TRIMS =================
PICK_X_TRIM = {(3, 2): 20, (1, 0): 20}      # per pair, mm along x
PICK_ADV    = {(3, 2): 140, (1, 0): 140}    # nose-in, mm
DROP_X_TRIM = {"RED": 10, "GREEN": 10, "BLACK": 10, "BLUE": 10, "YELLOW": 10}
DROP_ADV    = {"RED": 120, "GREEN": 120, "BLACK": 120, "BLUE": 120, "YELLOW": 120}   # to RELEASE point
DROP_CLEAR  = 10     # extra back-out after every drop so the turn clears the blocks

# push-drop knobs
RELEASE_BACK = 50    # back off after releasing, before closing the barrier
PUSH_DEPTH   = 130   # push this far PAST the release point (was 130+80-80)

# ================= GRABBERS  (EDIT: ports + angles) =================
LEFT_GRAB_PORT  = Port.D
RIGHT_GRAB_PORT = Port.F
LEFT_OPEN,  LEFT_CLOSED  = 90, 0
RIGHT_OPEN, RIGHT_CLOSED = -90, 0
GRAB_SPEED = 300

RUN_SPEED  = 300
PICK_SPEED = 300
DROP_SPEED = 200
SCAN_SPEED = 400
MIN_GAP    = 40

# ================= PORTS (frozen) =================
LEFT_MOTOR_PORT  = Port.A
RIGHT_MOTOR_PORT = Port.E
COLOR_SIDE_PORT  = Port.C
LEFT_MOTOR_REVERSED  = True
RIGHT_MOTOR_REVERSED = False
WHEEL_DIAMETER = 56
AXLE_TRACK     = 165

# ------------------------------------------------------------
hub = PrimeHub()
left_motor = Motor(LEFT_MOTOR_PORT,
    Direction.COUNTERCLOCKWISE if LEFT_MOTOR_REVERSED else Direction.CLOCKWISE)
right_motor = Motor(RIGHT_MOTOR_PORT,
    Direction.COUNTERCLOCKWISE if RIGHT_MOTOR_REVERSED else Direction.CLOCKWISE)
left_grab  = Motor(LEFT_GRAB_PORT)       # bare — never add direction
right_grab = Motor(RIGHT_GRAB_PORT)      # bare — never add direction
side_sensor = ColorSensor(COLOR_SIDE_PORT)

drive = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
drive.use_gyro(True)
drive.settings(straight_speed=RUN_SPEED)

timer = StopWatch()
def log(msg):
    print("[{:>5.1f}s] {}".format(timer.time() / 1000, msg))

# ============================================================
# SCAN (SCAN4 v3 — proven chain, unchanged)
# ============================================================
SAT_MIN, V_MIN = 55, 2
BLACK_S_MAX, BLACK_V_MIN = 30, 2

def classify(h, s, v):
    if s >= SAT_MIN and v >= V_MIN:
        if h >= 330 or h <= 15:  return "RED"
        if 25 <= h <= 55:        return "YELLOW"
        if 140 <= h <= 185:      return "GREEN"
        if 200 <= h <= 235:      return "BLUE"
    if s <= BLACK_S_MAX and v >= BLACK_V_MIN:
        return "BLACK"
    return None

def scan_row(length_mm):
    found = []
    last, hits, entry_pos, peak_v, rearm_at, n_blocks = None, 0, 0, 0, 0, 0
    d0 = drive.distance()
    drive.drive(SCAN_SPEED, 0)
    while drive.distance() - d0 < length_mm:
        d = drive.distance() - d0
        if d >= rearm_at:
            hsv = side_sensor.hsv()
            c = classify(hsv.h, hsv.s, hsv.v)
            if c is not None and c == last:
                hits += 1; peak_v = max(peak_v, hsv.v)
            elif c is not None:
                last, hits, entry_pos, peak_v = c, 1, d, hsv.v
            else:
                last, hits = None, 0
            if hits >= 2:
                if not (found and found[-1][1] == c and entry_pos - found[-1][0] < 70):
                    n_blocks += 1
                    print("Color {} detected: {} at {} mm  (h={} s={} v={})"
                          .format(n_blocks, c, int(entry_pos), hsv.h, hsv.s, hsv.v))
                found.append((entry_pos, c, peak_v))
                rearm_at = d + MIN_GAP
                last, hits = None, 0
        wait(20)
    drive.stop()
    return found

def merge(detections):
    merged = []
    for pos, c, pv in detections:
        if merged and merged[-1][1] == c and pos - merged[-1][0] < 70:
            p0, c0, pv0 = merged[-1]; merged[-1] = (p0, c0, max(pv0, pv)); continue
        merged.append((pos, c, pv))
    return merged

def build_slots(detections, first_block_mm):
    notes  = merge(detections)
    chroma = [(p, c) for p, c, _ in notes if c != "BLACK"]
    darks  = [p for p, c, _ in notes if c == "BLACK"]
    colors = [c for _, c in chroma]
    pos    = [p for p, _ in chroma]
    slots  = ["???"] * 4
    black_idx = None
    if len(chroma) == 4:
        slots = colors[:]
    elif len(chroma) == 3:
        g1, g2 = pos[1] - pos[0], pos[2] - pos[1]
        small = min(g1, g2)
        if g1 > 1.5 * small:   black_idx = 1
        elif g2 > 1.5 * small: black_idx = 2
        elif pos[0] < first_block_mm + 0.5 * small: black_idx = 3
        else:                  black_idx = 0
        slots = colors[:]; slots.insert(black_idx, "BLACK")
        print("BLACK -> slot {} by gaps ({} / {} mm, first block at {} mm)"
              .format(black_idx, int(g1), int(g2), int(pos[0])))
    else:
        print("WARNING: {} colors seen:".format(len(chroma)), colors)
        return slots
    if darks:
        dp = darks[0]; seen_idx = sum(1 for p in pos if p < dp)
        if black_idx is None:
            print("WARNING: sensor saw BLACK at {} mm but all 4 colors found".format(int(dp)))
        elif seen_idx != black_idx:
            print("WARNING: sensor BLACK at {} mm says slot {}, gaps say slot {}"
                  .format(int(dp), seen_idx, black_idx))
        else:
            print("BLACK confirmed by sensor at {} mm".format(int(dp)))
    elif black_idx is not None:
        print("BLACK not seen by sensor; filled by gaps only")
    return slots

def scan_colors(first_block_mm, length_mm):
    print("Scan start: first block ~{} mm, length {} mm".format(first_block_mm, length_mm))
    dets  = scan_row(length_mm)
    slots = build_slots(dets, first_block_mm)
    print("\n===== SLOTS =====")
    for i in range(4):
        print("  Slot {} : {}".format(i, slots[i]))
    print("=================")
    return slots

# ============================================================
# GRABBERS — absolute, stall-protected
# ============================================================
def _grab_goto(motor, target, name):
    motor.run_target(GRAB_SPEED, target, then=Stop.HOLD, wait=False)
    while not motor.done():
        if motor.stalled():
            motor.hold(); log("  {} stalled at {}".format(name, motor.angle())); break
        wait(10)

def grab_open(side):
    if side == "L": _grab_goto(left_grab,  LEFT_OPEN,  "left grabber")
    else:           _grab_goto(right_grab, RIGHT_OPEN, "right grabber")

def grab_close(side):
    if side == "L": _grab_goto(left_grab,  LEFT_CLOSED,  "left grabber")
    else:           _grab_goto(right_grab, RIGHT_CLOSED, "right grabber")

def open_both():  grab_open("L");  grab_open("R")
def close_both(): grab_close("L"); grab_close("R")

# ============================================================
# X-AXIS FRAME
# ============================================================
x_mm = 0.0
heading_sign = +1

def goto_x(target_mm):
    global x_mm
    delta = (target_mm - x_mm) * heading_sign
    log("  goto x {:.0f} -> {:.0f} (drive {:+.0f})".format(x_mm, target_mm, delta))
    if abs(delta) > 2:
        drive.settings(straight_speed=RUN_SPEED)
        drive.straight(delta)
    x_mm = target_mm

def flip_heading():
    global heading_sign
    heading_sign = -heading_sign

# ============================================================
# PICK PAIR — bot centred between the two slots, grab both
# ============================================================
def pick_pair(pair):
    hi, lo = pair
    xc = (PICK_X[hi] + PICK_X[lo]) / 2 + PICK_X_TRIM[pair]
    log("PICK pair {} at x {:.0f}, adv {}".format(pair, xc, PICK_ADV[pair]))
    goto_x(xc)
    hub.speaker.beep(800, 150)
    drive.turn(PICK_SIDE_TURN)
    open_both()
    drive.settings(straight_speed=PICK_SPEED)
    drive.straight(PICK_ADV[pair])
    close_both()
    drive.straight(-PICK_ADV[pair])
    drive.turn(-PICK_SIDE_TURN)

# ============================================================
# DROP PRIMITIVES — bot on drop line; each returns to line heading
# ============================================================
def target_at_x(x):
    """Color whose target centre is at x (±10 mm), else None."""
    for c, tx in DROP_X.items():
        if abs(tx - x) < 10:
            return c
    return None

drop_y_off = 0.0     # how far the bot has crept away from the drop row

def _note_clear():
    global drop_y_off
    drop_y_off += DROP_CLEAR

def _geom_centre(side, color):
    """Bot centre x for this grabber over this target — RAW geometry, no trims."""
    return DROP_X[color] - DROP_DX[side]

def _centre_for(side, color, trim=None):
    """Drive-to x: geometry + overall trim + per-color trim (or a given trim)."""
    t = DROP_X_TRIM[color] if trim is None else trim
    return _geom_centre(side, color) + DROP_X_TRIM_ALL + t

def drop_plain(side, color):
    log("DROP {} from {} (plain)".format(color, side))
    goto_x(_centre_for(side, color))
    hub.speaker.beep(500, 150)
    drive.turn(DROP_SIDE_TURN)
    drive.settings(straight_speed=DROP_SPEED)
    adv = DROP_ADV[color] + drop_y_off   # absorb earlier clearances
    drive.straight(adv)                  # straight to release point
    grab_open(side)
    wait(200)
    drive.straight(-(adv + DROP_CLEAR))
    _note_clear()
    drive.turn(-DROP_SIDE_TURN)

def drop_push(side, color):
    log("DROP {} from {} (PUSH: neighbour occupied)".format(color, side))
    goto_x(_centre_for(side, color))
    hub.speaker.beep(500, 150)
    drive.turn(DROP_SIDE_TURN)
    drive.settings(straight_speed=DROP_SPEED)
    adv = DROP_ADV[color] + drop_y_off    # absorb earlier clearances
    drive.straight(adv)                   # to release point
    grab_open(side)
    wait(200)
    drive.straight(-RELEASE_BACK)         # clear the block
    grab_close(side)                      # barrier down = pusher
    drive.straight(RELEASE_BACK + PUSH_DEPTH)   # push it in
    drive.straight(-(adv + PUSH_DEPTH + DROP_CLEAR))
    _note_clear()
    grab_open(side)
    drive.turn(-DROP_SIDE_TURN)

def drop_both(colorL, colorR):
    trim = min(DROP_X_TRIM[colorL], DROP_X_TRIM[colorR])
    xc = _centre_for("L", colorL, trim)
    log("DROP BOTH {} (L) + {} (R) at x {:.0f}".format(colorL, colorR, xc))
    goto_x(xc)
    hub.speaker.beep(600, 150)
    drive.turn(DROP_SIDE_TURN)
    drive.settings(straight_speed=DROP_SPEED)
    adv = max(DROP_ADV[colorL], DROP_ADV[colorR]) + drop_y_off
    drive.straight(adv)                  # straight to release point
    open_both()
    wait(200)
    drive.straight(-(adv + DROP_CLEAR))
    _note_clear()
    drive.turn(-DROP_SIDE_TURN)

# ============================================================
# DROP DECISION — the four cases
# ============================================================
def drop_pair(colorL, colorR, placed):
    """colorL/colorR = objects in left/right grabber. placed = set of colors
    already on targets. Adds the two colors to placed."""
    other = {"L": "R", "R": "L"}
    held  = {"L": colorL, "R": colorR}

    # case 1: adjacent and same order as grabbers -> one shot (raw geometry)
    if abs(_geom_centre("L", colorL) - _geom_centre("R", colorR)) < 10:
        drop_both(colorL, colorR)
        placed.update([colorL, colorR]); return

    # what sits under the OTHER grabber while each one drops? (raw geometry)
    def neighbour_occupied(side):
        xc = _geom_centre(side, held[side])
        n  = target_at_x(xc + DROP_DX[other[side]])
        return n is not None and n in placed

    occL, occR = neighbour_occupied("L"), neighbour_occupied("R")
    log("  neighbour occupied: L={} R={}".format(occL, occR))

    # drop the free-neighbour one first; push is used for ANY drop
    # whose neighbour is occupied
    if not occL:   order = ["L", "R"]
    elif not occR: order = ["R", "L"]
    else:          order = ["L", "R"]

    for side in order:
        if neighbour_occupied(side):
            drop_push(side, held[side])
        else:
            drop_plain(side, held[side])
        placed.add(held[side])

# ============================================================
# CROSSING
# ============================================================
def cross_to_drop():
    log("CROSS pick -> drop at x {:.0f}".format(x_mm))
    drive.turn(CROSS_TURN)
    drive.settings(straight_speed=RUN_SPEED)
    drive.straight(CROSS_MM + CROSS_TRIM)
    drive.turn(CROSS_TURN)
    flip_heading()

def cross_to_pick():
    global drop_y_off
    log("CROSS drop -> pick at x {:.0f} (y_off {:.0f})".format(x_mm, drop_y_off))
    drive.turn(CROSS_TURN)
    drive.settings(straight_speed=RUN_SPEED)
    drive.straight(CROSS_MM + CROSS_TRIM - drop_y_off)
    drop_y_off = 0.0
    drive.turn(CROSS_TURN)
    flip_heading()    

# ============================================================
# RUN
# ============================================================
timer.reset()
drive.reset()
open_both()

# --- your approach moves to the scan start, blocks on the RIGHT ---
# drive.straight(...)

slots = scan_colors(SCAN_FIRST_BLOCK_MM, SCAN_LENGTH_MM)
if "???" in slots:
    log("scan failed — stopping"); drive.stop(); raise SystemExit

x_mm = SCAN_LENGTH_MM - SCAN_FIRST_BLOCK_MM - 30 + SCAN_END_X_TRIM
heading_sign = +1
log("scan end: x {:.0f}".format(x_mm))

drive.turn(-90)
drive.straight(50)
drive.turn(-90)
flip_heading()                      # heading -x, slots on the left

placed = set()

# trip 1: slot 3 -> LEFT grabber, slot 2 -> RIGHT grabber
pick_pair((3, 2))
cross_to_drop()
drop_pair(slots[3], slots[2], placed)
cross_to_pick()

# trip 2: slot 1 -> LEFT, slot 0 -> RIGHT
pick_pair((1, 0))
cross_to_drop()
drop_pair(slots[1], slots[0], placed)

drive.stop()
log("ALL DONE. placed: {}".format(sorted(placed)))
