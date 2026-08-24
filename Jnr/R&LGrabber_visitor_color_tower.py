# ============================================================
# WRO JUNIOR — TWIN BARRIER GRABBERS: SCAN + PICK 2 + DROP   (v13)
#
# Bot has LEFT and RIGHT boom-barrier grabbers, 130 mm apart =
# slot pitch. Pick: centre between two slots, nose in, close both.
# Drop: four cases, decided from what is already placed.
#
#   drop_both  : both targets adjacent AND in the same left/right
#                order as the grabbers -> one nose-in, open both.
#   drop one   : the grabber whose neighbour target (where the
#                OTHER grabber sits) is EMPTY drops first.
#   EVERY placement = nose in, open, back RELEASE_BACK, close,
#                push PUSH_FWD, back out to the line.
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

#################################################################
#################################################################
##                                                             ##
##   PART 1 — TUNING AREA. Everything you change lives here.   ##
##                                                             ##
#################################################################
#################################################################

# ================= GEOMETRY =================
SLOT_PITCH = 130
PICK_X     = [0, 130, 260, 390]
DROP_X0    = -65
DROP_ORDER = ["RED", "GREEN", "BLACK", "BLUE", "YELLOW"]
DROP_X     = {c: DROP_X0 + SLOT_PITCH * i for i, c in enumerate(DROP_ORDER)}
CROSS_MM   = 550

SCAN_FIRST_BLOCK_MM = 50
SCAN_LENGTH_MM      = 580
# Used when the scan is partial or fails: unknown slots are filled from
# this order, WITHOUT duplicating colors the scan already identified.
SLOTS_FALLBACK = ["BLUE", "GREEN", "RED", "YELLOW"]   # slot 0,1,2,3

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
PICK_X_TRIM = {(3, 2): 50, (1, 0): 30}      # per pair, mm along x
PICK_ADV    = {(3, 2): 140, (1, 0): 140}    # nose-in, mm
DROP_X_TRIM = {"RED": 80, "GREEN": 100, "BLACK": 20, "BLUE": 50, "YELLOW": 10}
DROP_ADV    = {"RED": 80, "GREEN": 80, "BLACK": 80, "BLUE": 50, "YELLOW": 80}   # to RELEASE point
DROP_CLEAR  = 10     # extra back-out after every drop so the turn clears the blocks

# ================= PHASE 2 — NEW ROW + ZONES =================
NEW_PITCH   = 130
NEW_X0      = -415                       # B centre (41.5 cm beyond slot 0)
NEW_X       = {"B": NEW_X0, "BL": NEW_X0 - NEW_PITCH,
               "G": NEW_X0 - 2 * NEW_PITCH, "R": NEW_X0 - 3 * NEW_PITCH}
NEW_PAIR_X_TRIM = 0                      # B+BL pair pick, mm along x
NEW_PAIR_ADV    = 130
NEW_G_X_TRIM, NEW_G_ADV = 0, 130         # single picks
NEW_R_X_TRIM, NEW_R_ADV = 0, 130

LEFT_WALL_X = 1290                       # slot 0 centre -> left wall
BL_X        = LEFT_WALL_X - 210          # zone centre along x
B_X         = LEFT_WALL_X - 210
# zone placement: bot drives to (zone x - grabber offset), turns FACE_TURN
# from the line heading, noses in ADV, opens, backs out ADV + DROP_CLEAR
BL_FACE_TURN, BL_ADV, BL_X_TRIM = +90, 60, 0     # from pick line, heading +x
B_FACE_TURN,  B_ADV,  B_X_TRIM  = +90, 60, 0     # from drop line, heading -x

G_SLOT_X, G_ADV, G_X_TRIM = PICK_X[0], 100, 0    # G goes to slot 0
R_SAFE_X, R_ADV, R_X_TRIM = DROP_X["RED"] - 100, 100, 0   # 7x17 box right of red

# release-and-push knobs (used for EVERY placement)
RELEASE_BACK = 100   # after opening: back this far, then close the barrier
PUSH_FWD     = 80    # ...then push forward this far, then back out to the line

# ================= GRABBERS  (EDIT: ports + angles) =================
LEFT_GRAB_PORT  = Port.D
RIGHT_GRAB_PORT = Port.F
LEFT_OPEN,  LEFT_CLOSED  = 90, 0
RIGHT_OPEN, RIGHT_CLOSED = -90, 0
GRAB_SPEED = 300
GRAB_STALL_SPEED = 30    # deg/s: below this counts as "not moving"
GRAB_STALL_MS    = 150   # ...for this long = stalled on the object

RUN_SPEED  = 350
PICK_SPEED = 150
DROP_SPEED = 300
SCAN_SPEED = 350   # 300 was dropping detections
MIN_GAP    = 40

# --- MOTION TUNING (speed vs accuracy test knobs) ---
# Applies to the WHOLE run. Change ONE per test run.
# Later drive.settings(straight_speed=...) calls do NOT
# overwrite these — they persist.
TURN_RATE    = 300    # deg/s. Pybricks default ~130. Try 200 -> 250 -> 300.
TURN_ACC     = 500    # deg/s^2. Turn snap. Try 400 -> 600.
STRAIGHT_ACC = 500    # mm/s^2. Default ~228. Try 400 -> 600.

# ================= PORTS (frozen) =================
LEFT_MOTOR_PORT  = Port.A
RIGHT_MOTOR_PORT = Port.E
COLOR_SIDE_PORT  = Port.C
LEFT_MOTOR_REVERSED  = True
RIGHT_MOTOR_REVERSED = False
WHEEL_DIAMETER = 56
AXLE_TRACK     = 165

#################################################################
#################################################################
##                                                             ##
##   PART 2 — ENGINE. No tuning below this line: hardware,     ##
##   scan, grabbers, x-frame, pick/drop logic. Do not edit.    ##
##                                                             ##
#################################################################
#################################################################

# ------------------------------------------------------------
hub = PrimeHub()
hub.imu.settings(heading_correction=362.9)
left_motor = Motor(LEFT_MOTOR_PORT,
    Direction.COUNTERCLOCKWISE if LEFT_MOTOR_REVERSED else Direction.CLOCKWISE)
right_motor = Motor(RIGHT_MOTOR_PORT,
    Direction.COUNTERCLOCKWISE if RIGHT_MOTOR_REVERSED else Direction.CLOCKWISE)
left_grab  = Motor(LEFT_GRAB_PORT)       # bare — never add direction
right_grab = Motor(RIGHT_GRAB_PORT)      # bare — never add direction
side_sensor = ColorSensor(COLOR_SIDE_PORT)

drive = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
drive.use_gyro(True)
drive.settings(straight_speed=RUN_SPEED,
               straight_acceleration=STRAIGHT_ACC,
               turn_rate=TURN_RATE,
               turn_acceleration=TURN_ACC)

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

def _grab_close_stall(motor, open_deg, closed_deg, name):
    """Close to closed_deg (absolute). Stop early and HOLD if the barrier
    stops moving for GRAB_STALL_MS (object in the way), else stop at the angle."""
    motor.run_target(GRAB_SPEED, closed_deg, then=Stop.HOLD, wait=False)
    t, slow_since, why = StopWatch(), None, "angle"
    while not motor.done():
        now = t.time()
        if now > 1500:
            why = "timeout"; break
        moving = abs(motor.speed()) > GRAB_STALL_SPEED
        near   = abs(motor.angle() - closed_deg) < 8
        if now > 150 and not moving and not near:
            if slow_since is None:
                slow_since = now
            elif now - slow_since >= GRAB_STALL_MS:
                why = "stall"; break
        else:
            slow_since = None
        wait(10)
    motor.hold()
    log("  {} closed at {} ({})".format(name, motor.angle(), why))

def grab_close(side):
    if side == "L": _grab_close_stall(left_grab,  LEFT_OPEN,  LEFT_CLOSED,  "left grabber")
    else:           _grab_close_stall(right_grab, RIGHT_OPEN, RIGHT_CLOSED, "right grabber")

def open_both():
    """Both grabbers to OPEN in parallel."""
    left_grab.run_target(GRAB_SPEED, LEFT_OPEN, then=Stop.HOLD, wait=False)
    right_grab.run_target(GRAB_SPEED, RIGHT_OPEN, then=Stop.HOLD, wait=False)
    while not (left_grab.done() and right_grab.done()):
        if left_grab.stalled():
            left_grab.hold()
        if right_grab.stalled():
            right_grab.hold()
        wait(10)

def close_both():
    """Both grabbers close in parallel; each stops at its angle OR on a
    real stall (speed-based), independently."""
    targets = {"L": LEFT_CLOSED, "R": RIGHT_CLOSED}
    motors  = {"L": left_grab,   "R": right_grab}
    for sd in ("L", "R"):
        motors[sd].run_target(GRAB_SPEED, targets[sd], then=Stop.HOLD, wait=False)
    t = StopWatch()
    slow_since = {"L": None, "R": None}
    done = {"L": False, "R": False}
    while not (done["L"] and done["R"]):
        now = t.time()
        if now > 1500:
            for sd in ("L", "R"):
                if not done[sd]:
                    motors[sd].hold()
                    log("  {} grabber close timeout at {}".format(sd, motors[sd].angle()))
            break
        for sd in ("L", "R"):
            if done[sd]:
                continue
            m = motors[sd]
            if m.done():
                done[sd] = True
                log("  {} grabber closed at {} (angle)".format(sd, m.angle()))
                continue
            moving = abs(m.speed()) > GRAB_STALL_SPEED
            near   = abs(m.angle() - targets[sd]) < 8
            if now > 150 and not moving and not near:
                if slow_since[sd] is None:
                    slow_since[sd] = now
                elif now - slow_since[sd] >= GRAB_STALL_MS:
                    m.hold()
                    done[sd] = True
                    log("  {} grabber closed at {} (stall)".format(sd, m.angle()))
            else:
                slow_since[sd] = None
        wait(10)

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

def _open_sides(sides):
    if len(sides) == 2: open_both()
    else:               grab_open(sides[0])

def _close_sides(sides):
    if len(sides) == 2: close_both()
    else:               grab_close(sides[0])

def _release_push(sides, adv):
    """At the line, already turned to face the target. sides = ["L"], ["R"] or ["L","R"].
    nose in adv -> open -> back RELEASE_BACK -> close -> push PUSH_FWD -> back to line."""
    drive.settings(straight_speed=DROP_SPEED)
    drive.straight(adv)                      # to release point
    _open_sides(sides)
    wait(200)
    drive.straight(-RELEASE_BACK)            # clear the block(s)
    _close_sides(sides)                      # barrier down = pusher
    drive.straight(PUSH_FWD)                 # push in
    drive.straight(-(adv - RELEASE_BACK + PUSH_FWD + DROP_CLEAR))   # back to line (+clear)
    _note_clear()
    _open_sides(sides)

def drop_one(side, color):
    log("DROP {} from {}".format(color, side))
    goto_x(_centre_for(side, color))
    hub.speaker.beep(500, 150)
    drive.turn(DROP_SIDE_TURN)
    _release_push([side], DROP_ADV[color] + drop_y_off)
    drive.turn(-DROP_SIDE_TURN)

def drop_both(colorL, colorR):
    trim = min(DROP_X_TRIM[colorL], DROP_X_TRIM[colorR])
    xc = _centre_for("L", colorL, trim)
    log("DROP BOTH {} (L) + {} (R) at x {:.0f}".format(colorL, colorR, xc))
    goto_x(xc)
    hub.speaker.beep(600, 150)
    drive.turn(DROP_SIDE_TURN)
    _release_push(["L", "R"], max(DROP_ADV[colorL], DROP_ADV[colorR]) + drop_y_off)
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
        drop_one(side, held[side])
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
# PHASE 2 PRIMITIVES
# ============================================================
def pick_new_pair():
    """B -> LEFT grabber, BL -> RIGHT grabber. Bot on pick line heading -x."""
    xc = (NEW_X["B"] + NEW_X["BL"]) / 2 + NEW_PAIR_X_TRIM
    log("PICK new pair B+BL at x {:.0f}".format(xc))
    goto_x(xc)
    hub.speaker.beep(800, 150)
    drive.turn(PICK_SIDE_TURN)
    open_both()
    drive.settings(straight_speed=PICK_SPEED)
    drive.straight(NEW_PAIR_ADV)
    close_both()
    drive.straight(-NEW_PAIR_ADV)
    drive.turn(-PICK_SIDE_TURN)

def pick_single(side, x_target, x_trim, adv, name):
    """Pick one object into the given grabber. Bot on pick line heading -x."""
    xc = x_target - PICK_DX[side] + x_trim
    log("PICK {} into {} at x {:.0f}".format(name, side, xc))
    goto_x(xc)
    hub.speaker.beep(800, 150)
    drive.turn(PICK_SIDE_TURN)
    grab_open(side)
    drive.settings(straight_speed=PICK_SPEED)
    drive.straight(adv)
    grab_close(side)
    drive.straight(-adv)
    drive.turn(-PICK_SIDE_TURN)

def place_single(side, x_target, x_trim, face_turn, adv, name, dx_table):
    """Release one object: go to x, turn face_turn, nose in adv, open, back out.
    dx_table = PICK_DX or DROP_DX depending on which line the bot is on."""
    xc = x_target - dx_table[side] + x_trim
    log("PLACE {} from {} at x {:.0f} (turn {}, adv {})".format(name, side, xc, face_turn, adv))
    goto_x(xc)
    hub.speaker.beep(500, 150)
    drive.turn(face_turn)
    _release_push([side], adv)
    drive.turn(-face_turn)

def phase2():
    log("===== PHASE 2 =====")
    # 1. park (x 0, drop line, heading +x) -> pick line, heading -x
    cross_to_pick()
    # 2. B + BL
    pick_new_pair()
    # 3. along pick line to BL zone (heading +x). Right grabber holds BL.
    drive.turn(180); flip_heading()                          # now heading +x
    place_single("R", BL_X, BL_X_TRIM, BL_FACE_TURN, BL_ADV, "BL", PICK_DX)
    # 4. cross to drop line at this x, place B from left grabber
    drive.turn(CROSS_TURN)
    drive.settings(straight_speed=RUN_SPEED)
    drive.straight(CROSS_MM + CROSS_TRIM - drop_y_off)
    drive.turn(-CROSS_TURN)                                   # heading +x on drop line
    _reset_y()
    place_single("L", B_X, B_X_TRIM, B_FACE_TURN, B_ADV, "B", DROP_DX)
    # 5. back to pick line (cross_to_pick expects heading +x) -> heading -x
    cross_to_pick()
    pick_single("L", NEW_X["G"], NEW_G_X_TRIM, NEW_G_ADV, "G")
    # G to slot 0: bot heading -x, slots on the left -> same turn as picking
    place_single("L", G_SLOT_X, G_X_TRIM, PICK_SIDE_TURN, G_ADV, "G->slot0", PICK_DX)
    # 6. R
    pick_single("L", NEW_X["R"], NEW_R_X_TRIM, NEW_R_ADV, "R")
    # 7. cross to drop line, place R in safe zone, park
    cross_to_drop()
    place_single("L", R_SAFE_X, R_X_TRIM, DROP_SIDE_TURN, R_ADV, "R->safe", DROP_DX)
    goto_x(0)
    log("PHASE 2 DONE. parked at x 0, heading {:.1f}".format(drive.angle()))

def _reset_y():
    global drop_y_off
    drop_y_off = 0.0

def finalize_slots(slots):
    """Guarantee 4 valid, unique colors. Keeps everything the scan found;
    removes duplicates (keeps the first); fills gaps from SLOTS_FALLBACK
    order using only colors not already present."""
    seen = []
    for i in range(4):
        c = slots[i]
        if c in ("???", None) or c in seen:
            slots[i] = "???"
        else:
            seen.append(c)
    pool = [c for c in SLOTS_FALLBACK if c not in seen]
    pool += [c for c in DROP_ORDER if c not in seen and c not in pool]
    for i in range(4):
        if slots[i] == "???":
            slots[i] = pool.pop(0)
    if len(set(slots)) != 4:
        print("ERROR: finalize_slots produced duplicates:", slots)
    print("FINAL slots:", slots)
    return slots

#################################################################
#################################################################
##                                                             ##
##   PART 3 — YOUR MANEUVERS + RUN SEQUENCE                    ##
##                                                             ##
#################################################################
#################################################################

# ============================================================
# YOUR CODE — BEFORE (reach the scan start, blocks on the RIGHT)
# ============================================================
def before_scan():
    #drive.arc(-350, 90)
    drive.straight(350)
    drive.turn(90)
    drive.straight(320)
    drive.turn(-90)
    wait(100)
    drive.straight(150)

# ============================================================
# YOUR CODE — AFTER. Bot is parked at x = 0 on the drop line,
# heading +x (toward slot 3 side), all four objects placed.
# Write your moves here. phase2() above is ready when you want it:
# just call phase2() instead of / before your code.
# ============================================================
def after_drops():

    # pick first two  visitions Blue and black
    drive.straight(-440) # go back
    drive.turn(90) # turn towards visitors
    drive.straight(700) # appraoch thm
    close_both() # capture thm
    # Drop  the visitors
    drive.straight(-400)
    drive.turn(-90)
    drive.straight(1320) # reach to sweep area
    drive.curve(230,-90)
    wait(100)
    drive.straight(-50)
    drive.straight(20)
    grab_open("L") # drop black man
    drive.straight(-500)
    drive.turn(180)
    drive.straight(-100)
    drive.straight(20)
    grab_open("R") # drop blue man

    drive.curve(-250,90)
    # exit from the swiping area

    drive.straight(-2000)

    drive.straight(290)
    drive.turn(90)
    drive.straight(500)
    close_both()
    drive.straight(-100)
    drive.turn(-90)
    drive.straight(600)
    drive.turn(90)
    drive.straight(40)
    grab_open("L")
    drive.straight(-180)
    drive.turn(180)
    drive.straight(500)
    grab_open("R")
    drive.straight(-80)
    grab_close("R")
    drive.straight(50)

    #bring the bot to the correect place to pick big towers
    drive.straight(-370)
    drive.turn(90)
    drive.straight(-1000)

    # tower run after butting the wall
    drive.straight(500)
    drive.turn(-90)

    drive.straight(380)
    drive.turn(-90)
    ######
    drive.settings(straight_speed=100)   # slow down
    drive.straight(420)
    drive.settings(straight_speed=RUN_SPEED)   # restore
    ##########

    close_both()
    drive.straight(-1300)
    drive.turn(-90)
    drive.straight(190)
    drive.turn(-90)
    drive.straight(70)
    grab_open("L")
    drive.straight(-100)
    drive.turn(90)
    drive.straight(200)
    drive.turn(-90)
    drive.straight(100)
    grab_open("R")

# ============================================================
# RUN
# ============================================================
match_timer = StopWatch()
timer.reset()
drive.reset()
open_both()

before_scan()

slots = scan_colors(SCAN_FIRST_BLOCK_MM, SCAN_LENGTH_MM)
slots = finalize_slots(slots)

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

# park at a known point: x = 0 on the drop line, heading +x
goto_x(0)
log("ALL DROPS DONE. parked at x 0, heading {:.1f}. placed: {}".format(
    drive.angle(), sorted(placed)))
print("time so far: {:.1f} s".format(match_timer.time() / 1000))

# phase2()          # parametric phase 2 — enable when ready

after_drops()

drive.stop()
print("time taken: {:.1f} s".format(match_timer.time() / 1000))
