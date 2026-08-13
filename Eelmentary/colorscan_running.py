# ============================================================
# SCAN v8 — GREEN/RED anchored slot logic, self-calibrating
# Fixed anchors: GREEN = slot 2, RED = slot 3.
# Spacing measured live from green->red distance.
# BLACK found by gap analysis on whichever side has 1 color.
# ============================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# --- PORT CONFIG ---
LEFT_MOTOR_PORT    = Port.F
RIGHT_MOTOR_PORT   = Port.B
COLOR_SENSOR_PORT  = Port.D
LEFT_MOTOR_REVERSED  = True
RIGHT_MOTOR_REVERSED = False

# --- DRIVE TUNABLES ---
WHEEL_DIAMETER = 56
AXLE_TRACK     = 165
SCAN_SPEED     = 100
SCAN_LENGTH    = 1050
MIN_GAP        = 40

# --- CLASSIFIER (locked) ---
SAT_MIN, V_MIN = 55, 2
WHITE_S_MAX, WHITE_V_MIN = 30, 5

def classify(h, s, v):
    if s >= SAT_MIN and v >= V_MIN:
        if h >= 330 or h <= 15:
            return "RED"
        if 25 <= h <= 55:
            return "YELLOW"
        if 140 <= h <= 185:
            return "GREEN"
        if 200 <= h <= 235:
            return "BLUE"
    if s <= WHITE_S_MAX and v >= WHITE_V_MIN:
        return "WHITE"
    return None

# --- SETUP ---
hub = PrimeHub()
left_motor = Motor(LEFT_MOTOR_PORT,
    Direction.COUNTERCLOCKWISE if LEFT_MOTOR_REVERSED else Direction.CLOCKWISE)
right_motor = Motor(RIGHT_MOTOR_PORT,
    Direction.COUNTERCLOCKWISE if RIGHT_MOTOR_REVERSED else Direction.CLOCKWISE)
floor_sensor = ColorSensor(COLOR_SENSOR_PORT)

drive = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
drive.use_gyro(True)

timer = StopWatch()

def log(msg):
    print("[{:>5.1f}s] {}".format(timer.time() / 1000, msg))

# --- SCAN PASS (unchanged — detection works well) ---
def scan_row():
    log("SCAN: start")
    found = []
    last, hits, entry_pos = None, 0, 0
    rearm_at = 0

    drive.reset()
    drive.drive(SCAN_SPEED, 0)

    while drive.distance() < SCAN_LENGTH:
        d = drive.distance()
        if d >= rearm_at:
            hsv = floor_sensor.hsv()
            c = classify(hsv.h, hsv.s, hsv.v)
            if c is not None and c == last:
                hits += 1
            elif c is not None:
                last, hits, entry_pos = c, 1, d
            else:
                last, hits = None, 0

            if hits >= 2:
                found.append((entry_pos, c))
                log("  {} at ~{} mm".format(c, entry_pos))
                rearm_at = d + MIN_GAP
                last, hits = None, 0
        wait(20)

    drive.stop()
    return found

# --- MERGE consecutive same-color detections into one note ---
def merge(detections):
    merged = []
    for pos, c in detections:
        if merged and merged[-1][1] == c:
            continue                     # split base -> keep first pos
        merged.append((pos, c))
    return merged

# --- SLOT LOGIC: anchored on GREEN(2) and RED(3) ---
def build_map(detections):
    notes = merge(detections)
    colors = [c for _, c in notes]
    pos = {c: p for p, c in notes}       # first-entry position per color
    slot_map = [None] * 6

    # anchors must exist
    if "GREEN" not in pos or "RED" not in pos:
        print("ERROR: GREEN/RED anchor missing — got", colors)
        return slot_map

    slot_map[2] = "GREEN"
    slot_map[3] = "RED"

    # live spacing reference: green->red is exactly 1 slot
    spacing = pos["RED"] - pos["GREEN"]
    log("spacing reference (green->red) = {} mm".format(spacing))

    gi = colors.index("GREEN")
    ri = colors.index("RED")
    before = [(pos[c], c) for c in colors[:gi]]      # left of green
    after  = [(pos[c], c) for c in colors[ri + 1:]]  # right of red
    if ri != gi + 1:
        print("WARNING: unexpected color between GREEN and RED:",
              colors[gi + 1:ri])

    # ---- left side: slots 0, 1 ----
    if len(before) == 2:
        slot_map[0], slot_map[1] = before[0][1], before[1][1]
    elif len(before) == 1:
        p, c = before[0]
        gap = pos["GREEN"] - p
        if gap < 1.5 * spacing:          # adjacent to green -> slot 1
            slot_map[1] = c
            slot_map[0] = "BLACK"
            log("BLACK at slot 0 ({}->GREEN gap {} ~ 1x spacing)".format(c, gap))
        else:                            # far from green -> slot 0
            slot_map[0] = c
            slot_map[1] = "BLACK"
            log("BLACK at slot 1 ({}->GREEN gap {} ~ 2x spacing)".format(c, gap))
    else:
        print("WARNING: {} colors left of GREEN — expected 1 or 2".format(len(before)))

    # ---- right side: slots 4, 5 ----
    if len(after) == 2:
        slot_map[4], slot_map[5] = after[0][1], after[1][1]
    elif len(after) == 1:
        p, c = after[0]
        gap = p - pos["RED"]
        if gap < 1.5 * spacing:          # adjacent to red -> slot 4
            slot_map[4] = c
            slot_map[5] = "BLACK"
            log("BLACK at slot 5 (RED->{} gap {} ~ 1x spacing)".format(c, gap))
        else:                            # far from red -> slot 5
            slot_map[5] = c
            slot_map[4] = "BLACK"
            log("BLACK at slot 4 (RED->{} gap {} ~ 2x spacing)".format(c, gap))
    else:
        print("WARNING: {} colors right of RED — expected 1 or 2".format(len(after)))

    # sanity: exactly one BLACK
    if slot_map.count("BLACK") != 1:
        print("WARNING: BLACK placed {} times — one side missed a color"
              .format(slot_map.count("BLACK")))

    return slot_map

# 1. Approach: fast straight, then turn onto the note row
drive.settings(straight_speed= 400)
drive.straight(440)
drive.turn(-90)




# --- RUN ---
timer.reset()
detections = scan_row()
slot_map = build_map(detections)

print("")
print("========== FINAL SLOT MAP ==========")
for i in range(6):
    tag = " (fixed)" if i in (2, 3) else ""
    print("  Slot {} : {}{}".format(i, slot_map[i] if slot_map[i] else "???", tag))
print("====================================")
