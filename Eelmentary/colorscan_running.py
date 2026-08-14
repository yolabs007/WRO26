# ============================================================
# SCAN v8.1 — yesterday's code + double-white resolution
# NEW: every detection records its peak V. If TWO whites are
# found, the stronger is WHITE, the weaker becomes BLACK.
# If black is invisible (one white only), elimination logic
# works exactly as yesterday. Nothing else changed.
# ============================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# --- PORT CONFIG ---
LEFT_MOTOR_PORT    = Port.A
RIGHT_MOTOR_PORT   = Port.E
COLOR_SENSOR_PORT  = Port.C
LEFT_MOTOR_REVERSED  = True
RIGHT_MOTOR_REVERSED = False

# --- DRIVE TUNABLES ---
WHEEL_DIAMETER = 56
AXLE_TRACK     = 165
SCAN_SPEED     = 100
SCAN_LENGTH    = 1050
MIN_GAP        = 40

APPROACH_SPEED = 400
APPROACH_MM    = 440
APPROACH_TURN  = -90

# --- CLASSIFIER (yesterday's thresholds, unchanged) ---
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

# --- SCAN PASS — now records peak V per detection ---
def scan_row():
    log("SCAN: start")
    found = []                    # (entry_pos, color, peak_v)
    last, hits, entry_pos = None, 0, 0
    peak_v = 0
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
                peak_v = max(peak_v, hsv.v)
            elif c is not None:
                last, hits, entry_pos = c, 1, d
                peak_v = hsv.v
            else:
                last, hits = None, 0

            if hits >= 2:
                found.append((entry_pos, c, peak_v))
                log("  {} at ~{} mm (peak V {})".format(c, entry_pos, peak_v))
                rearm_at = d + MIN_GAP
                last, hits = None, 0
        wait(20)

    drive.stop()
    return found

# --- MERGE split bases (tracks peak V across merge) ---
def merge(detections):
    merged = []
    for pos, c, pv in detections:
        if merged and merged[-1][1] == c and pos - merged[-1][0] < 70:
            p0, c0, pv0 = merged[-1]
            merged[-1] = (p0, c0, max(pv0, pv))    # split base
            continue
        merged.append((pos, c, pv))
    return merged

# --- NEW: double-white resolution ---
def resolve_double_white(notes):
    """If two whites: stronger stays WHITE, weaker becomes BLACK."""
    whites = [n for n in notes if n[1] == "WHITE"]
    if len(whites) == 2:
        weaker = min(whites, key=lambda n: n[2])
        log("Double white: {} mm (V {}) vs {} mm (V {}) — weaker is BLACK"
            .format(whites[0][0], whites[0][2], whites[1][0], whites[1][2]))
        notes = [(p, "BLACK", pv) if (p == weaker[0] and c == "WHITE")
                 else (p, c, pv) for p, c, pv in notes]
    elif len(whites) > 2:
        print("WARNING: {} whites detected — check thresholds!".format(len(whites)))
    return notes

# --- SLOT LOGIC: GREEN(2)/RED(3) anchored ---
def build_map(detections):
    notes = resolve_double_white(merge(detections))
    colors = [c for _, c, _ in notes]
    pos = {c: p for p, c, _ in notes}
    slot_map = [None] * 6

    if "GREEN" not in pos or "RED" not in pos:
        print("ERROR: GREEN/RED anchor missing — got", colors)
        return slot_map

    slot_map[2] = "GREEN"
    slot_map[3] = "RED"
    spacing = pos["RED"] - pos["GREEN"]
    log("spacing reference (green->red) = {} mm".format(spacing))

    gi = colors.index("GREEN")
    ri = colors.index("RED")
    before = [(pos[c], c) for c in colors[:gi]]
    after  = [(pos[c], c) for c in colors[ri + 1:]]
    if ri != gi + 1:
        print("WARNING: unexpected color between GREEN and RED:",
              colors[gi + 1:ri])

    # ---- left side: slots 0, 1 ----
    if len(before) == 2:
        slot_map[0], slot_map[1] = before[0][1], before[1][1]
    elif len(before) == 1:
        p, c = before[0]
        gap = pos["GREEN"] - p
        if gap < 1.5 * spacing:
            slot_map[1] = c
            slot_map[0] = "BLACK"
            log("BLACK at slot 0 ({}->GREEN gap {} ~ 1x)".format(c, gap))
        else:
            slot_map[0] = c
            slot_map[1] = "BLACK"
            log("BLACK at slot 1 ({}->GREEN gap {} ~ 2x)".format(c, gap))
    else:
        print("WARNING: {} colors left of GREEN".format(len(before)))

    # ---- right side: slots 4, 5 ----
    if len(after) == 2:
        slot_map[4], slot_map[5] = after[0][1], after[1][1]
    elif len(after) == 1:
        p, c = after[0]
        gap = p - pos["RED"]
        if gap < 1.5 * spacing:
            slot_map[4] = c
            slot_map[5] = "BLACK"
            log("BLACK at slot 5 (RED->{} gap {} ~ 1x)".format(c, gap))
        else:
            slot_map[5] = c
            slot_map[4] = "BLACK"
            log("BLACK at slot 4 (RED->{} gap {} ~ 2x)".format(c, gap))
    else:
        print("WARNING: {} colors right of RED".format(len(after)))

    if slot_map.count("BLACK") != 1:
        print("WARNING: BLACK placed {} times".format(slot_map.count("BLACK")))

    return slot_map

# --- RUN ---
timer.reset()

# approach: fast straight + left turn onto the row
drive.settings(straight_speed=APPROACH_SPEED)
drive.straight(APPROACH_MM)
drive.turn(APPROACH_TURN)

detections = scan_row()
slot_map = build_map(detections)

print("")
print("========== FINAL SLOT MAP ==========")
for i in range(6):
    tag = " (fixed)" if i in (2, 3) else ""
    print("  Slot {} : {}{}".format(i, slot_map[i] if slot_map[i] else "???", tag))
print("====================================")
