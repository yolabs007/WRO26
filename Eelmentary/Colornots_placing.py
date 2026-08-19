# ============================================================
# WRO ELEMENTARY — PICK + DROP TUNING, ALL THREE PAIRS
# (extended from the WORKING single-pair file — all Rahul's
# edits preserved: START_AXIS_MM 1290, BLACK adv 100, drop_note
# does NOT turn back to line heading)
#
# PLACE THE BOT: on the line, at the spot where the scan used
# to end, facing along the line. Measure axle-to-line-wall and
# put it in START_AXIS_MM below.
#
# TRIPS (pick deep then outer; outer drops automated, second
# drop is YOUR local maneuver, one function per color):
#   1: pick YELLOW, BLACK -> drop BLACK -> second_drop_yellow()
#   2: pick GREEN,  RED   -> drop RED   -> second_drop_green()
#   3: pick WHITE,  BLUE  -> drop BLUE  -> second_drop_white()
# After each local drop the run flow turns back to line
# heading (one explicit turn) before the next pair.
# this code do not scan in the current form its only placing objects fter taking a default position 
# ============================================================

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch

# --- TUNING ---
START_AXIS_MM = 1290   # axle distance from line wall AT
                       # PLACEMENT (measure, edit)
SLOT_MM = [480, 610, 740, 870, 1000, 1130]

SLOT_COLORS = ["YELLOW", "WHITE", "GREEN",   # slot 0,1,2
               "RED", "BLACK", "BLUE"]       # slot 3,4,5
                                             # EDIT per setup

NOTE_SIDE_TURN = 90    # PROVEN on mat — FROZEN. Only Rahul changes this.
DROP_SIDE_TURN = -NOTE_SIDE_TURN   # targets = opposite side

# ============================================================
# ===============  CORRECTION KNOBS — TUNE HERE  =============
# PICKS are corrected PER SLOT (slot positions are fixed on
# the mat; colors move between slots every round).
# DROPS are corrected PER COLOR (each color's target is fixed).
# ============================================================

# along-the-line correction per SLOT, mm (+ = farther from wall)
#                   slot0  slot1  slot2  slot3  slot4  slot5
PICK_AXIS_TRIM  = [    0,     0,     0,     0,     0,     0]

# nose-in approach per SLOT, mm (raise until the grab captures)
PICK_ADV        = [   80,    80,    100,    120,    80,    80]     #towords the notes

# along-the-line correction per COLOR target, mm
DROP_AXIS_TRIM = {"WHITE": 0, "BLUE": 0, "GREEN": 0,
                  "RED": 0, "BLACK": 0, "YELLOW": 0}     # this is about black line after brabbing

# grabber trims per COLOR, deg (default 0). For a pair, set
# BOTH its colors to the same value. + = further closed/opened.
GRAB_CLOSE_TRIM = {"WHITE": 0, "BLUE": 0, "GREEN": -25,
                   "RED": -25, "BLACK": 0, "YELLOW": 0}
GRAB_READY_TRIM = {"WHITE": 30, "BLUE": 30, "GREEN": 10,
                   "RED": 10, "BLACK": 30, "YELLOW": 30}
RUN_SPEED  = 300
PICK_SPEED = 150
DROP_SPEED = 150

# --- DROP TABLE (plain numbers, per color) ---
TARGET_AXIS = {"WHITE": 510, "BLUE": 680, "GREEN": 830,
               "RED": 1000, "BLACK": 1160, "YELLOW": 1330}
DROP_ADV = {          # mm nosed in after the 90 turn, per color.
    "WHITE":  0,     # 20 everywhere = TEST MODE.
    "BLUE":   20,     # Perp depths from map for later tuning:
    "GREEN":  0,     # W110 B200 G255 R300 K330 Y270
    "RED":    150,     # (measured line edge -> box center).
    "BLACK":  180,
    "YELLOW": 0,
}

# --- GRABBER POSITIONS (proven names) ---
FRONT_OPEN   = 0
FRONT_READY  = 30
FRONT_CLOSED = 175

BACK_RAISED  = 0
BACK_PC      = 30
BACK_TRAP    = 61
BACK_RELEASE = 30

# --- PORTS (FINAL) ---
LEFT_MOTOR_PORT     = Port.A
BACK_GRAB_PORT      = Port.B
COLOR_SIDE_PORT     = Port.C
COLOR_DOWN_PORT     = Port.D
RIGHT_MOTOR_PORT    = Port.E
FRONT_GRAB_PORT     = Port.F
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
front_grab = Motor(FRONT_GRAB_PORT)     # bare — never add direction
back_grab  = Motor(BACK_GRAB_PORT)      # bare — never add direction
floor_sensor = ColorSensor(COLOR_SIDE_PORT)

drive = DriveBase(left_motor, right_motor, WHEEL_DIAMETER, AXLE_TRACK)
drive.use_gyro(True)
drive.settings(straight_speed=200)

timer = StopWatch()

def log(msg):
    print("[{:>5.1f}s] {}".format(timer.time() / 1000, msg))

# ------------------------------------------------------------
# GRABBERS — run_target absolute, stall-protected (proven)
# ------------------------------------------------------------
GRAB_SPEED = 300

def _grab_goto(motor, target_deg, name):
    motor.run_target(GRAB_SPEED, target_deg, then=Stop.HOLD, wait=False)
    while not motor.done():
        if motor.stalled():
            motor.hold()
            log("  {} stalled — holding at {}".format(name, motor.angle()))
            break
        wait(10)

def back_grab_goto(target_deg):
    _grab_goto(back_grab, target_deg, "back grabber")

def front_grab_goto(target_deg):
    _grab_goto(front_grab, target_deg, "front grabber")

# ------------------------------------------------------------
# AXIS FRAME — axle position along the line
# ------------------------------------------------------------
axis_mm = float(START_AXIS_MM)

def goto_axis(target_mm):
    global axis_mm
    delta = target_mm - axis_mm
    log("  goto {:.0f} -> {:.0f} (drive {:+.0f})".format(
        axis_mm, target_mm, delta))
    if abs(delta) > 2:
        drive.settings(straight_speed=RUN_SPEED)
        drive.straight(delta)
        axis_mm = target_mm

# ------------------------------------------------------------
# PICK — plain 90 spot turns only
# ------------------------------------------------------------
slot_of = {c: i for i, c in enumerate(SLOT_COLORS)}

def pick_note(color):
    s = slot_of[color]
    log("PICK {} (slot {}, {}{}{} mm, adv {})".format(
        color, s, SLOT_MM[s],
        "+" if PICK_AXIS_TRIM[s] >= 0 else "", PICK_AXIS_TRIM[s],
        PICK_ADV[s]))
    goto_axis(SLOT_MM[s] + PICK_AXIS_TRIM[s])
    hub.speaker.beep(800, 200)
    drive.turn(NOTE_SIDE_TURN)          # face the note
    front_grab_goto(FRONT_READY + GRAB_READY_TRIM[color])
    drive.settings(straight_speed=PICK_SPEED)
    drive.straight(PICK_ADV[s])         # nose in (per slot)
    front_grab_goto(FRONT_CLOSED + GRAB_CLOSE_TRIM[color])       # grab
    drive.straight(-PICK_ADV[s])        # nose out
    drive.turn(-NOTE_SIDE_TURN)         # back to line heading
    log("  {} done. heading {:.1f}".format(color, drive.angle()))

# ------------------------------------------------------------
# DROP — plain 90 turn to the OTHER side, nose in DROP_ADV,
# release, nose out. NOTE: does NOT turn back to line heading
# (Rahul's edit) — bot ends facing the drop side. The local
# second-drop maneuver starts from that pose.
# ------------------------------------------------------------
def drop_note(color, release_pos, resecure):
    log("DROP {} (axis {}, adv {})".format(
        color, TARGET_AXIS[color], DROP_ADV[color]))
    goto_axis(TARGET_AXIS[color] + DROP_AXIS_TRIM[color])
    hub.speaker.beep(500, 200)
    drive.turn(DROP_SIDE_TURN)          # face the target side
    drive.settings(straight_speed=DROP_SPEED)
    drive.straight(DROP_ADV[color])     # nose in
    front_grab_goto(release_pos + GRAB_READY_TRIM[color])        # release
    wait(300)
    drive.straight(-40)                 # FIRST NOTE OUT 
    front_grab_goto(FRONT_CLOSED + GRAB_CLOSE_TRIM[color])       # GRAB 2nd ONE BACK
    drive.straight(-DROP_ADV[color]+40)    # nose out
    if resecure:
        front_grab_goto(FRONT_CLOSED + GRAB_CLOSE_TRIM[color])   # re-hold inner note
    #drive.turn(-DROP_SIDE_TURN)        # back to line heading
    log("  {} dropped. heading {:.1f}".format(color, drive.angle()))

# ------------------------------------------------------------
# SECOND DROPS — YOUR LOCAL MANEUVERS, one per color.
# Start pose for each: at the outer drop's line position,
# FACING THE DROP SIDE (drop_note leaves the bot like that).
# CONTRACT: release the note inside your lines and RETURN to
# the SAME spot, SAME pose. The run flow then turns back to
# line heading before the next pair.
# ------------------------------------------------------------
def second_drop_yellow():
    log("SECOND DROP (local): YELLOW")
    # === YOUR LINES (from your working file) =======
    drive.turn(28)
    drive.straight(150)
    #grabber code line write here
    front_grab_goto(FRONT_OPEN)
    drive.straight(-100)
    drive.turn(-28)
    drive.straight(-40)
    # ===============================================
    log("  back at BLACK-drop spot. heading {:.1f}".format(drive.angle()))

def second_drop_green():
    log("SECOND DROP (local): GREEN")
    # === YOUR LINES HERE ===========================
    drive.turn(-30)
    drive.straight(110)
    #grabber code line write here
    front_grab_goto(FRONT_OPEN)
    drive.straight(-100)
    drive.turn(30)
    drive.straight(-40)

    # ===============================================
    log("  back at RED-drop spot. heading {:.1f}".format(drive.angle()))

def second_drop_white():
    log("SECOND DROP (local): WHITE")
    # === YOUR LINES HERE ===========================
    drive.straight(-40)
    drive.turn(-50)
    drive.straight(40)
    #grabber code line write here
    front_grab_goto(FRONT_OPEN-20)
    drive.straight(-40)
    #drive.turn(50)
    # ===============================================
    log("  back at BLUE-drop spot. heading {:.1f}".format(drive.angle()))

# ------------------------------------------------------------
# RUN — no scan. Placement = scan-end spot, facing along line.
# ------------------------------------------------------------
timer.reset()
drive.reset()        # heading 0 = placement heading

log("start: axis {:.0f}".format(axis_mm))

# ---- TRIP 1: YELLOW (deep) + BLACK (outer) ----
pick_note("YELLOW")
pick_note("BLACK")
drop_note("BLACK", FRONT_READY, True)
second_drop_yellow()
drive.turn(-DROP_SIDE_TURN)     # back to line heading for next pair
log("trip 1 done. axis {:.0f}, heading {:.1f}".format(axis_mm, drive.angle()))

# ---- TRIP 2: GREEN (deep) + RED (outer) ----
pick_note("GREEN")
pick_note("RED")
drop_note("RED", FRONT_READY, True)
second_drop_green()
drive.turn(-DROP_SIDE_TURN)     # back to line heading for next pair
log("trip 2 done. axis {:.0f}, heading {:.1f}".format(axis_mm, drive.angle()))

# ---- TRIP 3: WHITE (deep) + BLUE (outer) ----
pick_note("WHITE")
pick_note("BLUE")
drop_note("BLUE", FRONT_READY, True)
second_drop_white()
log("trip 3 done. axis {:.0f}, heading {:.1f}".format(axis_mm, drive.angle()))

drive.stop()
log("ALL PAIRS COMPLETE — axis {:.0f}".format(axis_mm))
