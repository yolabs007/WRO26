// ============================================================
// WRO FE 2026 — OBSTACLE — STEP 4 v15 : FULL RUN (3 laps, no parking)
// v15 (23 Sep): v14 + POST-PASS RECOVERY ("B"). v13 logs: after a pass the
//   open-challenge wall term sat at its +-12 clamp and the car came back only
//   25-30 cm in 900 mm after greens. Now, from RECOVER until POST_PASS_MM
//   later, the wall term aims at POST_PASS_FRACTION (80%) of the way back from
//   the offset measured by the side ToFs at the start, with clamp
//   POST_PASS_MAX_CORR. Ends at the target, at POST_PASS_MM, at a turn or at
//   a new pillar; then the normal open-challenge wall term. Same wall rules
//   (both walls -> centre, one wall -> WALL_TARGET_CM), ToF only, no carLat.
//   Log: column wc (wall-term servo correction) appended at the end.
// v14 (23 Sep, from v13 logs 19:49-20:00): everything below the v13 spec is
// unchanged except these additions (search "v14"):
//   1. SHAPE/SIZE GATE on every HuskyLens block, in every state:
//      a pillar is 10 cm tall, so its blob height grows with closeness
//      (h ~= 0.47*bottom + 30 in the v13 logs); the orange floor tape (read
//      as ID 2) stays flat. Unclipped block must have h >= 55% of that line
//      and h/w >= 1.0 (red) / 0.75 (green). Blocks touching the frame edge
//      or bottom are "clipped": tracked, but never used to re-plan.
//   2. TRACKING CONTINUITY in COMMIT/HOLD/RECOVER: x may not jump more than
//      TRACK_MAX_DX (+ widening with time), bottom may not jump back up
//      (farther) more than TRACK_MAX_BACK.
//   3. PASS POINT: re-plans may pull the encoder pass point closer freely,
//      but never push it more than PASS_PUSH_MAX_MM beyond the first plan.
//      COMMIT_CAP_MM of driven path in COMMIT -> HOLD regardless.
//   4. STALL UNSTICK (outside turns): forward command but encoder frozen
//      STALL_MS -> reverse UNSTICK_MM on lane-heading PD, then NONE.
//   5. CLOSE WRONG-SIDE EXIT: first pillar of a section seen within
//      BACKUP_WINDOW_MM after a corner, bottom >= BACKUP_BOTTOM and on the
//      side the car must cross (red right / green left) -> reverse straight
//      (lane-heading PD) until it is seen at bottom <= BACKUP_OK_BOTTOM or
//      BACKUP_MAX_MM / stall / timeout, then plan. Once per section.
//   Log: two columns appended at the end: meas (M measurable / C clipped /
//   - none) and rej (blocks rejected by the gate this frame).
//   Not done here (separate task): orange taught as HuskyLens ID 3.
// ------------------------------------------------------------
// v13 spec (agreed 23 Sep):
//   straights = open challenge: heading PD + wall term (+-12). NO centre-
//   after-turn, NO scan-back, NO lane return. After HOLD: PD back to lane.
//   corners = open challenge trigger + split 45/45 turn with reverse.
//   MIN_LEG_MM 1500 (first corner exempt). ENTRY_BOTTOM_MAX 235.
//   IMU: hdg,tgt logged; LOST watchdog: |hdg-lane| > LOST_DEG outside a
//   turn -> abandon phase, PD to lane.
// (= STEP 3 v3 + open-challenge v3b corner logic)
//   - corner decision only in pillar state NONE (pillar has priority)
//   - a finished corner = new lane: target_heading +/- 90, carLat/fwd reset
//   - pillar seen during the end-of-turn blend (heading within
//     ACQ_STRAIGHT_DEG of the new lane) ends the turn and is acquired
//   - run ends after TURNS_TO_RUN corners + FINISH_MM
// ---- STEP 3 notes:
//   - a new pillar can be acquired during RECOVER (any colour) and during
//     HOLD (same colour only); it re-enters COMMIT with a fresh plan from
//     the car's current lateral position and yaw
//   - gain model takes SIGNED yaw: after a red (yawed right) a green plan
//     first pays for cancelling that yaw
//   - lane test: pillars ~50 cm apart: R-R, G-G, R-G, G-R, each 2x
//
// Glitch's pillar logic on HuskyLens:
//   NONE    no pillar -> heading PD (wall term optional), OBST_SPEED
//   FAR     pillar seen, still small -> PD on pillar x, target near centre
//           (align with it)
//   COMMIT  pillar close -> PD on pillar x, target at the frame edge
//           OPPOSITE the pass side. RED -> pillar to LEFT edge, car passes
//           RIGHT.  GREEN -> pillar to RIGHT edge, car passes LEFT.
//   HOLD    pillar left the frame -> keep last servo angle for a while
//           (rear wheel clears the pillar)
//   RECOVER back on lane heading, ignore new pillars for a while
//
// v2 (after STEP 1 logs):
//   - ENTRY filter: a block only starts a pillar if it is pillar-shaped
//     (w <= PILLAR_W_MAX and h/w >= PILLAR_ASPECT_MIN). The red floor
//     line (w 150-319, h/w 0.2-0.5) never qualifies.
//   - Once in COMMIT no filter: track the block nearest the LAST x,
//     so the line cannot steal 'nearest' mid-pass (seen in run 1).
//     (v14: superseded — the gate + continuity now apply in COMMIT too)
//   - Trigger fixed to bottom (COMMIT_BOTTOM); h and w kept for logging.
//
// v3 — SWERVE_MODE:
//   0  camera closed-loop: COMMIT/HOLD steer by pillar-x PD to the frame
//      edge (0.15 / 0.85). Original Glitch form.
//   1  HYBRID (default): camera decides WHEN and WHICH SIDE; from COMMIT on
//      the car steers by the IMU heading PD to  lane_heading +/- SWERVE_DEG
//      (red -> +, right; green -> -, left). Pillar leaving the frame or the
//      floor line appearing no longer matters. HOLD keeps that heading,
//      RECOVER PDs back to lane_heading. Same PD and sign convention as the
//      open-challenge corners (+error = steer right), so no PILLAR_SIGN
//      question in this mode.
//
// v6 — FAR is a LANE SHIFT, not a pixel PD (mode 1):
//      pillar x in the frame is bearing, not lateral offset: steering
//      toward it 'centres' it by yawing the car, and the gain vanishes
//      once the heading straightens. So FAR now drives an open-loop
//      S-shift by encoder distance toward the pass side (red -> right,
//      green -> left): heading lane +/- FAR_SHIFT_DEG for FAR_SHIFT_MM,
//      then lane heading. Skipped when the pillar is already clearly on
//      the far side of centre. COMMIT can fire at any point during it.
//
// MOTOR_ON 0 = car on blocks, servo only (verify PILLAR_SIGN / KP first).
// MOTOR_ON 1 = drives. 1 m lane, one pillar mid-lane, each colour, each
//              lane side. Pass when: correct side, no touch, back on
//              heading before lane end, 3/3 per colour.
//
// Log line (CSV):
//   t_ms,mm,state,F,L,R,hdg,tgt,h_err,servo,col,x,x_frac,y,bottom,h,w,blocks,carLat,turn,meas,rej,wc
//
// HuskyLens: main I2C bus 0x32, 100 kHz. ID 1 = GREEN, ID 2 = RED.
// Hardware side copied from fe_open_challenge_v3b.ino as-is.
// ============================================================

#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_NeoPixel.h>
#include <BluetoothSerial.h>
#include "HUSKYLENS.h"

// ---------- PINS (v3b) ----------
#define IN1_PIN        26
#define IN2_PIN        27
#define SERVO_PIN      13
#define NEOPIXEL_PIN   14
#define START_BTN_PIN  25
#define ENCODER_A_PIN  18
#define ENCODER_B_PIN  19
#define TCAADDR        0x70
#define TOF_FRONT_CH   0
#define TOF_RIGHT_CH   1
#define TOF_LEFT_CH    2
#define BNO_CH         3

// ############################################################
// ##   TUNING                                               ##
// ############################################################

// ---------- Run ----------
#define MOTOR_ON              1   // 0 = on blocks, servo only; 1 = drive
#define SWERVE_MODE           2   // 0 = camera PD (Glitch), 1 = mini-corner fixed angle,
                                  // 2 = PLANNED from geometry (v11), 3 = path table
// ---- mode 3: path table by colour x position. YOU set every number.
//      position from x at first sight: FL < 0.20 < L < 0.40 < C < 0.60 < R < 0.80 < FR
//      phase 1: arc to lane+A1, hold D1 mm   (lane correction)
//      phase 2: arc to lane+A2, hold D2 mm   (the pass)
//      then full arc back to lane heading, then RECOVER_MM on lane.
//      Angles SIGNED, + = right. Red passes right (+), green left (-).
#define POS_FL_MAX         0.20
#define POS_L_MAX          0.40
#define POS_C_MAX          0.60
#define POS_R_MAX          0.80
//                               FL      L       C       R       FR     (pillar seen at)
const int RED_A1[5]   = {        5,     10,     20,     30,     35 };   // deg, phase 1
const int RED_D1[5]   = {      150,    200,    250,    300,    350 };   // mm
const int RED_A2[5]   = {       15,     20,     30,     30,     35 };   // deg, phase 2
const int RED_D2[5]   = {      250,    300,    300,    350,    350 };   // mm
//                               FL      L       C       R       FR
const int GREEN_A1[5] = {      -35,    -30,    -20,    -10,     -5 };
const int GREEN_D1[5] = {      350,    300,    250,    200,    150 };
const int GREEN_A2[5] = {      -35,    -30,    -30,    -20,    -15 };
const int GREEN_D2[5] = {      350,    350,    300,    300,    250 };
// ---- mode 2: geometry planner (re-solved every frame while pillar seen)
#define CLEAR_MM            150   // car centre must be this far from pillar centre
// ---- mode 2 tuning knobs (all multiply/limit what the planner asks for)
#define NEED_SCALE          1.0   // x the needed lateral offset. 0.8 = 20% less swerve
#define THETA_SCALE         1.0   // x the solved angle (applied after NEED_SCALE)
#define REPLAN_MODE           1   // 0 = plan once at first sight, freeze
                                  // 1 = re-plan every frame while seen (v11 behaviour)
                                  // 2 = re-plan but theta may only DECREASE
                                  //     (last run: re-plans pushed 18 -> 35 during the turn)
#define TURN_R_MM           400   // full-lock turn radius (from v8 recover: 6.5 deg / 47 mm)
#define STEER_LAG_MM         60   // distance driven before the arc really starts
#define THETA_MIN            10   // solved angle limits, deg
#define THETA_MAX            35
#define ZERO_THETA_IF_CLEAR   0   // v14 option, OFF: 1 = when the plan says the car is
                                  // already clear (need <= 0) hold the lane heading (0 deg)
                                  // instead of the THETA_MIN 10 deg swerve. All 6 such
                                  // plans in the v13 logs passed with 10 deg, so left off.
#define CAM_HFOV_DEG         55   // HuskyLens horizontal FOV. CALIBRATE: pillar 20 cm
                                  // left of centre at 60 cm -> x_frac should be ~0.31
#define CAM_K              4352   // distance_cm = CAM_K / (bottom - CAM_H0)
#define CAM_H0               15   //   fit from v8 log: bottom 79 -> 68 cm, 143 -> 34 cm
// ---- mode 1 = mini-corner: at first sight turn at FULL ARC (like a corner)
//      to lane +/- SWERVE_DEG, hold it past the pillar, full arc back.
#define SWERVE_DEG           20   // heading offset from lane. Path calc: 19 cm at pillar, 28 cm at end (R=0.40 m)
#define SWERVE_DEG_NEAR      15   // used instead if the pillar is already on the
                                  // far side (red x <= FAR_SKIP_X_RED etc.)
#define SWERVE_BLEND_DEG     10   // within this of target -> heading PD, else fixed arc
#define SERVO_SHIFT_LEFT     70   // fixed arc (from open v3b)
#define SERVO_SHIFT_RIGHT   130
#define OBST_SPEED           80   // no pillar / recover (was 110)
#define PILLAR_SPEED         80   // while handling a pillar (was 95)
#define STOP_FRONT_CM      15.0   // crash guard only (corner triggers at 85)
// ---------- Corners (open v3b) ----------
#define TURN_SPEED           80
#define REVERSE_SPEED       160
#define TURN_THRESHOLD_CM  85.0   // front wall closer than this -> consider turning
                                  // v14 CHECK: the pasted v13 said 105, but the 23 Sep logs only
                                  // replay correctly with 82-85 (run D: F=94 and F=86 with the right
                                  // side open did NOT turn; F=77 did). 85 = what the car ran.
#define SIDE_GAP_CM        70.0   // a side farther than this is "open" (70, not 50:
                                  // an off-centre car after a pass must not read a lane wall as open)
#define BLIND_LOCK_CM      40.0
#define TURN_COOLDOWN_MS   1000
#define MIN_LEG_MM         1500   // no new corner until this far since the last one (first corner exempt)
#define TURN_EXIT_DEG       8.0
#define TURN_TIMEOUT_MS    6500
#define TURN_BLEND_DEG       30   // within this of target -> heading PD instead of fixed arc
#define TURN_ABORT_FRONT_CM 22.0  // during a turn, past the half-way point: front closer than
                                  // this = pillar (or wall) in the arc -> end the turn here,
                                  // count it, and back out via SCANBACK (reverse on heading
                                  // hold toward the new lane heading). Replaces the old 12/20 cm
                                  // crash-reverse dither.
#define TURN_ABORT_AFTER_DEG 45   // only arm the abort once this much of the 90 is done
#define TURNS_TO_RUN         12   // 12 = three laps
#define LOST_DEG             45   // outside a turn: |heading - lane| beyond this = LOST ->
                                  // abandon pillar/recover phase, heading PD back to lane
#define STALL_MS            500   // split turn fwd half: encoder frozen this long -> go to reverse half
                                  // v14: also the stall-unstick time outside turns
// ---- SPLIT TURN: forward on lock to TURN_FWD_DEG, stop, reverse on the
//      opposite lock until the gyro reads the full 90, stop. The car pivots
//      near where the turn began: centred in the new lane, looking down it.
#define SPLIT_TURN            1   // 1 = split turn (below); 0 = plain forward arc
#define TURN_FWD_DEG         45   // degrees turned going forward before reversing
#define SPLIT_REV_SPEED      70   // reverse speed (PWM)
#define SPLIT_REV_BLEND_DEG  10   // within this of 90: reverse-PD instead of fixed lock
#define REV_MIN_MM          300   // reverse at least this far (from where reversing began),
                                  // straight on heading hold once the 90 is reached
#define TURN_THROUGH_DEG      0   // (SPLIT_TURN 0 only) exit heading overshoot toward inner side
#define SCAN_BACK             0   // (SPLIT_TURN 0 only) reverse-and-look after the turn
#define SCAN_BACK_MM        300
#define SCAN_BACK_SPEED      90
#define SCAN_BACK_SHIFT_DEG  20   // while reversing, nose points this far AWAY from centre
                                  // so the rear moves TOWARD centre; straightens at the end
#define CENTRE_AFTER_TURN     0   // v13: OFF (was a full-lock +-25 swing; removed)
#define LANE_ANCHOR_SEE_CM 80.0   // both side walls closer than this -> carLat re-anchored to lane centre
#define LANE_ANCHOR_GAIN   0.05   // blend per loop while in NONE (0 = off)
#define FINISH_MM          1000   // after the last corner: stop once this far into the
                                  // start section, with pillars still handled on the way

#define STOP_TIMEOUT_MS  200000
#define USE_WALL              1   // 1 = open-challenge wall centring in NONE only (clamp WALL_MAX_CORR)

// ---------- Camera ----------
#define FRAME_W             320
#define FRAME_H             240
#define GREEN_ID              1
#define RED_ID                2
#define MIN_PILLAR_H         12   // noise gate only

// ---------- Pillar entry filter (NONE/FAR only) ----------
#define PILLAR_W_MAX        130   // pillar never wider than ~115; line >= 150
#define PILLAR_ASPECT_MIN   1.0   // whole pillar: red ~2.0, green 1.3-1.8; line 0.2-0.5
#define ENTRY_BOTTOM_MAX    235   // acquire only while still FAR. Every real pillar
                                  // first appeared at bottom 67-121; a block that
                                  // shows up already at 215 (seen near the wall
                                  // after a pass) is not a pillar approach.

// ---------- v14: pillar SHAPE/SIZE gate (every state) ----------
#define GATE_ON               1   // 0 = v13 behaviour (entry filter only, no gate in COMMIT)
#define PIL_H_SLOPE        0.47   // real pillar height line: h ~= SLOPE*bottom + OFS (px)
#define PIL_H_OFS          30.0   //   (80th percentile of red pillar blobs, v13 logs)
#define PIL_H_MIN_FRAC     0.55   // unclipped RED block must reach this fraction of that height
                                  // (orange tape: <= 0.4; real pillars reach ~1.0 in 2-6 frames)
#define PIL_H_MIN_FRAC_G   0.45   // GREEN: looser. The tape only ever comes back as ID 2 (red);
                                  // far greens are shorter boxes (h 40-49 at bottom ~80)
#define RED_ASPECT_MIN     1.00   // h/w of an unclipped block in tracking (red)
#define GREEN_ASPECT_MIN   0.75   // green boxes run wider (h 44 w 47 seen on real greens)
#define CLIP_EDGE_PX          4   // box edge within this of the frame side = clipped at the side
#define CLIP_BOTTOM_PX      236   // bottom >= this = clipped at the frame bottom
#define TRACK_MAX_DX         60   // tracking: x jump allowed per accepted frame (px)
#define TRACK_DX_PER_100MS   20   //   + this per 100 ms without an accepted frame
#define TRACK_MAX_BACK       25   // tracking: bottom may not move UP (farther) more than this

// ---------- v14: pass point / COMMIT caps ----------
#define PASS_PUSH_MAX_MM    150   // re-plans may move the pass point closer freely, but
                                  // never more than this BEYOND the first plan's pass point
#define COMMIT_CAP_MM      1200   // driven path in one COMMIT; beyond it -> HOLD anyway
                                  // (v13 good passes: 460-1034 mm; failures 1380-1881 mm)

// ---------- v14: stall unstick (outside turns) ----------
#define UNSTICK_ON            1
#define UNSTICK_MM          150   // reverse this far on lane-heading PD, then NONE

// ---------- v14: close wrong-side pillar right after a corner ----------
#define BACKUP_ON             1
#define BACKUP_WINDOW_MM    800   // only this far into a section after a corner
#define BACKUP_BOTTOM       190   // first sighting this close (T9: 232; worked cases <= 157)
#define BACKUP_WRONG_X     0.60   // red at x_frac >= this, green at <= 1 - this = must cross in front
#define BACKUP_OK_BOTTOM    150   // stop backing once a measurable sighting is this far or farther
                                  // (E T6 b141-148 and E T7 b157 passed from there)
#define BACKUP_MAX_MM       200   // never reverse more than this (wall behind; stall also stops it)
#define BACK_STALL_MS       300   // reversing: encoder frozen this long = rear on a wall -> stop
#define BACK_TIMEOUT_MS    2500   // reversing never longer than this

// ---------- FAR -> COMMIT trigger ----------
#define COMMIT_BOTTOM       130   // bottom = y + h/2 (px, 0 = top).
                                  // STEP-1 logs: time from crossing this to
                                  // foot-leaves-frame = 0.44-0.56 s at 130,
                                  // 0.28 s at 160. TUNE THIS FIRST.

// ---------- FAR behaviour (SWERVE_MODE 1) ----------
#define FAR_MODE              0   // 0 = camera PD toward FAR_X target (Glitch-style,
                                  //     capped at FAR_MAX_CORR, no heading hold)
                                  // 1 = open-loop lane shift by encoder (below)
#define FAR_SHIFT_DEG        15   // heading offset from lane during the shift
#define FAR_SHIFT_MM        200   // distance at that heading; lateral ~ MM*sin(DEG)
                                  // 15 deg / 200 mm ~ 50 mm sideways
#define FAR_SKIP_X_RED     0.30   // red pillar already this far LEFT  -> no shift
#define FAR_SKIP_X_GREEN   0.70   // green pillar already this far RIGHT -> no shift

// ---------- Pillar x-targets, fraction of frame width ----------
#define FAR_X_RED          0.20   // FAR: push pillar toward LEFT edge  (car turns right)
#define FAR_X_GREEN        0.80   // FAR: push pillar toward RIGHT edge (car turns left)
#define COMMIT_X_RED       0.15   // pillar to LEFT edge  -> pass RIGHT
#define COMMIT_X_GREEN     0.85   // pillar to RIGHT edge -> pass LEFT

// ---------- Pillar PD ----------                        <<< STEP 1
// (mode 0 only from COMMIT on; FAR uses these in both modes)
#define PILLAR_KP          0.35   // servo deg per px
#define PILLAR_KD          0.00   // run-1: 0.02 at 25 Hz gave +-25 deg kicks. Keep 0
                                  // unless FAR visibly hunts; then try 0.003.
#define FAR_MAX_CORR         12   // FAR only nudges (was 30 -> swung heading 13 deg)
#define PILLAR_MAX_CORR      30   // mode-0 COMMIT clamp, +-30 like Glitch
#define PILLAR_SIGN          -1   // flip to +1 if servo goes the wrong way
#define PILLAR_LOST_MS      150   // no pillar this long -> "lost"

// ---------- After the pillar leaves the frame ----------  <<< STEP 1
#define HOLD_MODE             1   // 1 = by ENCODER (mm), 0 = by TIME (ms) fallback
#define PASS_HOLD_MM        200   // keep swerve heading this far past the pillar (car is 160 long)
#define RECOVER_MM          250   // (LANE_RETURN 0 only) heading-only recover distance
// ---- lane return (STEP 3 v2): after the pass, come back to the lateral
//      position the run started on (carLat = 0), not just the lane heading
#define LANE_RETURN           0   // v13: OFF. After HOLD: heading PD + wall term back to lane, drive on
#define RETURN_DEG           25   // heading offset used to come back across
#define RETURN_FRACTION     0.5   // come back this fraction of the offset (1.0 = all the way to the line)
#define RETURN_TOL_MM        40   // "there" when within this of the return target
#define RETURN_MAX_MM      1200   // give up (-> NONE) after this much distance
#define ACQ_STRAIGHT_DEG      8   // pillar seen during return: straighten first, acquire
                                  // only once heading is within this of the lane
#define PASS_HOLD_MS        400   //   (time fallback, HOLD_MODE 0 only)
#define RECOVER_MS          600

// ---------- Servo (v3b) ----------
#define SERVO_CENTER_DEG    100
#define SERVO_MIN_DEG        28
#define SERVO_MAX_DEG       142

// ---------- Heading PD (v3b) ----------
#define HEADING_KP          1.2
#define HEADING_KD         0.05
#define HEADING_MAX_CORR     30

// ---------- Wall term (v3b, only if USE_WALL) ----------
#define WALL_KP             1.2
#define WALL_TARGET_CM     42.0
#define WALL_SEE_CM        80.0
#define WALL_MAX_CORR        12

// ---------- v15: post-pass recovery (wall term, stronger, 80% target) ----------
#define POST_PASS_ON          1   // 0 = v14 behaviour
#define POST_PASS_FRACTION  0.8   // come back this fraction of the offset seen at the start of RECOVER
#define POST_PASS_MAX_CORR   25   // wall-term clamp during the window (normal: WALL_MAX_CORR 12)
#define POST_PASS_MM        800   // window length, from the start of RECOVER
#define POST_PASS_TOL_CM    3.0   // within this of the target (or crossed it) -> window ends
#define POST_PASS_MIN_CM    5.0   // offset smaller than this at the start -> no window needed

// ---------- Encoder / ToF (v3b) ----------
#define TICKS_PER_METER   657.0   // calibrated: stops at 1.00 m, checked 5x
#define TOF_HOLD_MS         150

// ---------- Logging ----------
#define LOG_TO_BT             1
#define BT_NAME     "YoLabs-FE"
#define LOG_BATCH            10

// ############################################################
// ##   end of tuning                                        ##
// ############################################################

Adafruit_VL53L0X tof[3];
Adafruit_BNO055  bno = Adafruit_BNO055(55, 0x28, &Wire);
Servo            steeringServo;
BluetoothSerial  SerialBT;
Adafruit_NeoPixel strip(16, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
HUSKYLENS        husky;

float target_heading = 0.0;       // lane heading, locked at START, +/-90 per corner
int   TURN = 0;
bool  isTurning = false, isReversing = false;
int   turnDirection = 0; int lockedTurnDirection = 0;
unsigned long turnStartTime = 0, lastTurnEndTime = 0;
float turnExitHeading = 0;        // lane heading +/- TURN_THROUGH_DEG during the turn
int   turnPhase = 0;              // split turn: 0 = forward arc, 1 = reverse arc
long  revStartTicks = 0;
float turnStartHeading = 0;
long  legStartTicks = 0;
float swerve_heading = 0.0;       // mode 1: target during COMMIT/HOLD
int   currentServoAngle = SERVO_CENTER_DEG;
unsigned long runStartTime = 0;
volatile long encoderTicks = 0;
float prevHeadingError = 0; unsigned long prevHeadingTime = 0;
float lastDist[3] = {999.0, 999.0, 999.0};
unsigned long lastGoodTime[3] = {0, 0, 0};
String logBuf; int logLines = 0;
float frontFirstSeen = 999;       // v14: was used in loop() but never declared (wall-signature
                                  // guard, disabled in v13). Declared only so the file compiles.

// pillar
enum PillarState { P_NONE, P_FAR, P_COMMIT, P_HOLD, P_RECOVER, P_SCANBACK, P_BACK };
PillarState pState = P_NONE;
int   pX = 0, pY = 0, pW = 0, pH = 0, pColor = 0, pCount = 0;
bool  pSeen = false;
unsigned long pLastSeen = 0;
float prevPillarErr = 0; unsigned long prevPillarTime = 0;
int   holdServo = SERVO_CENTER_DEG;
unsigned long phaseStartMs = 0; long phaseStartTicks = 0;
int   farPhase = 2;               // 0 = shifting out, 1 = back on lane heading, 2 = no shift
// mode 2 planner state
float latCarMM = 0;               // car lateral offset from the lane line, + = right
float fwdCarMM = 0;               // car forward progress along the lane
long  latLastTicks = 0;
float returnTargetLat = 0;        // lane-return aim point (fraction of the offset)
float planTheta = 0;              // solved swerve angle, deg (signed: + right)
float pillarFwdMM = 0;            // lane-frame forward position of the pillar (from last plan)
// v14 state
bool  pMeas = false;              // chosen block is unclipped and passed the size gate
int   pRej = 0;                   // blocks rejected by the gate / continuity this frame
int   refX = 0, refBottom = 0;    // tracking reference: last accepted block of the pillar
unsigned long refMs = 0;
float planPassMM = 0;             // pass point computed by the latest planSwerve() call
float pillarFwdFirst = 0;         // pass point of the first plan of this pillar
bool  pushCapLogged = false;
bool  holdFromCap = false;        // HOLD entered through COMMIT_CAP: no same-colour re-acquire in it
// v15 post-pass state
bool  postPass = false; bool postPassHaveO0 = false; float postPassO0 = 0; long postPassTicks = 0;
float postPassLaneW = 0;          // L+R seen with both walls in the window (0 = not yet)
float lastWallCorr = 0;           // logged
long  commitStartTicks = 0;
int   motorCmdSpeed = 0; bool motorCmdFwd = true;   // last command given to driveMotor
bool  backupUsed = false;         // close wrong-side backup already used in this section
bool  backForPillar = false; float backMaxMM = 0;
long  backLastTicks = 0; unsigned long backLastMove = 0;
// mode 3 state
int   pathA1 = 0, pathD1 = 0, pathA2 = 0, pathD2 = 0;
long  pathStartTicks = 0;
const char* POS_NAME[5] = {"FAR-LEFT", "LEFT", "CENTRE", "RIGHT", "FAR-RIGHT"};
long  farStartTicks = 0;
float shift_heading = 0.0;

float wallCentering0(float distL, float distR);   // v15
// ---------- helpers (v3b) ----------
void tcaselect(uint8_t i) { if (i > 7) return; Wire.beginTransmission(TCAADDR); Wire.write(1 << i); Wire.endTransmission(); }
void setNeoPixels(uint8_t r, uint8_t g, uint8_t b) { for (int i = 0; i < strip.numPixels(); i++) strip.setPixelColor(i, strip.Color(r, g, b)); strip.show(); }
void logLine(const String &s) {
  Serial.println(s);
#if LOG_TO_BT
  logBuf += s; logBuf += '\n'; logLines++;
  if (logLines >= LOG_BATCH) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; }
#endif
}
void logFlush() {
#if LOG_TO_BT
  if (logLines) { SerialBT.print(logBuf); logBuf = ""; logLines = 0; }
#endif
}
float getHeadingError(float target, float current) { float d = target - current; while (d > 180) d -= 360; while (d < -180) d += 360; return d; }
void driveMotor(int speed, bool forward) {
  motorCmdSpeed = speed; motorCmdFwd = forward;                        // v14: remembered for the stall guard
#if MOTOR_ON
  if (speed == 0) { analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW); }
  else { digitalWrite(IN2_PIN, forward ? HIGH : LOW); analogWrite(IN1_PIN, speed); }
#else
  analogWrite(IN1_PIN, 0); digitalWrite(IN2_PIN, LOW);
#endif
}
void setServo(int deg) { currentServoAngle = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG); steeringServo.write(currentServoAngle); }
void centerWheels() { steeringServo.write(SERVO_MIN_DEG); delay(400); steeringServo.write(SERVO_MAX_DEG); delay(400); steeringServo.write(SERVO_CENTER_DEG); delay(500); }
void IRAM_ATTR encoderISR() { if (digitalRead(ENCODER_B_PIN) == HIGH) encoderTicks++; else encoderTicks--; }
float mmSince(long ref) { long t; noInterrupts(); t = encoderTicks; interrupts(); return (t - ref) * (1000.0 / TICKS_PER_METER); }
void tofStartAll() { for (int i = 0; i < 3; i++) { tcaselect(i); tof[i].startRangeContinuous(30); } }
float getDistance(uint8_t ch) {
  tcaselect(ch);
  if (tof[ch].isRangeComplete()) {
    uint16_t mm = tof[ch].readRangeResult();
    if (mm > 0 && mm <= 2000) { lastDist[ch] = mm / 10.0; lastGoodTime[ch] = millis(); }
    else if (millis() - lastGoodTime[ch] > TOF_HOLD_MS) lastDist[ch] = 999.0;
  }
  return lastDist[ch];
}
float readHeading() { tcaselect(BNO_CH); sensors_event_t e; bno.getEvent(&e); return e.orientation.x; }
float headingPD(float err) {
  unsigned long now = millis();
  float dt = constrain((now - prevHeadingTime) / 1000.0, 0.001, 0.2);
  float d = HEADING_KD * (err - prevHeadingError) / dt;
  prevHeadingError = err; prevHeadingTime = now;
  return constrain(HEADING_KP * err + d, -HEADING_MAX_CORR, HEADING_MAX_CORR);
}
// v15: lateral offset from the open-challenge line in cm (+ = right), same wall rules as below.
//      false = no wall seen (the open-challenge term is 0 there too)
bool wallOffsetCm(float distL, float distR, float &o) {
  if (distL < WALL_SEE_CM && distR < WALL_SEE_CM) { o = (distL - distR) * 0.5; return true; }
  if (lockedTurnDirection == -1) {
    if (distL < WALL_SEE_CM) { o = distL - WALL_TARGET_CM; return true; }
    if (distR < WALL_SEE_CM) { o = WALL_TARGET_CM - distR; return true; }
  } else {
    if (distR < WALL_SEE_CM) { o = WALL_TARGET_CM - distR; return true; }
    if (distL < WALL_SEE_CM) { o = distL - WALL_TARGET_CM; return true; }
  }
  return false;
}
void endPostPass(const char* why) {
  if (!postPass) return;
  postPass = false;
  logLine(String("  POST-PASS end: ") + why + "  mm=" + String(mmSince(0), 0));
}
float wallCentering(float distL, float distR) { lastWallCorr = wallCentering0(distL, distR); return lastWallCorr; }
float wallCentering0(float distL, float distR) {
#if USE_WALL
#if POST_PASS_ON
  if (postPass) {
    float o;
    // offset from lane centre. Both walls: (L-R)/2, and remember the width. One wall and a
    // known width: same reference from that wall (no jump when the far wall drops past 80 cm).
    if (distL < WALL_SEE_CM && distR < WALL_SEE_CM) { o = (distL - distR) * 0.5; postPassLaneW = distL + distR; }
    else if (postPassLaneW > 0 && distL < WALL_SEE_CM) o = distL - postPassLaneW * 0.5;
    else if (postPassLaneW > 0 && distR < WALL_SEE_CM) o = postPassLaneW * 0.5 - distR;
    else if (!wallOffsetCm(distL, distR, o)) return 0;                // no wall seen: nothing to steer by
    if (!postPassHaveO0) {
      postPassO0 = o; postPassHaveO0 = true;
      if (fabs(o) < POST_PASS_MIN_CM) { endPostPass("already near the line"); }
      else logLine("  POST-PASS start: offset " + String(o, 0) + " cm -> target " + String((1.0 - POST_PASS_FRACTION) * o, 0) + " cm");
    }
    if (postPass) {
      float e = o - (1.0 - POST_PASS_FRACTION) * postPassO0;            // + = still right of the target
      if (fabs(e) <= POST_PASS_TOL_CM || e * postPassO0 < 0) endPostPass("target reached");
      else return constrain(-WALL_KP * e, -POST_PASS_MAX_CORR, POST_PASS_MAX_CORR);
    }
  }
#endif
  float wc = 0;                                                        // open-challenge v3b term
  if (distL < WALL_SEE_CM && distR < WALL_SEE_CM) {
    wc = (distR - distL) * (WALL_KP * 0.5);
  } else if (lockedTurnDirection == 1) {
    if      (distR < WALL_SEE_CM) wc -= (WALL_TARGET_CM - distR) * WALL_KP;
    else if (distL < WALL_SEE_CM) wc += (WALL_TARGET_CM - distL) * WALL_KP;
  } else if (lockedTurnDirection == -1) {
    if      (distL < WALL_SEE_CM) wc += (WALL_TARGET_CM - distL) * WALL_KP;
    else if (distR < WALL_SEE_CM) wc -= (WALL_TARGET_CM - distR) * WALL_KP;
  } else {
    if      (distR < WALL_SEE_CM) wc -= (WALL_TARGET_CM - distR) * WALL_KP;
    else if (distL < WALL_SEE_CM) wc += (WALL_TARGET_CM - distL) * WALL_KP;
  }
  return constrain(wc, -WALL_MAX_CORR, WALL_MAX_CORR);
#else
  return 0;
#endif
}
// corner-style: full arc until within SWERVE_BLEND_DEG, then heading PD
void driveArcToHeading(float hdgTarget, float currentHeading, int speed) {
  float err = getHeadingError(hdgTarget, currentHeading);        // + = need to turn right
  if (err >  SWERVE_BLEND_DEG)      setServo(SERVO_SHIFT_RIGHT);
  else if (err < -SWERVE_BLEND_DEG) setServo(SERVO_SHIFT_LEFT);
  else setServo(SERVO_CENTER_DEG + headingPD(err));
  driveMotor(speed, true);
}
// steer to a given heading, no wall term
void driveToHeading(float hdgTarget, float currentHeading, int speed) {
  setServo(SERVO_CENTER_DEG + headingPD(getHeadingError(hdgTarget, currentHeading)));
  driveMotor(speed, true);
}
void driveStraightStep(float headingError, float distL, float distR, int speed) {
  setServo(SERVO_CENTER_DEG + headingPD(headingError) + wallCentering(distL, distR));
  driveMotor(speed, true);
}

// ---------- camera ----------
// v14 gate: 0 = reject (not pillar-shaped / too short for its distance),
//           1 = clipped at frame side or bottom (track only, never re-plan),
//           2 = measurable (unclipped, pillar-shaped, tall enough).
int gateClass(int id, int xc, int yc, int w, int h) {
#if GATE_ON
  int bottom = yc + h / 2;
  bool clipB = (bottom >= CLIP_BOTTOM_PX);
  bool clipS = (xc - w / 2 <= CLIP_EDGE_PX) || (xc + w / 2 >= FRAME_W - CLIP_EDGE_PX);
  if (clipB || clipS) return 1;
  float aspMin = (id == GREEN_ID) ? GREEN_ASPECT_MIN : RED_ASPECT_MIN;
  if ((float)h < aspMin * (float)w) return 0;
  float frac = (id == GREEN_ID) ? PIL_H_MIN_FRAC_G : PIL_H_MIN_FRAC;
  if ((float)h < frac * (PIL_H_SLOPE * bottom + PIL_H_OFS)) return 0;
  return 2;
#else
  return 2;
#endif
}

// NONE       : nearest pillar-SHAPED block (lowest in frame), entry filter on.
// FAR/COMMIT : block of the same colour nearest the last x.
//              v14: also through the gate + continuity (see header).
bool readPillar(bool tracking) {
  pSeen = false; pCount = 0; pRej = 0;
  if (!husky.request()) return false;
  int bestY = -1; int bestDx = 100000; bool bestMeas = false;
  while (husky.available()) {
    HUSKYLENSResult r = husky.read();
    if (r.command != COMMAND_RETURN_BLOCK) continue;
    if (r.ID != RED_ID && r.ID != GREEN_ID) continue;
    if (r.height < MIN_PILLAR_H) continue;
    pCount++;
    if (!tracking) {
      if (r.width > PILLAR_W_MAX) continue;                                   // line
      if (r.yCenter + r.height / 2 > ENTRY_BOTTOM_MAX) continue;               // too close to be a new pillar
      if ((float)r.height / (float)(r.width > 1 ? r.width : 1) < PILLAR_ASPECT_MIN) continue;
      int g = gateClass(r.ID, r.xCenter, r.yCenter, r.width, r.height);        // v14: size for its distance
      if (g == 0) { pRej++; continue; }
      if (r.yCenter > bestY) { bestY = r.yCenter; pX = r.xCenter; pY = r.yCenter; pW = r.width; pH = r.height; pColor = r.ID; pSeen = true; bestMeas = (g == 2); }
    } else {
      if (r.ID != pColor) continue;
      int g = gateClass(r.ID, r.xCenter, r.yCenter, r.width, r.height);        // v14
      if (g == 0) { pRej++; continue; }
#if GATE_ON
      int dx = abs(r.xCenter - refX);
      int dxLim = TRACK_MAX_DX + (int)(TRACK_DX_PER_100MS * ((millis() - refMs) / 100.0));
      if (dx > dxLim) { pRej++; continue; }                                   // v14: jumped sideways
      if (r.yCenter + r.height / 2 < refBottom - TRACK_MAX_BACK) { pRej++; continue; }   // v14: jumped farther
#else
      int dx = abs(r.xCenter - pX);                                           // v13: nearest the last x
#endif
      if (dx < bestDx) { bestDx = dx; pX = r.xCenter; pY = r.yCenter; pW = r.width; pH = r.height; pSeen = true; bestMeas = (g == 2); }
    }
  }
  pMeas = pSeen && bestMeas;
  if (pSeen) pLastSeen = millis();
  if (pSeen && tracking) { refX = pX; refBottom = pY + pH / 2; refMs = millis(); }
  return pSeen;
}

bool commitNow() { return (pY + pH / 2) >= COMMIT_BOTTOM; }

// ---------- pillar PD: returns servo angle ----------
int pillarServo(bool commit) {
  float frac = (pColor == RED_ID) ? (commit ? COMMIT_X_RED : FAR_X_RED)
                                  : (commit ? COMMIT_X_GREEN : FAR_X_GREEN);
  float err = frac * FRAME_W - pX;                 // + = pillar left of target
  unsigned long now = millis();
  float dt = constrain((now - prevPillarTime) / 1000.0, 0.001, 0.2);
  float d = PILLAR_KD * (err - prevPillarErr) / dt;
  prevPillarErr = err; prevPillarTime = now;
  float lim = commit ? PILLAR_MAX_CORR : FAR_MAX_CORR;
  float corr = constrain(PILLAR_KP * err + d, -lim, lim);
  return SERVO_CENTER_DEG + PILLAR_SIGN * corr;
}

// ---------- mode 2 planner ----------
// lateral gain (mm) of: STEER_LAG straight, arc to theta, straight at theta
// until forward distance fwd is used up
// yawNowDeg = how far the car has ALREADY turned toward the pass side (>= 0).
// The arc is charged only from yawNow to theta; lag only when yawNow == 0.
float lateralGain(float thetaDeg, float fwdMM, float yawNowDeg) {
  float th = radians(thetaDeg), y0 = radians(min(yawNowDeg, thetaDeg));   // y0 may be NEGATIVE (yawed the wrong way)
  float lag = (fabs(yawNowDeg) <= 0.5) ? STEER_LAG_MM : 0;
  float arcFwd = TURN_R_MM * (sin(th) - sin(y0));
  float arcLat = TURN_R_MM * (cos(y0) - cos(th));
  float f = fwdMM - lag - arcFwd;
  if (f < 0) { float a = y0 + max(0.0f, fwdMM - lag) / TURN_R_MM; return TURN_R_MM * (cos(y0) - cos(min(a, th))); }
  return arcLat + f * tan(th);
}
// update car lateral position from encoder + gyro (call every loop)
void trackLateral(float offRightDeg) {
  long t; noInterrupts(); t = encoderTicks; interrupts();
  float ds = (t - latLastTicks) * (1000.0 / TICKS_PER_METER); latLastTicks = t;
  latCarMM += ds * sin(radians(offRightDeg));
  fwdCarMM += ds * cos(radians(offRightDeg));
}
// solve theta from the pillar seen now. offRightDeg = car yaw right of lane.
// returns signed angle (+ right, - left) and logs the plan.
// v14: the pass point goes to planPassMM; the CALLER decides whether it becomes pillarFwdMM.
float planSwerve(float offRightDeg, bool logIt) {
  float bottom = pY + pH / 2.0;
  float d_mm   = 10.0 * CAM_K / max(bottom - CAM_H0, 5.0f);
  float beta   = (pX / (float)FRAME_W - 0.5) * CAM_HFOV_DEG;        // + = pillar right of nose
  float bearL  = beta + offRightDeg;                                 // bearing in lane frame
  float pilLat = latCarMM + d_mm * sin(radians(bearL));             // pillar lateral, lane frame
  float fwd    = d_mm * cos(radians(bearL));                         // forward distance to it
  planPassMM   = fwdCarMM + fwd;                                     // camera-independent pass point candidate
  float need;                                                        // lateral the car must still gain
  int sgn;
  if (pColor == RED_ID) { sgn = +1; need = (pilLat + CLEAR_MM) - latCarMM; }
  else                  { sgn = -1; need = latCarMM - (pilLat - CLEAR_MM); }
  need *= NEED_SCALE;
  float yawNow = sgn * offRightDeg;                                  // + = already turned toward pass side
  int th = THETA_MIN;
  if (need > 0) { for (th = THETA_MIN; th < THETA_MAX; th++) if (lateralGain(th, fwd, yawNow) >= need) break; }
  th = constrain((int)(th * THETA_SCALE + 0.5), THETA_MIN, THETA_MAX);
#if ZERO_THETA_IF_CLEAR
  if (need <= 0) th = 0;
#endif
  if (logIt) logLine("  plan: d=" + String(d_mm, 0) + "mm bearing=" + String(bearL, 1) + " pillarLat=" + String(pilLat, 0) +
                     " carLat=" + String(latCarMM, 0) + " need=" + String(need, 0) + " fwd=" + String(fwd, 0) + " -> theta=" + String(sgn * th) +
                     " (max gain " + String(lateralGain(THETA_MAX, fwd, yawNow), 0) + (pMeas ? "" : ", clipped block") + ")");
  return sgn * th;
}
void setSwerveHeading(float thetaSigned) {
  swerve_heading = target_heading + thetaSigned;
  if (swerve_heading >= 360) swerve_heading -= 360;
  if (swerve_heading <    0) swerve_heading += 360;
}

int pillarCount = 0;
void acquirePillar(float offRightDeg, int n) {
  pillarCount = n;
  planTheta = planSwerve(offRightDeg, true);
  pillarFwdMM = planPassMM; pillarFwdFirst = planPassMM; pushCapLogged = false; holdFromCap = false;   // v14
  endPostPass("new pillar");                                                        // v15
  commitStartTicks = encoderTicks;                                                  // v14
  refX = pX; refBottom = pY + pH / 2; refMs = millis();                             // v14: tracking reference
  setSwerveHeading(planTheta);
  pState = P_COMMIT;
  logLine(String("PILLAR #") + pillarCount + " " + (pColor == RED_ID ? "RED" : "GREEN") + " x=" + String(pX / (float)FRAME_W, 2) + " bottom=" + String(pY + pH / 2) +
          "  -> SWERVE " + String(planTheta, 0) + " deg hdg=" + String(swerve_heading, 1) + "  carLat=" + String(latCarMM, 0) + " yaw=" + String(offRightDeg, 1) + " mm=" + String(mmSince(0), 0));
}

void stopRun(const String &why);   // defined below
void enterPhase(PillarState s, const char* name);
// ---------- v14: reverse on lane-heading PD (backup for a clean entry, or stall unstick) ----------
void startBack(float maxMM, bool forPillar, const char* why) {
  driveMotor(0, true); delay(100);
  backMaxMM = maxMM; backForPillar = forPillar;
  if (forPillar) backupUsed = true;
  long t; noInterrupts(); t = encoderTicks; interrupts();
  backLastTicks = t; backLastMove = millis();
  logLine(String("  BACK ") + String(maxMM, 0) + " mm: " + why + "  h_err=" + String(getHeadingError(target_heading, readHeading()), 1) + " mm=" + String(mmSince(0), 0));
  enterPhase(P_BACK, "BACK");
}
bool backupWanted() {
#if BACKUP_ON && SWERVE_MODE == 2
  if (TURN == 0 || backupUsed) return false;
  float leg = mmSince(legStartTicks);
  if (leg > BACKUP_WINDOW_MM) return false;
  if (pY + pH / 2 < BACKUP_BOTTOM) return false;
  float xf = pX / (float)FRAME_W;
  return (pColor == RED_ID) ? (xf >= BACKUP_WRONG_X) : (xf <= 1.0 - BACKUP_WRONG_X);
#else
  return false;
#endif
}
// ---------- corners (open v3b) ----------
float legMM() { return mmSince(legStartTicks); }
bool decideTurn(float distF, float distL, float distR) {
  if (distF > TURN_THRESHOLD_CM || distF <= 2.0) return false;
  if (millis() - lastTurnEndTime < TURN_COOLDOWN_MS) return false;
  if (TURN > 0 && MIN_LEG_MM > 0 && legMM() < MIN_LEG_MM) return false;   // first corner exempt
  bool rightOpen = (distR > SIDE_GAP_CM), leftOpen = (distL > SIDE_GAP_CM);
  if (lockedTurnDirection == 0) {
    if (rightOpen && !leftOpen)      { lockedTurnDirection =  1; logLine("LOCKED: CLOCKWISE");      return true; }
    else if (leftOpen && !rightOpen) { lockedTurnDirection = -1; logLine("LOCKED: ANTI-CLOCKWISE"); return true; }
    else if (distF <= BLIND_LOCK_CM) { lockedTurnDirection = (distR > distL) ? 1 : -1; logLine(String("BLIND LOCK: ") + (lockedTurnDirection == 1 ? "CW" : "CCW")); return true; }
    return false;
  }
  if (lockedTurnDirection ==  1 && (rightOpen || distF <= BLIND_LOCK_CM)) return true;
  if (lockedTurnDirection == -1 && (leftOpen  || distF <= BLIND_LOCK_CM)) return true;
  return false;
}
void startTurn() {
  isTurning = true; turnStartTime = millis(); setNeoPixels(255, 165, 0);
  endPostPass("corner");                                              // v15
  turnDirection = lockedTurnDirection;
  if (turnDirection == 1) { target_heading += 90.0; currentServoAngle = SERVO_SHIFT_RIGHT; }
  else                    { target_heading -= 90.0; currentServoAngle = SERVO_SHIFT_LEFT;  }
  if (target_heading >= 360.0) target_heading -= 360.0;
  if (target_heading <    0.0) target_heading += 360.0;
  turnPhase = 0; turnStartHeading = readHeading();
  turnExitHeading = target_heading + turnDirection * TURN_THROUGH_DEG;
  if (turnExitHeading >= 360.0) turnExitHeading -= 360.0;
  if (turnExitHeading <    0.0) turnExitHeading += 360.0;
  logLine("TURN " + String(TURN + 1) + " start, leg " + String(legMM(), 0) + " mm, new lane hdg=" + String(target_heading, 1) + " exit via " + String(turnExitHeading, 1));
}
float laneCentreLat(float distL, float distR) {                        // + = right of lane centre, mm
  if (distL < LANE_ANCHOR_SEE_CM && distR < LANE_ANCHOR_SEE_CM) return (distL - distR) * 5.0;
  return latCarMM;                                                     // can't tell: keep current
}
void endTurn(const char* why, float distL, float distR) {
  isTurning = false; isReversing = false; TURN++;
  lastTurnEndTime = millis(); legStartTicks = encoderTicks;
  latCarMM = laneCentreLat(distL, distR); fwdCarMM = 0; pillarCount = 0;   // new lane frame, 0 = lane centre
  pState = P_NONE;
  backupUsed = false;                                                  // v14: one backup per section
#if SCAN_BACK
  driveMotor(0, true); delay(150);                                     // settle before reversing
  enterPhase(P_SCANBACK, "SCANBACK");
#elif CENTRE_AFTER_TURN
  if (fabs(latCarMM) > RETURN_TOL_MM) { returnTargetLat = 0; enterPhase(P_RECOVER, "CENTRE after turn"); }
#endif
  currentServoAngle = SERVO_CENTER_DEG; steeringServo.write(currentServoAngle);
  setNeoPixels(0, 255, 255);
  logLine(String("TURN ") + TURN + " done (" + why + ")  lane centre anchor carLat=" + String(latCarMM, 0));
}
// returns true if a pillar was acquired out of the blend
bool turnStep(float laneError, float currentHeading, float distF, float distL, float distR) {
#if SPLIT_TURN
  {
    float turned = fabs(getHeadingError(currentHeading, turnStartHeading));   // degrees turned so far
    float err = getHeadingError(target_heading, currentHeading);            // + = need more right
    if (turnPhase == 0) {                                                     // FORWARD on lock
      steeringServo.write(currentServoAngle); driveMotor(TURN_SPEED, true);
      { static long lastT = 0; static unsigned long lastMove = 0; long t; noInterrupts(); t = encoderTicks; interrupts();
        if (t != lastT) { lastT = t; lastMove = millis(); }
        if (lastMove == 0) lastMove = millis();
        if (millis() - lastMove > STALL_MS) { logLine("  split turn: STALL in fwd half after " + String(turned, 0) + " deg -> reversing"); turned = TURN_FWD_DEG; lastMove = millis(); }
      }
      if (turned >= TURN_FWD_DEG) {
        driveMotor(0, true); delay(150);
        currentServoAngle = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;   // opposite lock for reverse
        steeringServo.write(currentServoAngle); delay(200);
        turnPhase = 1; revStartTicks = encoderTicks;
        logLine("  split turn: fwd " + String(turned, 0) + " deg done, reversing  mm=" + String(mmSince(0), 0));
      }
      return false;
    }
    // REVERSE on opposite lock: same rotation direction as forward-on-lock.
    // Once the heading is reached, keep reversing straight until REV_MIN_MM.
    float reversed = -mmSince(revStartTicks);
    if (fabs(err) > SPLIT_REV_BLEND_DEG) { steeringServo.write(currentServoAngle); }
    else { setServo(SERVO_CENTER_DEG - headingPD(err)); }                    // reverse-PD (sign inverted)
    driveMotor(SPLIT_REV_SPEED, false);
    bool headingOK = fabs(err) < TURN_EXIT_DEG;
    if ((headingOK && reversed >= REV_MIN_MM) || millis() - turnStartTime > TURN_TIMEOUT_MS) {
      driveMotor(0, true); delay(150);
      logLine("  split turn: reverse done h_err=" + String(err, 1) + " reversed=" + String(reversed, 0) + " mm  mm=" + String(mmSince(0), 0));
      endTurn(headingOK ? "split" : "timeout", distL, distR);
    }
    return false;
  }
#endif
  float headingError = getHeadingError(turnExitHeading, currentHeading);   // turn-through target
  isReversing = false;
  float turned = 90.0 - fabs(getHeadingError(target_heading, currentHeading));   // degrees of the 90 completed
  if (turned >= TURN_ABORT_AFTER_DEG && distF > 2.0 && distF < TURN_ABORT_FRONT_CM) {
    logLine("  TURN ABORT: obstacle ahead F=" + String(distF, 0) + " cm after " + String(turned, 0) + " deg -> backing out");
    endTurn("obstacle in arc", distL, distR);                              // counts the turn, sets new lane, -> SCANBACK
    return true;
  }
  if (isReversing) {
    int a = (turnDirection == 1) ? SERVO_SHIFT_LEFT : SERVO_SHIFT_RIGHT;
    steeringServo.write(a); currentServoAngle = a; driveMotor(REVERSE_SPEED, false);
  } else if (TURN_BLEND_DEG > 0 && fabs(headingError) < TURN_BLEND_DEG) {
    if (fabs(laneError) <= ACQ_STRAIGHT_DEG && readPillar(false)) {     // pillar right after the corner
      endTurn("pillar in view", distL, distR);
      acquirePillar(-laneError, 1);
      return true;
    }
    setServo(SERVO_CENTER_DEG + headingPD(headingError)); driveMotor(TURN_SPEED, true);   // PD to the exit heading, no wall term
  } else {
    steeringServo.write(currentServoAngle); driveMotor(TURN_SPEED, true);
  }
  if (fabs(headingError) < TURN_EXIT_DEG || millis() - turnStartTime > TURN_TIMEOUT_MS) endTurn(fabs(headingError) < TURN_EXIT_DEG ? "heading" : "timeout", distL, distR);
  return false;
}
void finishRun() {
  setNeoPixels(255, 0, 255); long t0 = encoderTicks;
  logLine("corners done, finishing " + String(FINISH_MM) + " mm");
  while (mmSince(t0) < FINISH_MM) {
    float distR = getDistance(TOF_RIGHT_CH), distL = getDistance(TOF_LEFT_CH);
    driveStraightStep(getHeadingError(target_heading, readHeading()), distL, distR, OBST_SPEED);
    delay(15);
  }
  stopRun("finished");
}

bool phaseDone(unsigned long ms, float mm) {
#if HOLD_MODE == 0
  return millis() - phaseStartMs >= ms;
#else
  return mmSince(phaseStartTicks) >= mm;
#endif
}
void enterPhase(PillarState s, const char* name) {
  pState = s; phaseStartMs = millis(); phaseStartTicks = encoderTicks;
  logLine(String("  -> ") + name);
}

// ---------- pillar state machine ----------
// returns true if it drove the servo/motor this loop
bool pillarStep(float headingError, float currentHeading, float distL, float distR) {
  bool seen = (pState == P_BACK) ? false : readPillar(pState != P_NONE);   // v14: BACK reads the camera itself
  bool lostLong = (millis() - pLastSeen > PILLAR_LOST_MS);

  switch (pState) {
    case P_NONE:
#if SWERVE_MODE == 2
      if (seen) {
        if (backupWanted()) {                                              // v14: close wrong-side after a corner
          startBack(BACKUP_MAX_MM, true, (String(pColor == RED_ID ? "RED" : "GREEN") + " x=" + String(pX / (float)FRAME_W, 2) + " bottom=" + String(pY + pH / 2) + " too close, must cross").c_str());
          return true;
        }
        acquirePillar(-headingError, 1);
      }
      return false;
#elif SWERVE_MODE == 3
      if (seen) {
        float xf = pX / (float)FRAME_W;
        int pos = (xf < POS_FL_MAX) ? 0 : (xf < POS_L_MAX) ? 1 : (xf < POS_C_MAX) ? 2 : (xf < POS_R_MAX) ? 3 : 4;
        if (pColor == RED_ID) { pathA1 = RED_A1[pos];   pathD1 = RED_D1[pos];   pathA2 = RED_A2[pos];   pathD2 = RED_D2[pos]; }
        else                  { pathA1 = GREEN_A1[pos]; pathD1 = GREEN_D1[pos]; pathA2 = GREEN_A2[pos]; pathD2 = GREEN_D2[pos]; }
        pathStartTicks = encoderTicks;
        setSwerveHeading(pathA1);
        pState = P_FAR;                                                    // phase 1
        logLine(String("PILLAR ") + (pColor == RED_ID ? "RED" : "GREEN") + " " + POS_NAME[pos] + " (x=" + String(xf, 2) + " bottom=" + String(pY + pH / 2) + ")" +
                "  path: A1=" + pathA1 + " D1=" + pathD1 + " A2=" + pathA2 + " D2=" + pathD2 + "  mm=" + String(mmSince(0), 0));
      }
      return false;
#elif SWERVE_MODE
      if (seen) {
        float xf = pX / (float)FRAME_W;
        bool near = (pColor == RED_ID) ? (xf <= FAR_SKIP_X_RED) : (xf >= FAR_SKIP_X_GREEN);
        float deg = near ? SWERVE_DEG_NEAR : SWERVE_DEG;
        swerve_heading = target_heading + (pColor == RED_ID ? deg : -deg);
        if (swerve_heading >= 360) swerve_heading -= 360;
        if (swerve_heading <    0) swerve_heading += 360;
        pState = P_COMMIT;
        logLine(String("PILLAR ") + (pColor == RED_ID ? "RED" : "GREEN") + " x=" + String(xf, 2) + " bottom=" + String(pY + pH / 2) +
                "  -> SWERVE " + String(deg, 0) + " deg " + (pColor == RED_ID ? "RIGHT" : "LEFT") + " hdg=" + String(swerve_heading, 1) + "  mm=" + String(mmSince(0), 0));
      }
      return false;
#endif
      if (seen) { pState = P_FAR;
                  float frac0 = (pColor == RED_ID) ? FAR_X_RED : FAR_X_GREEN;
                  prevPillarErr = frac0 * FRAME_W - pX; prevPillarTime = millis();   // mode 0: no D kick
                  float xf = pX / (float)FRAME_W;
                  bool skip = (pColor == RED_ID) ? (xf <= FAR_SKIP_X_RED) : (xf >= FAR_SKIP_X_GREEN);
                  farPhase = skip ? 2 : 0; farStartTicks = encoderTicks;
                  shift_heading = target_heading + (pColor == RED_ID ? FAR_SHIFT_DEG : -FAR_SHIFT_DEG);
                  if (shift_heading >= 360) shift_heading -= 360;
                  if (shift_heading <    0) shift_heading += 360;
#if FAR_MODE == 0
                  logLine(String("PILLAR ") + (pColor == RED_ID ? "RED" : "GREEN") + " far x=" + String(xf, 2) + "  camera PD to " + String(frac0, 2));
#else
                  logLine(String("PILLAR ") + (pColor == RED_ID ? "RED" : "GREEN") + " far x=" + String(xf, 2) +
                          (skip ? "  no shift" : String("  shift ") + (pColor == RED_ID ? "RIGHT" : "LEFT") + " " + FAR_SHIFT_MM + " mm hdg=" + String(shift_heading, 1)));
#endif
                }
      return false;

    case P_FAR:
#if SWERVE_MODE == 3
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
      if (mmSince(pathStartTicks) >= pathD1) {
        setSwerveHeading(pathA2); pathStartTicks = encoderTicks; pState = P_COMMIT;
        logLine("  -> phase 2: A2=" + String(pathA2) + " hdg=" + String(swerve_heading, 1) + "  h_err=" + String(headingError, 1) + " lat=" + String(latCarMM, 0) + " mm=" + String(mmSince(0), 0));
      }
      return true;
#endif
      if (seen) {
        if (commitNow()) {
          pState = P_COMMIT;
          // swerve = lane +/- SWERVE_DEG, but never LESS than the offset FAR already built
          float offNow = -headingError;                       // + = already turned right of lane
          float off = (pColor == RED_ID) ? max((float)SWERVE_DEG, offNow) : -max((float)SWERVE_DEG, -offNow);
          swerve_heading = target_heading + off;
          if (swerve_heading >= 360) swerve_heading -= 360;
          if (swerve_heading <    0) swerve_heading += 360;
          logLine("  -> COMMIT x=" + String(pX / (float)FRAME_W, 2) + " bottom=" + String(pY + pH / 2) + " h=" + String(pH) + " w=" + String(pW) +
                  (SWERVE_MODE ? "  swerve_hdg=" + String(swerve_heading, 1) : ""));
        }
#if SWERVE_MODE
        if (pState == P_COMMIT) driveToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
        else {
#if FAR_MODE == 0
          setServo(pillarServo(false)); driveMotor(PILLAR_SPEED, true);      // camera PD, +-FAR_MAX_CORR
#else
          if (farPhase == 0 && mmSince(farStartTicks) >= FAR_SHIFT_MM) { farPhase = 1; logLine("  shift done, back to lane heading  mm=" + String(mmSince(0), 0)); }
          driveToHeading(farPhase == 0 ? shift_heading : target_heading, currentHeading, PILLAR_SPEED);
#endif
        }
#else
        setServo(pillarServo(pState == P_COMMIT)); driveMotor(PILLAR_SPEED, true);
#endif
        return true;
      }
      if (lostLong) { pState = P_NONE; logLine("  far: lost"); }
      return false;

    case P_COMMIT:
#if SWERVE_MODE == 3
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
      if (mmSince(pathStartTicks) >= pathD2) {
        enterPhase(P_RECOVER, "RECOVER (arc back to lane)");
        logLine("     h_err=" + String(headingError, 1) + " lat=" + String(latCarMM, 0) + " mm=" + String(mmSince(0), 0));
      }
      return true;
#elif SWERVE_MODE == 2
#if REPLAN_MODE > 0
      if (seen && pMeas) {                                                 // v14: only measurable blocks re-plan
        float th = planSwerve(-headingError, false);
        float pass = min(planPassMM, pillarFwdFirst + PASS_PUSH_MAX_MM);    // v14: closer freely, farther capped
        if (planPassMM > pass && !pushCapLogged) { logLine("  pass point capped at first plan +" + String(PASS_PUSH_MAX_MM) + " mm (asked +" + String(planPassMM - pillarFwdFirst, 0) + ")"); pushCapLogged = true; }
        pillarFwdMM = pass;
        bool take = (fabs(th - planTheta) >= 2);
#if REPLAN_MODE == 2
        take = take && (fabs(th) < fabs(planTheta));                      // only shrink
#endif
        if (take) { planTheta = th; setSwerveHeading(planTheta); logLine("  re-plan theta=" + String(planTheta, 0) + " hdg=" + String(swerve_heading, 1)); }
      }
#endif
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
      // pillar 'passed' = car forward progress reaches the pillar's forward position.
      // Camera losing it (yaw takes it out of frame) does NOT end COMMIT.
      {
        bool capped = (mmSince(commitStartTicks) >= COMMIT_CAP_MM);         // v14
        if (fwdCarMM >= pillarFwdMM || capped) {
          holdFromCap = capped && fwdCarMM < pillarFwdMM;
          enterPhase(P_HOLD, "HOLD");
          logLine(String("     ") + (capped && fwdCarMM < pillarFwdMM ? "COMMIT CAP " + String(COMMIT_CAP_MM) + " mm reached" : String("nose level with pillar")) +
                  ": fwd=" + String(fwdCarMM, 0) + " lat=" + String(latCarMM, 0) + " h_err=" + String(headingError, 1) + (seen ? " (pillar still in view)" : " (pillar out of view)"));
        }
      }
      return true;
#elif SWERVE_MODE
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);   // pillar in view or not
#else
      if (seen) { setServo(pillarServo(true)); driveMotor(PILLAR_SPEED, true); return true; }
#endif
      if (lostLong) { holdServo = currentServoAngle; enterPhase(P_HOLD, "HOLD"); logLine("     servo=" + String(holdServo) + " h_err=" + String(headingError, 1) + "  mm=" + String(mmSince(0), 0)); }
      return true;

    case P_HOLD:
#if SWERVE_MODE == 2
      {
        int lastColor = pColor;
        bool newSeen = readPillar(false);                    // entry filter on: a NEW pillar-shaped block
        if (newSeen && pColor == lastColor && !holdFromCap) { acquirePillar(-headingError, pillarCount + 1); return true; }   // v14: not after a cap
        pColor = lastColor;
      }
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
      if (phaseDone(PASS_HOLD_MS, PASS_HOLD_MM)) { returnTargetLat = latCarMM * (1.0 - RETURN_FRACTION); enterPhase(P_RECOVER, "RECOVER"); logLine("     return target carLat=" + String(returnTargetLat, 0) + " from " + String(latCarMM, 0));
#if POST_PASS_ON
        postPass = true; postPassHaveO0 = false; postPassLaneW = 0; postPassTicks = encoderTicks;   // v15
#endif
      }
      return true;
#elif SWERVE_MODE
      driveArcToHeading(swerve_heading, currentHeading, PILLAR_SPEED);
#else
      setServo(holdServo); driveMotor(PILLAR_SPEED, true);
#endif
      if (phaseDone(PASS_HOLD_MS, PASS_HOLD_MM)) enterPhase(P_RECOVER, "RECOVER");
      return true;

    case P_SCANBACK: {
      bool seenNow = readPillar(false);
      if (seenNow && fabs(headingError) <= ACQ_STRAIGHT_DEG + TURN_THROUGH_DEG) {
        driveMotor(0, true); delay(100);
        logLine("  scanback: pillar in view, going forward  mm=" + String(mmSince(0), 0));
        acquirePillar(-headingError, 1);                                 // plans from current yaw
        return true;
      }
      // reverse, moving the rear toward lane centre: nose away from centre while off-centre,
      // then straight. Steering sign inverts in reverse.
      {
        float arcBack = TURN_R_MM * (1 - cos(radians(SCAN_BACK_SHIFT_DEG)));
        float hdg = target_heading;
        if (fabs(latCarMM) > arcBack + RETURN_TOL_MM) hdg = target_heading + (latCarMM > 0 ? SCAN_BACK_SHIFT_DEG : -SCAN_BACK_SHIFT_DEG);
        if (hdg >= 360) hdg -= 360; if (hdg < 0) hdg += 360;
        setServo(SERVO_CENTER_DEG - headingPD(getHeadingError(hdg, currentHeading)));
        driveMotor(SCAN_BACK_SPEED, false);
      }
      if (mmSince(phaseStartTicks) <= -SCAN_BACK_MM) {
        driveMotor(0, true); delay(100);
        logLine("  scanback done, nothing seen  carLat=" + String(latCarMM, 0) + " h_err=" + String(headingError, 1));
#if CENTRE_AFTER_TURN
        returnTargetLat = 0; enterPhase(P_RECOVER, "CENTRE after turn");
#else
        enterPhase(P_NONE, "NONE");
#endif
      }
      return true;
    }

    case P_BACK: {                                                         // v14
      // reverse straight on the lane heading (reverse-PD: sign inverted, as in the split turn)
      setServo(SERVO_CENTER_DEG - headingPD(headingError));
      driveMotor(SPLIT_REV_SPEED, false);
      long t; noInterrupts(); t = encoderTicks; interrupts();
      if (t != backLastTicks) { backLastTicks = t; backLastMove = millis(); }
      float backed = -mmSince(phaseStartTicks);
      bool stall   = (millis() - backLastMove > BACK_STALL_MS);
      bool timeout = (millis() - phaseStartMs > BACK_TIMEOUT_MS);
      bool clean   = false;
      if (!backForPillar) { pSeen = false; pMeas = false; pRej = 0; }        // unstick: camera not read
      if (backForPillar) {
        bool s = readPillar(false);
        clean = s && pMeas && (pY + pH / 2 <= BACKUP_OK_BOTTOM);
      }
      if (backed >= backMaxMM || stall || timeout || clean) {
        driveMotor(0, true); delay(150);
        logLine(String("  back done: ") + (clean ? "clean view" : backed >= backMaxMM ? "max distance" : stall ? "stall (wall behind)" : "timeout") +
                " backed=" + String(backed, 0) + " mm h_err=" + String(headingError, 1) + " mm=" + String(mmSince(0), 0));
        enterPhase(P_NONE, "NONE");
        if (clean) acquirePillar(-headingError, 1);
      }
      return true;
    }

    case P_RECOVER:
#if SWERVE_MODE == 2
      {
        bool nextSeen = readPillar(false);                                   // next pillar in view?
        if (nextSeen && fabs(headingError) <= ACQ_STRAIGHT_DEG) { acquirePillar(-headingError, pillarCount + 1); return true; }
#if LANE_RETURN
        float arcBack = TURN_R_MM * (1 - cos(radians(RETURN_DEG)));     // lateral consumed by the arc back to lane heading
        float err = latCarMM - returnTargetLat;                          // + = still right of the aim point
        float hdg;
        if (nextSeen)                                  hdg = target_heading;   // straighten first, plan when straight
        else if (fabs(err) > arcBack + RETURN_TOL_MM)  hdg = target_heading + (err > 0 ? -RETURN_DEG : RETURN_DEG);
        else                                           hdg = target_heading;
        if (hdg >= 360) hdg -= 360; if (hdg < 0) hdg += 360;
        driveArcToHeading(hdg, currentHeading, OBST_SPEED);
        bool onLine = (fabs(latCarMM - returnTargetLat) <= RETURN_TOL_MM) && (fabs(headingError) < 5);
        if (!nextSeen && (onLine || mmSince(phaseStartTicks) >= RETURN_MAX_MM)) {
          enterPhase(P_NONE, onLine ? "NONE (back on line)" : "NONE (return timeout)");
          logLine("     carLat=" + String(latCarMM, 0) + " h_err=" + String(headingError, 1) + " mm=" + String(mmSince(0), 0));
        }
#else
        driveStraightStep(headingError, distL, distR, OBST_SPEED);         // open-challenge straight: PD + wall term
        if (!nextSeen && phaseDone(RECOVER_MS, RECOVER_MM)) enterPhase(P_NONE, "NONE");
#endif
      }
      return true;
#elif SWERVE_MODE
      driveArcToHeading(target_heading, currentHeading, OBST_SPEED);     // full arc back to lane
#else
      driveStraightStep(headingError, distL, distR, OBST_SPEED);
#endif
      if (phaseDone(RECOVER_MS, RECOVER_MM)) enterPhase(P_NONE, "NONE");
      return true;
  }
  return false;
}

const char* pStateName() {
  switch (pState) { case P_NONE: return "NONE"; case P_FAR: return "FAR"; case P_COMMIT: return "COMMIT"; case P_HOLD: return "HOLD"; case P_RECOVER: return "RECOVER";
                    case P_SCANBACK: return "SCANBACK"; case P_BACK: return "BACK"; }
  return "?";
}

// v14: forward command given but the encoder has not moved for STALL_MS (outside turns)
bool stalledForward() {
  static long lastT = 0; static unsigned long lastMove = 0; static bool wasFwd = false;
  long t; noInterrupts(); t = encoderTicks; interrupts();
  bool fwdCmd = (motorCmdSpeed > 0 && motorCmdFwd);
  if (!fwdCmd || !wasFwd || t != lastT) { lastT = t; lastMove = millis(); }
  wasFwd = fwdCmd;
  return fwdCmd && (millis() - lastMove > STALL_MS);
}

void stopRun(const String &why) {
  driveMotor(0, true); steeringServo.write(SERVO_CENTER_DEG); setNeoPixels(255, 0, 0);
  logLine("STOP " + why + " t=" + String(millis() - runStartTime) + " ms  mm=" + String(mmSince(0), 0));
  logFlush(); while (1) delay(1000);
}

// ---------- setup ----------
void setup() {
  Serial.begin(115200);
#if LOG_TO_BT
  SerialBT.begin(BT_NAME);
#endif
  pinMode(IN1_PIN, OUTPUT); pinMode(IN2_PIN, OUTPUT); pinMode(START_BTN_PIN, INPUT_PULLUP);
  pinMode(ENCODER_A_PIN, INPUT_PULLUP); pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), encoderISR, RISING);

  strip.begin(); setNeoPixels(255, 0, 0);
  Wire.begin(); Wire.setClock(100000);
  steeringServo.setPeriodHertz(50); steeringServo.attach(SERVO_PIN, 500, 2400);
  centerWheels(); driveMotor(0, true);

  while (!husky.begin(Wire)) { logLine("HuskyLens not found, retrying"); delay(500); }
  for (int i = 0; i < 3; i++) { tcaselect(i); if (!tof[i].begin()) { logLine("ToF " + String(i) + " missing"); while (1) { setNeoPixels(255,255,0); delay(100); setNeoPixels(0,0,0); delay(100); } } }
  tofStartAll();
  tcaselect(BNO_CH);
  if (!bno.begin()) { logLine("BNO055 missing"); while (1) { setNeoPixels(255,255,0); delay(100); setNeoPixels(0,0,0); delay(100); } }
  delay(100); bno.setExtCrystalUse(true);

  setNeoPixels(0, 255, 0);
  logLine(String("STEP 4 v15 FULL RUN ready (MOTOR_ON=") + MOTOR_ON + ", SWERVE_MODE=" + SWERVE_MODE + ", GATE=" + GATE_ON + ", BACKUP=" + BACKUP_ON + ", UNSTICK=" + UNSTICK_ON + ", POST_PASS=" + POST_PASS_ON + "). Press START."); logFlush();
  while (digitalRead(START_BTN_PIN) == HIGH) delay(50);
  delay(50); while (digitalRead(START_BTN_PIN) == LOW) delay(10);

  target_heading = readHeading();
  prevHeadingError = 0; prevHeadingTime = prevPillarTime = millis();
  encoderTicks = 0; latLastTicks = 0; latCarMM = 0; fwdCarMM = 0; legStartTicks = 0; runStartTime = millis();
  { float dL = getDistance(TOF_LEFT_CH), dR = getDistance(TOF_RIGHT_CH); delay(40); dL = getDistance(TOF_LEFT_CH); dR = getDistance(TOF_RIGHT_CH);
    latCarMM = laneCentreLat(dL, dR); logLine("start lane anchor carLat=" + String(latCarMM, 0) + " (L=" + String(dL, 0) + " R=" + String(dR, 0) + ")"); }
  setNeoPixels(0, 255, 255);
  logLine("heading locked " + String(target_heading, 1));
  logLine("t_ms,mm,state,F,L,R,hdg,tgt,h_err,servo,col,x,x_frac,y,bottom,h,w,blocks,carLat,turn,meas,rej,wc");
}

// ---------- loop ----------
void loop() {
  float distF = getDistance(TOF_FRONT_CH);
  float distR = getDistance(TOF_RIGHT_CH);
  float distL = getDistance(TOF_LEFT_CH);
  float currentHeading = readHeading();
  float headingError = getHeadingError(target_heading, currentHeading);
  trackLateral(-headingError);
  { static float prevF = 999;                                          // wall signature: value F had when it came in from 999
    if (prevF >= 999 && distF < 999) frontFirstSeen = distF;
    if (distF >= 999) frontFirstSeen = 999;
    prevF = distF; }                                       // car lateral offset, + right

  if (TURN >= TURNS_TO_RUN && pState == P_NONE && !isTurning && legMM() >= FINISH_MM)
    stopRun("finished: " + String(TURN) + " corners, " + String(legMM(), 0) + " mm into start section");

  // LOST watchdog: outside a turn the heading must stay within LOST_DEG of the lane
  { static bool lostLogged = false;
    if (!isTurning && fabs(headingError) > LOST_DEG) {
      if (!lostLogged) { logLine("LOST: hdg=" + String(currentHeading, 1) + " lane=" + String(target_heading, 1) + " err=" + String(headingError, 1) + " in " + pStateName() + " -> PD to lane"); lostLogged = true; }
      pState = P_NONE;
    } else if (fabs(headingError) < LOST_DEG - 10) lostLogged = false;
  }
  if (postPass && mmSince(postPassTicks) > POST_PASS_MM) endPostPass("window distance");   // v15
  if (postPass && pState != P_RECOVER && pState != P_NONE) endPostPass("left NONE/RECOVER");  // v15
  lastWallCorr = 0;                                                    // v15: logged only when used this loop
  // corner rule: active in NONE and in RECOVER (after the hold), never while a pillar is in view
  bool cornerAllowed = (pState == P_NONE || pState == P_RECOVER) && !pSeen;
  if (isTurning) {
    turnStep(headingError, currentHeading, distF, distL, distR);
  } else if (cornerAllowed && decideTurn(distF, distL, distR)) {
    if (pState != P_NONE) logLine(String("  corner during ") + pStateName() + ": pillar phase abandoned");
    pState = P_NONE; startTurn();
  } else if (!pillarStep(headingError, currentHeading, distL, distR)) {
    driveStraightStep(headingError, distL, distR, OBST_SPEED);
    if (LANE_ANCHOR_GAIN > 0) latCarMM += LANE_ANCHOR_GAIN * (laneCentreLat(distL, distR) - latCarMM);   // slow drift correction
  }

  logLine(String(millis() - runStartTime) + "," + String(mmSince(0), 0) + "," + pStateName() + "," + String(distF, 0) + "," + String(distL, 0) + "," + String(distR, 0) + "," +
          String(currentHeading, 1) + "," + String(target_heading, 1) + "," + String(headingError, 1) + "," + String(currentServoAngle) + "," +
          (pSeen ? (pColor == RED_ID ? "R" : "G") : "-") + "," +
          String(pSeen ? pX : 0) + "," + String(pSeen ? pX / (float)FRAME_W : 0, 2) + "," +
          String(pSeen ? pY : 0) + "," + String(pSeen ? pY + pH / 2 : 0) + "," +
          String(pSeen ? pH : 0) + "," + String(pSeen ? pW : 0) + "," + String(pCount) + "," + String(latCarMM, 0) + "," + (isTurning ? "T" : "") + String(TURN) + "," +
          (pSeen ? (pMeas ? "M" : "C") : "-") + "," + String(pRej) + "," + String(lastWallCorr, 1));

#if MOTOR_ON && UNSTICK_ON
  // v14: pushing but not moving (pillar, wall) outside a turn -> back off, then lane heading
  if (!isTurning && pState != P_BACK && stalledForward()) {
    logLine(String("STALL in ") + pStateName() + ": encoder frozen " + STALL_MS + " ms  F=" + String(distF, 0) + " h_err=" + String(headingError, 1));
    startBack(UNSTICK_MM, false, "unstick");
  }
#endif
#if MOTOR_ON
  if (pState == P_NONE && !isTurning && distF <= STOP_FRONT_CM && distF > 2.0)   // crash guard only
    stopRun("front " + String(distF, 0) + " cm");
#endif
  if (millis() - runStartTime > STOP_TIMEOUT_MS) stopRun("timeout");
  if (digitalRead(START_BTN_PIN) == LOW)         stopRun("button");
}
