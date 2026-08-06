#!/usr/bin/env python3
"""
CP5 — Xbox Controller + Keyboard Teleop Node (v4, heavily commented)

WHAT'S NEW vs v3: this version adds KEYBOARD control as a second input
source, running alongside (not replacing) the Xbox controller. Both
sources feed into the exact same downstream pipeline (smoothing,
integration, IK, publishing) — a keypress and a joystick nudge are just
two different ways of producing the same kind of "raw axis value," and
everything after that point doesn't care which one produced it.

BIG PICTURE — what this node is for:

  It reads controller input (Xbox controller OR keyboard, or both at once)
  and turns that into THREE separate outputs:

  1. /cmd_vel (geometry_msgs/Twist) — "how fast do you want to walk/turn
     right now." DIRECT: whatever's being pressed/pushed right now IS the
     answer, no memory of a moment ago. Release everything, this becomes
     zero immediately. Consumed by CP6's gait generator.

  2. /body_pose (std_msgs/Float64MultiArray) — a "desired body pose" that
     IS remembered over time (dx, dy, dz, roll, pitch, yaw). Whatever pose
     you last set stays set, forever, until changed again or reset with
     the neutral key/button. Consumed by CP6's gait generator, which
     combines it with the current gait offset before solving IK.

  3. /gait_mode (std_msgs/String) — "trot" or "crawl", toggled by the X
     button. Consumed by CP6's gait generator to pick which gait to run.

  4. /step_height (std_msgs/Float64) — how high a swinging foot lifts
     during gait, D-pad up/down. Independent of body pose; clamped to
     [0.0, 0.08] meters. Consumed by CP6's gait generator.

  ARCHITECTURE CHANGE AS OF CP6 (read this if comparing to CP5's version):
  Earlier (CP5), this node computed FINAL joint angles itself (calling
  body_pose_to_joint_angles() and publishing straight to /joint_command).
  That stopped being correct once CP6 needed to ADD a time-varying gait
  offset on top of the pose before solving IK — two nodes both trying to
  own /joint_command would fight over it. So as of CP6: this node now
  publishes only the RAW pose request (/body_pose) and no longer touches
  /joint_command at all. The CP6 gait node is the sole owner of
  /joint_command now. This node still runs IK internally as a
  reachability CHECK (to preserve the "stop exactly at the limit" clamp
  behavior) but throws the resulting angles away rather than publishing
  them — see the end of control_loop() for the reasoning.

ARCHITECTURE CHANGE AS OF CP4 (read this if comparing to CP6's version):
  This version adds a BASE/ARM mode toggle (button 7, edge-triggered,
  confirmed via joy_mapping_sniffer.py -- the controller's AGR/AGL grip
  buttons do NOT appear on /joy at all under joy_node, so they were not
  usable). While in "base" mode, this node behaves exactly as it did in
  CP6 -- all four outputs above are unchanged. While in "arm" mode:
    - /cmd_vel is published as all-zero, continuously (base holds still)
    - /body_pose is republished continuously with self.pose UNCHANGED
      (base holds its exact last pose -- pitch/roll/height/yaw all frozen)
    - /gait_mode and /step_height are also republished unchanged --
      button 7 fully freezes the base, no other button does anything to
      it while in arm mode
    - the sticks/triggers/bumpers instead drive TWO NEW outputs:
      5. /arm_target (Float64MultiArray, order [dx, dy, dz, pitch,
         wrist_roll]) -- dx/dy/dz are OFFSETS from whatever "home"
         end-effector pose a downstream, not-yet-built arm-IK node
         defines (same convention as /body_pose's dx/dy/dz). pitch and
         wrist_roll are likewise offsets from a home orientation, NOT
         absolute angles. This is now a FULLY DETERMINED system: 5
         positioning joints (shoulder_pan, shoulder_lift, elbow_flex,
         wrist_flex, wrist_roll), 5 targets (x, y, z, pitch, wrist_roll)
         -- no leftover unconstrained DOF, unlike an earlier draft of
         this design that only specified x/y/z.
           Left stick fwd/back  = dz (vertical)
           Right stick left/right = dy (sideways)
           Right stick fwd/back   = dx (in/out)
           RT (held, depth) = pitch down / LT (held, depth) = pitch up
           LB (held) = rotate gripper left / RB (held) = rotate gripper right
         All rate-based: hold to move, release and it holds put -- same
         exp_smooth-then-integrate pattern as body pose. RT/LT/LB/RB mean
         body pitch/roll in base mode and arm pitch/gripper-rotation in
         arm mode -- same button, different meaning depending on mode,
         same as A/B's dual meaning (height vs. gripper open/close).
      6. /gripper_target (Float64, radians) -- A held = opening, B held
         = closing, rate-based, clamped to the SO-101 URDF's real
         `gripper` joint limits ([-0.174533, 1.74533] rad -- sourced
         directly from so101_new_calib.urdf, not guessed).
    Y-BUTTON RESET IS MODE-SCOPED, not shared (Harsh's explicit choice):
    in base mode, Y still does exactly what it always did (smooth reset
    of self.pose only). In arm mode, Y instead smoothly resets
    self.arm_target (dx/dy/dz/pitch/wrist_roll) to neutral over the same
    RESET_DURATION/smoothstep easing, completely independent reset state
    from body pose's -- pressing Y in one mode does NOT touch the other
    mode's pose. The gripper JAW (/gripper_target, A/B) is untouched by
    the arm-mode reset either way -- wrist_roll (LB/RB) DOES reset, per
    Harsh's explicit distinction between "the jaw" and "the gripper's
    rotation."
    This node does NOT solve arm IK itself and does NOT touch
    /joint_command for the arm -- same CP6 reasoning (avoid two owners
    fighting over one topic) applies here. A separate downstream node
    (not yet built) will consume /arm_target + /gripper_target, solve
    IK, and publish JointState with arm joint names to the EXISTING
    /joint_command topic (confirmed live: the arm's joints are part of
    the same merged 21-joint articulation as the legs, so no new
    OmniGraph was needed for /joint_command to reach them).
    NOTE (unconfirmed): the sign of EVERY new arm axis (dx/dy/dz, pitch,
    wrist_roll -- does RT mean pitch up or down, does LB rotate left or
    right in the direction you'd expect) has NOT been verified live yet
    -- same category of caveat as AX_DPAD_Y's polarity below. Test once
    the downstream arm node exists to actually see motion; flip the
    sign at the point each raw_arm_* value is read in control_loop() if
    backward, don't guess here.

WHY KEYBOARD NEEDS A DIFFERENT APPROACH THAN THE JOYSTICK:
  A joystick's /joy message directly tells us "how far is this control
  currently deflected," continuously, in real time — including telling us
  the exact instant something is released (it just goes back to 0/idle).
  A keyboard has no such thing built in. A terminal only ever tells us
  "this character arrived," with no matching "this key was released"
  event at all. So we have to FAKE "currently held" ourselves: when a key
  arrives, we remember "I saw this key just now." As long as you keep
  physically holding a key down, your OS's own keyboard auto-repeat
  feature keeps re-sending that same character every so often (this is
  the exact same repeat behavior you see if you hold a letter key down
  while typing in any text editor) — so as long as new copies of that
  character keep arriving faster than our timeout, we correctly treat it
  as "still held." The moment you actually release it, no more repeats
  arrive, our timeout expires shortly after, and we correctly treat it as
  "released." This is a standard trick (the same one tools like
  teleop_twist_keyboard use), not something unique to this file.

HOW KEYBOARD INPUT REACHES THIS CODE AT ALL:
  Normally, a terminal only hands your program a full LINE of text once
  you press Enter (this is called "line-buffered" or "canonical" mode) —
  that's obviously useless for real-time control. This node switches the
  terminal into "cbreak mode" instead (via Python's tty/termios modules),
  which hands us each individual keypress the instant it happens, with no
  Enter required, and without echoing the character back to the screen.
  We restore the terminal to its normal behavior when the node exits, so
  your terminal isn't left in a weird state afterward.
"""

import sys
import os
import time
import select
import termios
import tty
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String, Float64

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pose_controller import body_pose_to_joint_angles
from ik_leg import UnreachableTargetError

# =============================================================================
# CONFIRMED /joy MAPPING (from CP5a) — unchanged from v3.
# =============================================================================
AX_LEFT_X   = 0
AX_LEFT_Y   = 1
AX_LT       = 2
AX_RIGHT_X  = 3
AX_RIGHT_Y  = 4
AX_RT       = 5
AX_DPAD_X   = 6
# NEW: D-pad vertical axis, for step-height control. UNLIKE AX_DPAD_X
# (whose sign was empirically confirmed working correctly for standing
# yaw), this axis's polarity has NOT been tested yet -- standard joy
# drivers often report D-pad-up as +1, but this varies. If up/down feel
# backward once tested, flip the sign at the one place raw_step_height
# is read below, rather than guessing here.
AX_DPAD_Y   = 7

BTN_A  = 0
BTN_B  = 1
BTN_Y  = 3
BTN_LB = 4
BTN_RB = 5

# NEW (CP6): standard Xbox controller layout has X at index 2 (A=0, B=1,
# X=2, Y=3). UNLIKE the buttons above, this one was never empirically
# confirmed the way CP5a confirmed A/B/Y/LB/RB (X was unused until now,
# so there was nothing to test). Worth a quick real-controller check
# (e.g. temporarily log buttons[2] while pressing X) before fully
# trusting this — same standard CP5a itself set for every mapping here.
BTN_X = 2

# =============================================================================
# NEW (CP4): ARM MODE -- mode-toggle button + arm/gripper control constants.
# =============================================================================
BTN_MODE_TOGGLE = 7   # center-tab button, confirmed via joy_mapping_sniffer.py
                       # (AGR/AGL produced no /joy event at all -- not usable).
                       # Edge-triggered toggle between "base" and "arm" mode,
                       # same pattern as BTN_X's gait-mode toggle below.

ARM_TARGET_RATE = 0.03   # m/s per axis while stick held -- STARTING POINT,
                          # same magnitude as HEIGHT_RATE, not yet tuned live.
# REPLACES the earlier single symmetric ARM_TARGET_LIMIT placeholder.
# These 6 bounds are GROUNDED in a real reachability sweep run against
# ik_arm.py (the actual verified IK, not a guess) -- swept each direction
# independently (other offsets held at 0, pitch=0) to find where
# ArmUnreachableTargetError starts firing, then pulled in by a small
# safety margin (not run right up to the exact numerical boundary, which
# can be fragile/flickery near a solver's tolerance). Real swept limits
# were: dx [-0.200, +0.060], dy [-0.210, +0.210], dz [-0.280, +0.070] --
# NOTE the real workspace is quite lopsided (e.g. +dx only reaches 0.06m
# while -dz reaches 0.28m) -- the old single symmetric +/-0.10m clamp was
# simultaneously too TIGHT on -dz (Harsh's reported problem: couldn't
# reach down far enough to pick something off the ground) and too LOOSE
# on +dx (would have let a target get commanded into an area the real IK
# can't reach, triggering fallback right at the edge).
ARM_DX_MIN, ARM_DX_MAX = -0.18, 0.05
ARM_DY_MIN, ARM_DY_MAX = -0.19, 0.19
# UPDATED (this chat, live feedback: "needs to reach further down"): the
# ORIGINAL sweep behind -0.25 only varied dz in isolation (dx=0, pitch=0)
# and found ~-0.28m. That undersold the real reach -- a SECOND sweep,
# combining dz with dx AND pitch together (all three are independently
# stick/trigger-controllable at the same time during actual teleop, so
# this isn't a hypothetical), found comfortable, safely-margined
# reachability down to dz=-0.34m when paired with dx around -0.15m and
# pitch around +0.75 rad (both well within THEIR OWN existing clamps).
# Verified: that combined target solves with ~0mm error, not at the edge
# of what's possible (true combined optimum found was closer to -0.40m).
ARM_DZ_MIN, ARM_DZ_MAX = -0.34, 0.06

GRIPPER_RATE = 1.0       # rad/s while A/B held -- starting point, not tuned.
# Real, URDF-sourced limits for the `gripper` revolute joint
# (so101_new_calib.urdf) -- NOT guessed.
GRIPPER_MIN = -0.174533  # radians (closed end, per URDF <limit lower=...>)
GRIPPER_MAX = 1.74533    # radians (open end, per URDF <limit upper=...>)
GRIPPER_DEFAULT = 0.0    # starting value on node startup -- within the valid
                          # range, but NOT verified as the arm's true at-rest
                          # servo position.

ARM_PITCH_RATE = 0.15    # rad/s while RT/LT held -- starting point, same
                          # magnitude as PITCH_RATE_MAX (body pitch).
# Clamp is the REAL `wrist_flex` joint's URDF limit (so101_new_calib.urdf),
# not guessed -- but note this is a SINGLE-joint limit, not a verified
# true end-effector-pitch range once the full 5-joint IK chain is
# involved. Reasonable physical ceiling for now; revisit once the arm-IK
# node exists and can report what's actually achievable.
ARM_PITCH_LIMIT = 1.65806  # radians, symmetric +/-

ARM_GRIPPER_ROT_RATE = 0.15  # rad/s while LB/RB held -- starting point,
                              # same magnitude as ROLL_RATE (body roll).
# Real, URDF-sourced limits for the `wrist_roll` joint -- NOT guessed.
# Asymmetric range, straight from <limit lower=.../upper=...>.
ARM_GRIPPER_ROT_MIN = -2.74385  # radians
ARM_GRIPPER_ROT_MAX = 2.84121   # radians

# =============================================================================
# KEYBOARD MAPPING (new in v4)
#
# WHY THESE SPECIFIC KEYS: chosen to avoid Isaac Sim / Omniverse Kit's own
# default viewport shortcuts (W/A/S/D fly-camera movement, Q/E fly
# up/down, W/E/R as move/rotate/scale gizmo tools, F to focus/frame the
# selected object, number keys and Space for various viewport actions).
# WORTH KNOWING: a terminal and the Isaac Sim window only ever receive
# keyboard input when THEY individually have OS focus — so there isn't
# actually a way for a keypress to go to both at once, regardless of
# which keys we pick. The real benefit of avoiding Isaac's own shortcuts
# is just not fighting your own muscle memory if you're switching focus
# between the two windows a lot — not a technical conflict being
# "prevented," since there wasn't a real conflict possible either way.
# =============================================================================
KEY_FORWARD       = 'i'   # velocity: forward
KEY_BACKWARD      = 'k'   # velocity: backward
KEY_STRAFE_LEFT   = 'j'   # velocity: strafe left
KEY_STRAFE_RIGHT  = 'l'   # velocity: strafe right
KEY_TURN_LEFT     = 'u'   # velocity: stepping-yaw left
KEY_TURN_RIGHT    = 'o'   # velocity: stepping-yaw right

KEY_PITCH_FWD     = 't'   # pose: pitch forward
KEY_PITCH_BACK    = 'g'   # pose: pitch back
KEY_ROLL_LEFT     = 'h'   # pose: roll left
KEY_ROLL_RIGHT    = 'y'   # pose: roll right
KEY_HEIGHT_UP     = 'p'   # pose: height up
KEY_HEIGHT_DOWN   = 'm'   # pose: height down
KEY_YAW_LEFT      = 'z'   # pose: standing yaw left
KEY_YAW_RIGHT     = 'x'   # pose: standing yaw right
KEY_RESET         = 'n'   # pose: smooth return to neutral (like the Y button)

# How long (seconds) after the LAST time we saw a key's character arrive
# before we consider it "released." Must be longer than the gap between
# repeat-characters from a genuinely held key (OS repeat rates are
# typically much faster than this once repeating starts), but short
# enough that letting go feels responsive rather than sticky.
KEY_HOLD_TIMEOUT = 0.2

# =============================================================================
# TUNABLE CONSTANTS (unchanged from v3)
# =============================================================================
STICK_DEADZONE    = 0.15
TRIGGER_DEADZONE  = 0.05

CONTROL_RATE_HZ   = 70.0
DT                = 1.0 / CONTROL_RATE_HZ

MAX_LINEAR_SPEED   = 0.15
MAX_ANGULAR_SPEED  = 0.6

HEIGHT_RATE    = 0.03
ROLL_RATE      = 0.15
YAW_RATE       = 0.15
PITCH_RATE_MAX = 0.15

# NEW: step-height (gait swing lift amount) control, D-pad up/down.
STEP_HEIGHT_RATE = 0.03    # m/s while held -- same magnitude as HEIGHT_RATE
STEP_HEIGHT_MIN  = 0.0     # meters -- CANNOT go below 0 (a negative lift is meaningless)
STEP_HEIGHT_MAX  = 0.08    # meters -- upper bound, per Harsh's explicit request
STEP_HEIGHT_DEFAULT = 0.02 # meters -- matches the gait node's original fixed value,
                            # so behavior on startup (before any D-pad input) is
                            # unchanged from what's already been tuned/confirmed working

RATE_SMOOTHING_TAU = 0.15
RESET_DURATION = 1.5

WARN_THROTTLE_SEC = 1.0


def apply_deadzone(value, deadzone):
    """Below the deadzone, treat as exactly zero (see v3 for full
    reasoning — unchanged here)."""
    return 0.0 if abs(value) < deadzone else value


def trigger_depth(axis_value, deadzone=TRIGGER_DEADZONE):
    """Converts a trigger's raw +1.0(idle)/-1.0(full pull) reading into a
    clean 0.0-to-1.0 'how far pressed' value (see v3 for full reasoning —
    unchanged here)."""
    depth = (1.0 - axis_value) / 2.0
    return depth if depth > deadzone else 0.0


def smoothstep(t):
    """Eases progress through [0,1] so motion starts and stops gently
    instead of at constant speed (see v3 for full reasoning — unchanged
    here)."""
    t = max(0.0, min(1.0, t))
    return t * t * (3 - 2 * t)


def exp_smooth(current, target, dt, tau):
    """One step of exponential smoothing toward `target` (see v3 for full
    reasoning — unchanged here)."""
    alpha = min(1.0, dt / tau)
    return current + (target - current) * alpha


def clip(value, lo=-1.0, hi=1.0):
    """Keeps a combined joystick+keyboard value inside [-1, 1]. Needed
    because v4 lets BOTH input sources contribute to the same axis at
    once (e.g. you could technically hold a trigger AND a keyboard key
    for the same pose axis simultaneously) — without this, the two
    contributions could add up past what any single source could ever
    produce on its own, which would behave strangely (e.g. faster-than-
    max-speed motion)."""
    return max(lo, min(hi, value))


class XboxTeleopNode(Node):
    """
    Main node class. Combines TWO input sources — the Xbox controller
    (via /joy) and the keyboard (via raw terminal input) — into the same
    single pipeline: compute raw axis values -> smooth -> integrate pose
    -> solve IK -> publish. Everything from "compute raw axis values"
    onward doesn't know or care whether a given axis's value came from
    the joystick, the keyboard, or (in principle) both at once.
    """

    def __init__(self):
        super().__init__('xbox_teleop_node')

        # ---------------------------------------------------------------
        # PERSISTENT POSE STATE. Same invariant as v3: self.pose is
        # ALWAYS a pose that has already been confirmed reachable by IK.
        # We only ever build a separate `candidate` and commit it here
        # AFTER a successful solve — see control_loop() below.
        # ---------------------------------------------------------------
        self.pose = dict(dx=0.0, dy=0.0, dz=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        # (self.last_angles removed as of CP6 -- this node no longer
        # computes or tracks final joint angles at all, only the raw
        # pose. The CP6 gait node is now the sole thing that knows/cares
        # about actual servo angles.)

        # Smoothing filter state, one per pose axis (unchanged from v3).
        self.filt_pitch = 0.0
        self.filt_roll = 0.0
        self.filt_height = 0.0
        # NEW: step-height control. Uses the SAME smoothed-rate-then-
        # integrate pattern as filt_height/candidate['dz'] above -- a
        # separate value, not reusing filt_height, since this controls
        # GAIT SWING LIFT (published on /step_height, consumed by the
        # gait node), not body pose height (which is what filt_height/dz
        # already controls). These are two genuinely different things
        # that happen to both be "vertical."
        self.filt_step_height_rate = 0.0
        self.step_height = STEP_HEIGHT_DEFAULT
        self.filt_yaw = 0.0

        # Y/N-button reset state (unchanged from v3).
        self.reset_active = False
        self.reset_start_pose = None
        self.reset_start_time = None

        # Joystick state.
        self.latest_joy = None
        self.prev_y_button = 0
        self.prev_x_button = 0   # NEW (CP6): edge-detection for the gait-switch button

        # NEW (CP6): which gait the robot should currently be running.
        # Lives here (not in the gait node) because "which gait" is a
        # teleop INPUT decision -- same reasoning as Y-button reset
        # already living here rather than downstream. Defaults to "trot".
        self.gait_mode = "trot"

        # ---------------------------------------------------------------
        # NEW (CP4): base/arm mode toggle state.
        # ---------------------------------------------------------------
        self.control_mode = "base"     # "base" or "arm"
        self.prev_mode_button = 0      # edge-detection, same pattern as prev_x_button

        # NEW (CP4): persistent arm target. dx/dy/dz are OFFSETS from
        # whatever "home" end-effector pose the (not-yet-built) downstream
        # arm-IK node defines -- same convention as self.pose's dx/dy/dz,
        # NOT absolute world coordinates. pitch/wrist_roll are likewise
        # offsets from a home orientation. Fully determined: 5 positioning
        # joints, 5 targets here.
        self.arm_target = dict(dx=0.0, dy=0.0, dz=0.0, pitch=0.0, wrist_roll=0.0)
        self.filt_arm_dx = 0.0
        self.filt_arm_dy = 0.0
        self.filt_arm_dz = 0.0
        self.filt_arm_pitch = 0.0
        self.filt_arm_wrist_roll = 0.0

        # NEW (CP4): persistent gripper target, in radians, matching the
        # real `gripper` joint's URDF range directly -- no 0-1
        # normalization, so a downstream consumer can use this value as-is.
        self.gripper_target = GRIPPER_DEFAULT
        self.filt_gripper_rate = 0.0

        # NEW (CP4, arm reset): mirrors the body pose's Y-button reset
        # state machine (self.reset_active/reset_start_pose/reset_start_time
        # below), but for self.arm_target instead. Kept COMPLETELY
        # SEPARATE from the body pose reset -- Y means something different
        # depending on mode (Harsh's explicit choice), not one shared
        # "reset everything" action.
        self.arm_reset_active = False
        self.arm_reset_start_target = None
        self.arm_reset_start_time = None

        # ---------------------------------------------------------------
        # KEYBOARD STATE (new in v4).
        #
        # key_last_seen maps a character (e.g. 'i') to the monotonic
        # timestamp of the last time we saw it arrive from the terminal
        # (including OS auto-repeat copies while held). A key counts as
        # "currently held" if that timestamp is more recent than
        # KEY_HOLD_TIMEOUT seconds ago — see is_key_active() below.
        # ---------------------------------------------------------------
        self.key_last_seen = {}

        # prev_key state for the RESET key specifically, so we can
        # edge-detect it (trigger once per press) the same way the Y
        # button is edge-detected, rather than re-triggering every tick
        # for as long as it's technically still "seen as held" by our
        # timeout logic.
        self.prev_reset_active_key = False

        self.last_warn_time = 0.0

        # ---------------------------------------------------------------
        # TERMINAL SETUP FOR RAW KEYBOARD READING (new in v4).
        # ---------------------------------------------------------------
        # sys.stdin's file descriptor -- we need this to both reconfigure
        # the terminal AND to check/read from it directly later.
        self.stdin_fd = sys.stdin.fileno()

        # Save the terminal's CURRENT settings before we change anything,
        # so we can put them back exactly as they were when this node
        # shuts down. Skipping this would leave your terminal in cbreak
        # mode (no line buffering, no echo) even after the script exits,
        # which would make your terminal feel broken until you ran
        # `reset` or opened a new tab.
        self.old_termios_settings = termios.tcgetattr(self.stdin_fd)

        # Switch the terminal into "cbreak" mode: characters become
        # available to read ONE AT A TIME, immediately, without waiting
        # for Enter, and without being echoed back to the screen. (cbreak
        # leaves Ctrl+C's normal behavior -- raising KeyboardInterrupt --
        # intact, unlike the more extreme "raw" mode would.)
        tty.setcbreak(self.stdin_fd)

        # ---------------------------------------------------------------
        # ROS2 PLUMBING (unchanged from v3).
        # ---------------------------------------------------------------
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # CP6 CHANGE: this node no longer owns /joint_command -- the CP6
        # gait node does, since it's the one that knows how to combine
        # this pose with the current gait offset before solving IK. This
        # node now only publishes the RAW pose request.
        self.body_pose_pub = self.create_publisher(Float64MultiArray, '/body_pose', 10)
        # NEW (CP6): which gait is currently active, so the gait node
        # knows what to run.
        self.gait_mode_pub = self.create_publisher(String, '/gait_mode', 10)
        # NEW: gait swing-lift height, D-pad up/down controlled.
        self.step_height_pub = self.create_publisher(Float64, '/step_height', 10)
        # NEW (CP4): raw arm target request -- mirrors /body_pose's role and
        # message type (Float64MultiArray, order convention [dx, dy, dz],
        # same "no field names, this is a hand-shake" caveat as
        # publish_body_pose()).
        self.arm_target_pub = self.create_publisher(Float64MultiArray, '/arm_target', 10)
        # NEW (CP4): raw gripper target request. Separate topic -- same
        # reasoning as /step_height being kept separate from /body_pose.
        self.gripper_target_pub = self.create_publisher(Float64, '/gripper_target', 10)
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            f"Xbox + Keyboard teleop node started ({CONTROL_RATE_HZ} Hz control loop).")
        self.get_logger().info(
            "Keyboard active in THIS terminal window (click here to send key "
            "commands). Isaac Sim's own shortcuts are unaffected -- each "
            "window only receives keys while it has focus.")
        self.get_logger().info(
            "Keys: I/K=fwd/back  J/L=strafe  U/O=turn  T/G=pitch  H/Y=roll  "
            "P/M=height  Z/X=stand-yaw  N=neutral reset")
        self.get_logger().info(
            "Controller X button (BTN_X) = toggle gait mode (trot <-> crawl). "
            "No keyboard equivalent yet.")
        self.get_logger().info(
            "Controller D-pad UP/DOWN = step height (0.0 - 0.08m). "
            "No keyboard equivalent yet.")
        self.get_logger().info(
            "Controller button 7 (center tab) = toggle BASE/ARM mode. "
            "In ARM mode: left stick fwd/back = vertical, right stick "
            "left/right = sideways, right stick fwd/back = in/out, "
            "RT/LT held = pitch down/up, LB/RB held = rotate gripper "
            "left/right, A/B held = gripper open/close. Base fully "
            "freezes while in ARM mode. No keyboard equivalent yet.")

    def restore_terminal(self):
        """
        Puts the terminal back to its normal (line-buffered, echoing)
        mode. MUST be called before the process exits, or the terminal
        will be left in cbreak mode afterward. Called from main()'s
        `finally` block, guaranteeing it runs even if the node exits via
        Ctrl+C or an exception.
        """
        termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.old_termios_settings)

    def read_available_keys(self):
        """
        Non-blocking check: "is there any keyboard input waiting to be
        read RIGHT NOW, and if so, what is it?" Called once per control
        loop tick.

        select.select([self.stdin_fd], [], [], 0) asks the operating
        system "is stdin ready to be read from, checking right now, don't
        wait at all (timeout=0)?" This is what makes it non-blocking --
        without this check, calling a plain read on stdin would FREEZE
        this function (and therefore the entire node, since everything
        runs on one thread) until a key was actually pressed, which would
        break the fixed-rate control loop entirely.

        We loop reading one byte at a time for as long as select keeps
        saying "yes, more is available" -- this drains ALL pending
        keypresses this tick (in case multiple arrived between one tick
        and the next), rather than only ever processing one per tick.

        For every character read, we stamp key_last_seen[char] with the
        current time -- this is the entire mechanism that lets
        is_key_active() later answer "is this key currently held."
        """
        now = time.monotonic()
        while select.select([self.stdin_fd], [], [], 0)[0]:
            ch = os.read(self.stdin_fd, 1).decode(errors='ignore').lower()
            if ch:
                self.key_last_seen[ch] = now

    def is_key_active(self, key):
        """
        Returns True if `key` was seen recently enough (within
        KEY_HOLD_TIMEOUT seconds) to be considered "currently held."
        Returns False for a key that's never been pressed at all (not in
        the dict yet) or one that hasn't been seen recently (i.e. was
        released and enough time has passed).
        """
        last_seen = self.key_last_seen.get(key, 0.0)
        return (time.monotonic() - last_seen) < KEY_HOLD_TIMEOUT

    def joy_callback(self, msg):
        """Stashes the latest /joy message; actual computation happens in
        control_loop() (unchanged from v3)."""
        self.latest_joy = msg

    def warn_throttled(self, text):
        """Prints `text` as a warning, but no more often than once every
        WARN_THROTTLE_SEC seconds (unchanged from v3)."""
        now = time.monotonic()
        if now - self.last_warn_time >= WARN_THROTTLE_SEC:
            self.get_logger().warn(text)
            self.last_warn_time = now

    def publish_body_pose(self, pose):
        """
        NEW (CP6, replaces v3/v4's publish_angles()).

        Publishes the current pose request as a Float64MultiArray, in a
        FIXED order: [dx, dy, dz, roll, pitch, yaw].

        IMPORTANT TRADEOFF: unlike the old JointState message (which
        names each joint explicitly, so a subscriber can't misread which
        number means what), this array has NO field names. The order is
        a convention this publisher and the CP6 gait node's subscriber
        must agree on by hand. If that order ever gets out of sync
        between the two files, this fails SILENTLY (wrong numbers, no
        error) rather than loudly. Accepting this for now rather than
        building a custom .msg package -- flagged here deliberately so
        it isn't forgotten, not because it's a good long-term design.
        """
        msg = Float64MultiArray()
        msg.data = [pose['dx'], pose['dy'], pose['dz'],
                    pose['roll'], pose['pitch'], pose['yaw']]
        self.body_pose_pub.publish(msg)

    def control_loop(self):
        """
        Main logic, called every DT seconds (~33ms). Reads BOTH input
        sources (joystick + keyboard), combines them into one set of raw
        axis values per control, then runs the exact same pipeline v3
        used from that point onward.
        """
        # --- Step 0: drain any pending keyboard input for this tick ---
        # Always do this, even if no /joy message has ever arrived --
        # keyboard-only operation (no Xbox controller plugged in at all)
        # should work fine.
        self.read_available_keys()

        # If we don't have a /joy message yet, treat all joystick-derived
        # values as neutral/idle defaults rather than crashing -- this
        # lets the node run keyboard-only, with no controller connected.
        if self.latest_joy is not None:
            axes = self.latest_joy.axes
            buttons = self.latest_joy.buttons
        else:
            axes = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]  # LT/RT idle at +1.0, rest at 0.0
            buttons = [0, 0, 0, 0, 0, 0, 0, 0]  # CP4: grew from 6 to 8 elements
            # (was only 6 -- reading buttons[BTN_MODE_TOGGLE] (index 7) below
            # against the old 6-element fallback would have raised an
            # IndexError the instant this node ran with no controller
            # plugged in, keyboard-only. Fixed here rather than special-
            # casing the mode-toggle read.)

        # =====================================================================
        # PART 0.5 (NEW, CP4): BASE/ARM MODE TOGGLE -- button 7, edge-triggered.
        # =====================================================================
        # Runs EVERY tick regardless of current mode -- this is the one
        # piece of logic that must never itself be gated by the mode it's
        # switching. Guarded with len(buttons) in case a /joy message ever
        # arrives with fewer than 8 buttons (some drivers/controllers
        # report varying counts) -- defaults to "not pressed" rather than
        # crashing.
        mode_btn_now = buttons[BTN_MODE_TOGGLE] if len(buttons) > BTN_MODE_TOGGLE else 0
        if mode_btn_now == 1 and self.prev_mode_button == 0:
            self.control_mode = "arm" if self.control_mode == "base" else "base"
            self.get_logger().info(f"Control mode switched -> {self.control_mode}")
        self.prev_mode_button = mode_btn_now

        if self.control_mode == "arm":
            # =================================================================
            # ARM MODE (NEW, CP4) -- base fully frozen; sticks/A/B drive the
            # arm target + gripper instead. See module docstring's "CP4"
            # section for the full design rationale.
            # =================================================================

            # ---- Base freeze: every base output still republished every
            # tick (same continuous-publish convention as always), just
            # held at zero (/cmd_vel) or at its last unchanged value
            # (everything else) rather than driven by stick/button input.
            self.cmd_vel_pub.publish(Twist())  # all fields default 0.0

            self.publish_body_pose(self.pose)  # self.pose untouched this tick

            gait_msg = String()
            gait_msg.data = self.gait_mode
            self.gait_mode_pub.publish(gait_msg)

            step_height_msg = Float64()
            step_height_msg.data = self.step_height
            self.step_height_pub.publish(step_height_msg)

            # ---- Arm target: rate-based, offsets from a downstream-defined
            # "home" pose. Signs below are UNCONFIRMED live -- see module
            # docstring's CP4 note. Flip here if a stick moves the arm
            # backward from what's expected, once there's something
            # downstream actually moving to check against.
            raw_arm_dz = apply_deadzone(axes[AX_LEFT_Y], STICK_DEADZONE)   # vertical
            raw_arm_dy = apply_deadzone(axes[AX_RIGHT_X], STICK_DEADZONE)  # sideways
            raw_arm_dx = apply_deadzone(axes[AX_RIGHT_Y], STICK_DEADZONE)  # in/out

            rt_depth = trigger_depth(axes[AX_RT])
            lt_depth = trigger_depth(axes[AX_LT])
            raw_arm_pitch = clip(lt_depth - rt_depth)
            # ^ NOTE: sign UNCONFIRMED live -- see module docstring's CP4
            # note. If RT ends up pitching up instead of down, negate
            # this line.

            lb_held = buttons[BTN_LB] if len(buttons) > BTN_LB else 0
            rb_held = buttons[BTN_RB] if len(buttons) > BTN_RB else 0
            raw_arm_wrist_roll = float(lb_held - rb_held)
            # ^ NOTE: sign UNCONFIRMED live -- see module docstring's CP4
            # note. If LB ends up rotating right instead of left, negate
            # this line.

            any_arm_input = (raw_arm_dz != 0.0 or raw_arm_dy != 0.0
                              or raw_arm_dx != 0.0 or abs(raw_arm_pitch) > 0
                              or raw_arm_wrist_roll != 0.0)

            # ---- NEW: Y-button reset, ARM MODE MEANING (mirrors body
            # pose's reset state machine below, but scoped to arm_target
            # ONLY -- self.gripper_target is untouched, deliberately, per
            # Harsh's explicit "gripper's pose" = jaw only, wrist_roll DOES
            # reset). Completely separate reset_active flag/state from
            # body pose's -- Y means a different thing in each mode, not
            # one shared "reset everything" action.
            y_now = buttons[BTN_Y] if len(buttons) > BTN_Y else 0
            arm_reset_pressed_this_tick = (y_now == 1 and self.prev_y_button == 0)
            if arm_reset_pressed_this_tick and not self.arm_reset_active:
                self.arm_reset_active = True
                self.arm_reset_start_target = dict(self.arm_target)
                self.arm_reset_start_time = time.monotonic()
                self.get_logger().info(
                    "Arm reset triggered -- smoothly returning to neutral "
                    "(gripper jaw untouched).")
            self.prev_y_button = y_now

            candidate = dict(self.arm_target)

            if self.arm_reset_active:
                if any_arm_input:
                    self.arm_reset_active = False
                    self.get_logger().info("Arm reset interrupted by new input.")
                else:
                    elapsed = time.monotonic() - self.arm_reset_start_time
                    if elapsed >= RESET_DURATION:
                        candidate = dict(dx=0.0, dy=0.0, dz=0.0,
                                          pitch=0.0, wrist_roll=0.0)
                        self.arm_reset_active = False
                    else:
                        ease = smoothstep(elapsed / RESET_DURATION)
                        candidate = {
                            k: v * (1.0 - ease)
                            for k, v in self.arm_reset_start_target.items()
                        }

            if not self.arm_reset_active:
                self.filt_arm_dz = exp_smooth(self.filt_arm_dz, raw_arm_dz, DT, RATE_SMOOTHING_TAU)
                self.filt_arm_dy = exp_smooth(self.filt_arm_dy, raw_arm_dy, DT, RATE_SMOOTHING_TAU)
                self.filt_arm_dx = exp_smooth(self.filt_arm_dx, raw_arm_dx, DT, RATE_SMOOTHING_TAU)
                self.filt_arm_pitch = exp_smooth(
                    self.filt_arm_pitch, raw_arm_pitch, DT, RATE_SMOOTHING_TAU)
                self.filt_arm_wrist_roll = exp_smooth(
                    self.filt_arm_wrist_roll, raw_arm_wrist_roll, DT, RATE_SMOOTHING_TAU)

                candidate['dz'] += self.filt_arm_dz * ARM_TARGET_RATE * DT
                candidate['dy'] += self.filt_arm_dy * ARM_TARGET_RATE * DT
                candidate['dx'] += self.filt_arm_dx * ARM_TARGET_RATE * DT
                candidate['pitch'] += self.filt_arm_pitch * ARM_PITCH_RATE * DT
                candidate['wrist_roll'] += (
                    self.filt_arm_wrist_roll * ARM_GRIPPER_ROT_RATE * DT)

                for axis, lo, hi in (('dx', ARM_DX_MIN, ARM_DX_MAX),
                                     ('dy', ARM_DY_MIN, ARM_DY_MAX),
                                     ('dz', ARM_DZ_MIN, ARM_DZ_MAX)):
                    candidate[axis] = max(lo, min(hi, candidate[axis]))
                candidate['pitch'] = max(-ARM_PITCH_LIMIT,
                                          min(ARM_PITCH_LIMIT, candidate['pitch']))
                candidate['wrist_roll'] = max(
                    ARM_GRIPPER_ROT_MIN, min(ARM_GRIPPER_ROT_MAX, candidate['wrist_roll']))
            # NOTE: while self.arm_reset_active IS easing toward zero, no
            # clamp is applied here -- unnecessary, since the eased value
            # is always a blend between an already-in-bounds start value
            # and zero (also in-bounds), and that range stays within
            # bounds throughout for every axis here (each bound interval
            # contains zero) -- same reasoning as why body pose's own
            # reset arc doesn't need reachability-checking mid-ease.

            self.arm_target = candidate

            arm_target_msg = Float64MultiArray()
            arm_target_msg.data = [self.arm_target['dx'],
                                    self.arm_target['dy'],
                                    self.arm_target['dz'],
                                    self.arm_target['pitch'],
                                    self.arm_target['wrist_roll']]
            self.arm_target_pub.publish(arm_target_msg)

            # ---- Gripper: rate-based, A=open (held) / B=close (held) ----
            a_held = buttons[BTN_A] if len(buttons) > BTN_A else 0
            b_held = buttons[BTN_B] if len(buttons) > BTN_B else 0
            raw_gripper_rate = float(a_held - b_held)

            self.filt_gripper_rate = exp_smooth(
                self.filt_gripper_rate, raw_gripper_rate, DT, RATE_SMOOTHING_TAU)
            self.gripper_target += self.filt_gripper_rate * GRIPPER_RATE * DT
            self.gripper_target = max(GRIPPER_MIN, min(GRIPPER_MAX, self.gripper_target))

            gripper_msg = Float64()
            gripper_msg.data = self.gripper_target
            self.gripper_target_pub.publish(gripper_msg)

            # Arm mode is fully self-contained above -- skip all base-mode
            # logic below for this tick.
            return

        # =====================================================================
        # PART 1: VELOCITY (/cmd_vel) -- DIRECT, NO MEMORY, from BOTH sources.
        # BASE MODE ONLY (arm mode returned above before reaching here).
        # =====================================================================

        # Joystick contribution (same as v3).
        lx_joy = apply_deadzone(axes[AX_LEFT_Y], STICK_DEADZONE)
        ly_joy = apply_deadzone(axes[AX_LEFT_X], STICK_DEADZONE)
        az_joy = apply_deadzone(axes[AX_RIGHT_X], STICK_DEADZONE)

        # Keyboard contribution: a key is either fully active (contributes
        # +-1.0) or not active at all (contributes 0.0) -- there's no
        # in-between for a keyboard, unlike an analog stick.
        lx_key = (1.0 if self.is_key_active(KEY_FORWARD) else 0.0) \
               - (1.0 if self.is_key_active(KEY_BACKWARD) else 0.0)
        ly_key = (1.0 if self.is_key_active(KEY_STRAFE_LEFT) else 0.0) \
               - (1.0 if self.is_key_active(KEY_STRAFE_RIGHT) else 0.0)
        az_key = (1.0 if self.is_key_active(KEY_TURN_LEFT) else 0.0) \
               - (1.0 if self.is_key_active(KEY_TURN_RIGHT) else 0.0)

        # Combine both sources and clip to [-1, 1] (see clip()'s
        # docstring for why this matters).
        lx = clip(lx_joy + lx_key)
        ly = clip(ly_joy + ly_key)
        az = clip(az_joy + az_key)

        twist = Twist()
        twist.linear.x = lx * MAX_LINEAR_SPEED
        twist.linear.y = ly * MAX_LINEAR_SPEED
        twist.angular.z = az * MAX_ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)

        # =====================================================================
        # PART 1.5 (NEW, CP6): GAIT MODE SWITCH -- X button, edge-triggered.
        # =====================================================================
        # Edge-triggered (only toggles on the tick X goes from not-pressed
        # to pressed), same pattern as the Y-button reset just below --
        # NOT held-based, since "switch gait" is a one-shot action, not
        # something that should keep re-triggering every tick X happens
        # to still be held down.
        x_now = buttons[BTN_X]
        if x_now == 1 and self.prev_x_button == 0:
            self.gait_mode = "crawl" if self.gait_mode == "trot" else "trot"
            self.get_logger().info(f"Gait mode switched -> {self.gait_mode}")
        self.prev_x_button = x_now

        # Published CONTINUOUSLY every tick (not just on change) -- same
        # convention already used for /cmd_vel, for the same reason
        # (future e-stop/watchdog compatibility, standard ROS2 practice).
        gait_msg = String()
        gait_msg.data = self.gait_mode
        self.gait_mode_pub.publish(gait_msg)

        # =====================================================================
        # PART 1.75 (NEW): STEP HEIGHT (/step_height) -- D-pad up/down.
        # =====================================================================
        # Deliberately separate from PART 2's POSE block below: step
        # height controls how high a SWINGING foot lifts during gait
        # (consumed by the gait node's swing trajectory), which has
        # nothing to do with body pose (dx/dy/dz/roll/pitch/yaw) and
        # involves NO IK/reachability check -- there's no "is this
        # reachable" question for a lift height, unlike a body pose
        # target. So this doesn't go through the candidate/reachability
        # machinery PART 2 uses below; it's a simple smoothed-rate
        # integrator, same technique as filt_height/dz, just standalone.
        raw_step_height_rate = axes[AX_DPAD_Y]
        # ^ NOTE: polarity untested -- see AX_DPAD_Y's definition above.
        # If D-pad up decreases height instead of increasing it, negate
        # this line: `raw_step_height_rate = -axes[AX_DPAD_Y]`.

        self.filt_step_height_rate = exp_smooth(
            self.filt_step_height_rate, raw_step_height_rate, DT, RATE_SMOOTHING_TAU)
        self.step_height += self.filt_step_height_rate * STEP_HEIGHT_RATE * DT
        # Hard clamp -- per Harsh's explicit requirement: must never go
        # below 0 (a negative lift is physically meaningless) or above
        # 0.08m.
        self.step_height = max(STEP_HEIGHT_MIN, min(STEP_HEIGHT_MAX, self.step_height))

        step_height_msg = Float64()
        step_height_msg.data = self.step_height
        self.step_height_pub.publish(step_height_msg)

        # =====================================================================
        # PART 2: POSE (/body_pose) -- STATEFUL, from BOTH sources.
        # =====================================================================

        # ---- Step 2a: raw pose-axis inputs, joystick + keyboard combined ----

        rt = trigger_depth(axes[AX_RT])
        lt = trigger_depth(axes[AX_LT])
        pitch_key = (1.0 if self.is_key_active(KEY_PITCH_FWD) else 0.0) \
                  - (1.0 if self.is_key_active(KEY_PITCH_BACK) else 0.0)
        # EMPIRICAL SIGN FIX (confirmed live): pitch direction was
        # inverted from what felt intuitive. Flipped HERE, at the raw
        # input, rather than inside pose_controller.py's rpy_to_matrix --
        # that math was already sim-validated in CP4c (pure-pitch test
        # case passed within 2mm tolerance against real Isaac Sim TF
        # data), so it's self-consistent and correct as a coordinate
        # transform; this is purely a "which stick/trigger direction
        # means which physical tilt" preference, same category as the
        # gait engine's own vx/vy sign fixes -- flip at the source,
        # don't touch validated core math.
        raw_pitch = -clip((rt - lt) + pitch_key)

        rb = buttons[BTN_RB]
        lb = buttons[BTN_LB]
        roll_key = (1.0 if self.is_key_active(KEY_ROLL_LEFT) else 0.0) \
                 - (1.0 if self.is_key_active(KEY_ROLL_RIGHT) else 0.0)
        # FIXED in v3 (kept in v4): LB/left-roll-key should roll left,
        # RB/right-roll-key should roll right.
        raw_roll = clip(float(lb - rb) + roll_key)

        a = buttons[BTN_A]
        b = buttons[BTN_B]
        height_key = (1.0 if self.is_key_active(KEY_HEIGHT_UP) else 0.0) \
                   - (1.0 if self.is_key_active(KEY_HEIGHT_DOWN) else 0.0)
        raw_height = clip(float(a - b) + height_key)

        yaw_key = (1.0 if self.is_key_active(KEY_YAW_LEFT) else 0.0) \
                - (1.0 if self.is_key_active(KEY_YAW_RIGHT) else 0.0)
        raw_yaw = clip(axes[AX_DPAD_X] + yaw_key)

        any_pose_input = (abs(raw_pitch) > 0 or raw_roll != 0.0
                           or raw_height != 0.0 or raw_yaw != 0.0)

        # ---- Step 2b: handle reset (Y button OR N key) ----
        y_now = buttons[BTN_Y]
        n_now = self.is_key_active(KEY_RESET)
        # A reset should START when EITHER the button transitions from
        # not-pressed to pressed, OR the key becomes active when it
        # wasn't a moment ago -- either input source can trigger it.
        reset_pressed_this_tick = (
            (y_now == 1 and self.prev_y_button == 0)
            or (n_now and not self.prev_reset_active_key)
        )
        if reset_pressed_this_tick and not self.reset_active:
            self.reset_active = True
            self.reset_start_pose = dict(self.pose)
            self.reset_start_time = time.monotonic()
            self.get_logger().info("Reset triggered -- smoothly returning to neutral.")
        self.prev_y_button = y_now
        self.prev_reset_active_key = n_now

        # ---- Step 2c: build CANDIDATE pose (self.pose untouched until confirmed reachable) ----
        candidate = dict(self.pose)

        if self.reset_active:
            if any_pose_input:
                self.reset_active = False
                self.get_logger().info("Reset interrupted by new input.")
            else:
                elapsed = time.monotonic() - self.reset_start_time
                if elapsed >= RESET_DURATION:
                    candidate = dict(dx=0.0, dy=0.0, dz=0.0,
                                      roll=0.0, pitch=0.0, yaw=0.0)
                    self.reset_active = False
                else:
                    ease = smoothstep(elapsed / RESET_DURATION)
                    candidate = {
                        k: v * (1.0 - ease)
                        for k, v in self.reset_start_pose.items()
                    }

        # ---- Step 2d: normal pose update (only if not mid-reset) ----
        if not self.reset_active:
            self.filt_pitch = exp_smooth(self.filt_pitch, raw_pitch, DT, RATE_SMOOTHING_TAU)
            self.filt_roll = exp_smooth(self.filt_roll, raw_roll, DT, RATE_SMOOTHING_TAU)
            self.filt_height = exp_smooth(self.filt_height, raw_height, DT, RATE_SMOOTHING_TAU)
            self.filt_yaw = exp_smooth(self.filt_yaw, raw_yaw, DT, RATE_SMOOTHING_TAU)

            candidate['pitch'] += self.filt_pitch * PITCH_RATE_MAX * DT
            candidate['roll']  += self.filt_roll  * ROLL_RATE * DT
            candidate['dz']    += self.filt_height * HEIGHT_RATE * DT
            candidate['yaw']   += self.filt_yaw   * YAW_RATE * DT

        # ---- Step 2e: try the candidate; only commit if reachable ----
        # CP6 CHANGE: we still solve IK here, but ONLY as a reachability
        # CHECK -- this is what preserves the "stop exactly at the limit,
        # don't drift past it" clamp behavior from CP5 v3. We deliberately
        # THROW AWAY the solved angles themselves; this node no longer
        # tracks or publishes final joint angles at all, since the CP6
        # gait node will re-solve IK anyway once it adds the gait offset
        # on top of this pose. Solving IK twice (once here for
        # validation, once in the gait node for the real answer) is a
        # real inefficiency worth naming honestly -- acceptable for now
        # since IK is cheap per-leg (progress_log's note: parallelization
        # only becomes worth considering above ~80Hz combined solves).
        try:
            body_pose_to_joint_angles(**candidate)
        except UnreachableTargetError as e:
            # candidate is discarded; self.pose stays at the last pose
            # that WAS confirmed reachable. We still republish THAT last-
            # good pose (not silence) so /body_pose keeps flowing
            # continuously even while held at the limit -- same
            # continuous-publishing convention used for /cmd_vel and
            # /gait_mode.
            self.warn_throttled(f"At reachability limit, holding: {e}")
            self.publish_body_pose(self.pose)
            return

        self.pose = candidate
        self.publish_body_pose(self.pose)


def main():
    rclpy.init()
    node = XboxTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # CRITICAL: put the terminal back to normal mode before exiting.
        # Skipping this leaves the terminal in cbreak mode (no line
        # buffering, no echoed input) even after the script has stopped.
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
