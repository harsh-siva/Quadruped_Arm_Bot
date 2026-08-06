#!/usr/bin/env python3
"""
CP6 -- Gait Generator Node (trot + crawl, smooth blended switching).

Sole owner of /joint_command as of CP6 -- see xbox_teleop_node.py's module
docstring for the CP4/CP5 -> CP6 architecture change this reflects (that
node no longer computes or publishes final joint angles itself; it only
publishes the raw pose/velocity/gait-mode REQUESTS this node consumes).

Subscribes to:
  /cmd_vel     (geometry_msgs/Twist)          -- from xbox_teleop_node.py
  /body_pose   (std_msgs/Float64MultiArray)   -- from xbox_teleop_node.py
    NOTE: fixed order [dx, dy, dz, roll, pitch, yaw], no field names --
    see xbox_teleop_node.py's publish_body_pose() docstring for why this
    is a real (accepted, flagged) fragility.
  /gait_mode   (std_msgs/String, "trot" or "crawl") -- from xbox_teleop_node.py

Publishes:
  /joint_command (sensor_msgs/JointState) -- final servo angles, 12 joints.

HIGH-LEVEL PIPELINE (see CP6e's diagram from chat discussion):
  cmd_vel + gait table  --> per-leg gait OFFSET (CP6b/c/d, this file)
  commanded body pose   --> per-leg pose TARGET (CP4's pose_controller.py)
  offset rotated into current body tilt, ADDED to pose target (CP6e)
  --> per-leg final foot target --> ik_leg() --> joint angles --> publish
"""

import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String, Float64
from sensor_msgs.msg import JointState

from pose_controller import body_pose_to_leg_targets, rpy_to_matrix
from fk_leg import LEG_PARAMS
from ik_leg import ik_leg, UnreachableTargetError

# =============================================================================
# CONTROL LOOP RATE
# =============================================================================
# Matches xbox_teleop_node.py's rate -- justified by the CP6f timing check:
# worst-case 4-leg IK solve measured at 5.333ms on Harsh's machine,
# comfortably inside a 20ms/50Hz tick budget (~27% of budget, ~14.7ms
# headroom for everything else this loop does).
CONTROL_RATE_HZ = 70.0
DT = 1.0 / CONTROL_RATE_HZ

# =============================================================================
# JOINT_ORDER -- moved here from xbox_teleop_node.py as of CP6 (this node
# is now the one building JointState messages; that file no longer does).
# =============================================================================
JOINT_ORDER = [
    ("FR", "J_Coxa_FR", "J_Femur_FR", "J_Tibia_FR"),
    ("BR", "J_Coxa_BR", "J_Femur_BR", "J_Tibia_BR"),
    ("FL", "J_Coxa_FL", "J_Femur_FL", "J_Tibia_FL"),
    ("BL", "J_Coxa_BL", "J_Femur_BL", "J_Tibia_BL"),
]

def exp_smooth(current, target, dt, tau):
    """
    One step of exponential smoothing toward `target`. Same technique
    (and same shape of formula) as xbox_teleop_node.py's own exp_smooth()
    -- duplicated here rather than imported, since importing the teleop
    NODE file into this one would be an odd cross-dependency for a
    3-line utility function. Same "duplicate a small independent piece
    on purpose" pattern this project already uses for C3 in
    validate_foot_position.py/validate_body_pose.py.
    """
    alpha = min(1.0, dt / tau)
    return current + (target - current) * alpha


# How quickly the gait engine's INTERNAL velocity tracks the raw
# /cmd_vel it receives. Starting from the SAME value as CP5's
# RATE_SMOOTHING_TAU (0.15) for consistency -- untuned placeholder,
# adjust if it feels too sluggish or still too jerky once tested live.
CMD_VEL_SMOOTHING_TAU = 0.15
# Both trot and crawl share the SAME structure (duty_cycle + per-leg phase
# offset) so switching gaits is "swap which table gets read," not two
# separate engines -- see CP6a/CP6c chat discussion.
#
# CONVENTION: offset marks the START of that leg's SWING window (not
# stance). E.g. crawl's BL=0.25 means BL's swing occupies [0.25, 0.5) of
# the cycle.
GAIT_TABLES = {
    "trot": dict(
        duty_cycle=0.5,
        offsets={"FR": 0.0, "BL": 0.0, "FL": 0.5, "BR": 0.5},
        # UPDATED (Harsh's request, this chat): raised from 0.05 to get a
        # bigger hip/coxa swing. Starting at 0.10 -- inside the leg's real
        # workspace (~0.2-0.25m total reach per fk_leg.py's C1+C2+C3), but
        # UNVERIFIED against ik_leg() at this exact value yet. Watch for
        # UnreachableTargetError warnings live; back off if they appear
        # constantly (occasional warnings mid-turn are expected/handled).
        max_step_length=0.12,
        cycle_period_min=0.6,
        # RAISED from 1.4 -- needed so low-speed commands can still reach
        # the now-larger max_step_length without hitting this ceiling and
        # falling back to a shorter stride (see compute_shared_step_params'
        # docstring for why this ceiling matters more now).
        cycle_period_max=3.0,
    ),

    "crawl": dict(
        duty_cycle=0.75,
        # Stepping order FR -> BL -> FL -> BR: each successive lift is on
        # the "opposite corner" from the last (never two same-side legs
        # back-to-back), which keeps the 3-foot support triangle as
        # large/centered as possible at every instant -- see CP6c
        # discussion for the full stability reasoning.
        offsets={"FR": 0.0, "BL": 0.25, "FL": 0.5, "BR": 0.75},
        # TUNING FIX (from live sim feedback): crawl was sharing trot's
        # max_step_length/cycle_period bounds. Because crawl's swing
        # fraction (0.25) is much smaller than trot's (0.5), sharing the
        # same step-length cap made crawl's ABSOLUTE swing time roughly
        # 3x shorter than trot's at the same commanded speed -- felt
        # rushed/jittery, and capped step length too early for a gait
        # that's meant to be slower and more deliberate. Bigger step
        # cap + longer period bounds, both roughly 1.6-2x trot's,
        # starting guesses -- UNTUNED, adjust from here based on feel.
        max_step_length=0.1,
        cycle_period_min=1.2,
        cycle_period_max=2.4,
    ),
}

# =============================================================================
# CP6d -- STEP-PARAMETER TUNABLES
# =============================================================================
# Functional placeholders, same spirit as xbox_teleop_node.py's own
# un-tuned constants (progress_log.md flags those explicitly; doing the
# same here rather than pretending these numbers are final).
# NOTE: MAX_STEP_LENGTH / CYCLE_PERIOD_MIN / CYCLE_PERIOD_MAX used to
# live here as GLOBAL constants shared by both gaits -- moved INTO
# GAIT_TABLES (per-gait) as of the crawl tuning fix, since sharing them
# was the root cause of crawl's swing time being ~3x too short. See
# GAIT_TABLES's crawl entry for the full explanation.
STEP_HEIGHT_DEFAULT = 0.02  # meters -- swing clearance. NO LONGER the sole
                            # value: as of the D-pad step-height control,
                            # this is only the STARTUP value used before
                            # the first /step_height message arrives (same
                            # role latest_body_pose's zero-default plays).
                            # Actual runtime value is self.latest_step_height.
                            # Independent, exposed parameter by design --
                            # see CP6d's terrain-adaptation discussion for
                            # why amplitude was kept separate from other
                            # gait parameters from the start.
STEP_HEIGHT_MIN = 0.0      # meters -- matches xbox_teleop_node.py's clamp;
                            # defensive re-clamp here too (see body_pose's
                            # own note on trusting-but-verifying upstream
                            # values), not just relying on teleop's clamp.
STEP_HEIGHT_MAX = 0.08      # meters -- matches xbox_teleop_node.py's clamp.

# CP6f -- gait-switch blend window.
GAIT_SWITCH_DURATION = 0.4   # seconds, tunable transition length.


def compute_leg_velocities(vx_cmd, vy_cmd, wz):
    """
    Computes EACH leg's own (vx, vy) velocity in the body frame,
    combining commanded body translation with the ROTATIONAL
    contribution from wz (angular.z), via the standard rigid-body
    point-velocity formula v = omega x r, where r is that leg's position
    relative to the body's rotation center.

    NOTE: takes plain floats (vx_cmd, vy_cmd, wz), NOT a raw Twist
    message. As of the cmd_vel-smoothing fix, the caller (control_loop)
    passes SMOOTHED values here, not /cmd_vel's raw instantaneous
    numbers -- see CMD_VEL_SMOOTHING_TAU. This function itself doesn't
    know or care whether its inputs are smoothed; that's decided once,
    upstream, not duplicated here.

    APPROXIMATION: uses hip_offset's (x, y) as that leg's radius r. The
    true rotation center is the body origin either way; hip_offset is a
    reasonable stand-in for "how far out is this leg," not re-derived
    from the neutral foot position -- kept simple deliberately.

    In 2D, omega x r for r=(x,y), omega=wz (about z): v_rot = (-wz*y, wz*x).

    Returns: {leg: (vx, vy)}.
    """
    # EMPIRICAL SIGN FIX (confirmed in CP6g sim testing): flip vx_cmd's
    # contribution here, the single source point it enters the per-leg
    # calculation. wz is NOT flipped -- confirmed correct as-is by
    # Harsh's turning test.
    vx_cmd = -vx_cmd
    # EMPIRICAL SIGN FIX (confirmed live, strafe test): flip vy_cmd's
    # contribution too, same single-source-point pattern as vx_cmd
    # above. Confirmed via direct sim observation, not assumed from
    # vx_cmd's fix -- strafe and forward/back are physically different
    # directions and needed their own independent confirmation.
    vy_cmd = -vy_cmd

    velocities = {}
    for leg, p in LEG_PARAMS.items():
        hip_x, hip_y = p["hip_offset"][0], p["hip_offset"][1]
        v_rot_x = -wz * hip_y
        v_rot_y =  wz * hip_x
        velocities[leg] = (vx_cmd + v_rot_x, vy_cmd + v_rot_y)
    return velocities


# Speed (m/s, per-leg) below which step_height ramps toward zero -- i.e.
# how much stick push before legs start actually lifting off the ground.
# Tunable placeholder, same spirit as this file's other un-tuned
# constants.
SPEED_RAMP_THRESHOLD = 0.01

# How wide a band around vx=0 the forward/backward turn-direction flip
# smoothly transitions over, instead of snapping instantly at exactly
# vx=0 -- same magnitude as SPEED_RAMP_THRESHOLD, reused deliberately
# for consistency; both are "how much velocity before X fully engages"
# thresholds. Untuned placeholder.
DIRECTION_FLIP_BAND = 0.01


def turn_direction_multiplier(vx, band):
    """
    Returns the multiplier applied to angular.z for car-style turning
    (Harsh's explicit choice): +1 while moving forward OR standing still
    (vx >= 0), smoothly transitioning to -1 only as vx actually goes
    NEGATIVE (reversing), fully -1 once vx <= -band.

    BUG THIS REPLACES: an earlier version (smooth_sign) was a symmetric
    odd function that evaluated to EXACTLY 0 at vx=0 -- which silently
    killed standalone in-place rotation (right stick alone, no forward/
    back input) entirely, since multiplying angular.z by 0 zeroes it
    out. That was the wrong SHAPE for this job, not just a missing edge
    case: standing still should behave like the already-confirmed-
    correct forward case (multiplier=+1), not like an undefined
    midpoint. This version is asymmetric on purpose: pinned at +1 for
    ALL vx >= 0 (covers both forward AND standalone rotation identically
    -- deliberately does not "ramp down" near vx=0 the way the old
    version did), and only eases toward -1 once vx is actually negative.
    Uses a cosine ease (zero slope at both t=0 and t=1) so the flip
    itself is still smooth, not an instant snap, once reversing does
    start.
    """
    if vx >= 0:
        return 1.0
    if vx <= -band:
        return -1.0
    t = -vx / band   # 0 at vx=0, 1 at vx=-band
    return np.cos(np.pi * t)   # 1 at t=0, -1 at t=1, smooth in between


def speed_ramp(speed, threshold):
    """
    Smoothly scales from 0 (no commanded motion -- legs stay planted, no
    lift) to 1 (full step_height) as speed goes from 0 to `threshold`.
    Uses the SAME raised-cosine ease shape as swing_trajectory's own
    curves (CP6b) and the gait-switch blend weight (CP6f) -- continuous
    in both value AND slope, so there's no jump at speed=0 or at the
    threshold, consistent with this file's smoothness principle
    throughout. Deliberately a function of REAL-TIME speed, not a
    timer-based on/off switch -- this means it can never get "stuck"
    with a leg frozen mid-air, since it just continuously tracks
    whatever cmd_vel actually is, tick to tick.
    """
    if speed <= 0:
        return 0.0
    if speed >= threshold:
        return 1.0
    t = speed / threshold
    return (1 - np.cos(np.pi * t)) / 2.0


def compute_shared_step_params(vx_cmd, vy_cmd, wz, duty_cycle,
                                max_step_length, cycle_period_min, cycle_period_max):
    """
    Step LENGTH differs per leg (see compute_leg_velocities), but cycle
    PERIOD must stay the SAME for all 4 legs -- otherwise legs drift out
    of phase with each other over time, breaking trot/crawl's offset
    patterns (CP6c). Sizes ONE shared period so the FASTEST-moving leg
    (typically an outer leg mid-turn) doesn't exceed max_step_length;
    slower legs then naturally get shorter steps via the same period
    (step_length = velocity * duty_cycle * period -- CP6d's original
    formula, applied per-leg using this shared period).

    max_step_length/cycle_period_min/cycle_period_max are now PER-GAIT
    (passed in from GAIT_TABLES), not global -- see the crawl tuning fix
    note on GAIT_TABLES for why sharing them across gaits was wrong.

    Takes SMOOTHED vx_cmd/vy_cmd/wz (see compute_leg_velocities' note on
    why smoothing happens upstream, once, in control_loop).

    CHANGED (Harsh's request, this chat): previously, desired stride
    length scaled DOWN proportionally with commanded speed (via
    desired_length = max_speed * duty_cycle * cycle_period_max), so
    max_step_length was only actually reached at top speed -- slower
    speeds got a smaller stride "for free" as a side effect of that
    formula, not by design. Now cycle_period always solves for
    max_step_length directly, regardless of speed: stride length is
    constant across the whole speed range, and PERIOD (stepping cadence)
    is what changes with speed instead. Real tradeoff: at very low
    speed, the period needed to still cover max_step_length can exceed
    cycle_period_max, in which case the clip() below still caps stride
    short of max -- just for a different reason now (period ceiling
    instead of speed-based scaling). If this happens, raise
    cycle_period_max in GAIT_TABLES rather than change this formula.

    Returns: (leg_velocities dict, cycle_period, max_speed)
    """
    leg_velocities = compute_leg_velocities(vx_cmd, vy_cmd, wz)
    speeds = {leg: np.hypot(vx, vy) for leg, (vx, vy) in leg_velocities.items()}
    max_speed = max(speeds.values())

    if max_speed < 1e-6:
        return leg_velocities, cycle_period_max, max_speed

    cycle_period = max_step_length / (duty_cycle * max_speed)
    cycle_period = np.clip(cycle_period, cycle_period_min, cycle_period_max)

    return leg_velocities, cycle_period, max_speed

# =============================================================================
# CP6b -- TRAJECTORY SHAPES
# =============================================================================
def swing_trajectory(s, start_xy, end_xy, step_height):
    """
    Foot path during SWING (foot in the air), as a function of
    swing-progress s in [0, 1] (0 = liftoff, 1 = touchdown).

    Horizontal: cosine-eased -- velocity is exactly zero at s=0 and s=1,
    so horizontal speed ramps smoothly instead of snapping.
    Vertical: sine arc, zero at s=0/s=1, peak at s=0.5. Zero VERTICAL
    velocity at touchdown is what prevents the foot from slamming/
    bouncing -- the more important of the two zero-velocity properties
    (see stance_trajectory()'s docstring for the accepted asymmetry here).

    Returns: (x, y, z) offset from the neutral foot position, in the
    body's own forward/lateral/vertical directions -- NOT yet rotated
    into the current tilted body frame. That rotation happens once, in
    the main control loop (CP6e).
    """
    ease = (1 - np.cos(np.pi * s)) / 2.0
    x = start_xy[0] + (end_xy[0] - start_xy[0]) * ease
    y = start_xy[1] + (end_xy[1] - start_xy[1]) * ease
    # CORRECTED (caught by CP6f's own verification test, not assumed
    # correct from the original CP6b design discussion): sin(pi*s) has
    # zero POSITION at s=0/s=1 but MAXIMUM slope there (dz/ds = pi*cos at
    # s=0 -> pi, not 0) -- exactly backwards from "zero touchdown
    # velocity." (1-cos(2*pi*s))/2 has zero POSITION *and* zero VELOCITY
    # at s=0, s=0.5, and s=1 -- same "raised cosine" shape already used
    # for the horizontal ease above, just peaking mid-swing instead of
    # ramping one-way.
    z = step_height * (1 - np.cos(2 * np.pi * s)) / 2.0
    return np.array([x, y, z])


def stance_trajectory(s, start_xy, end_xy):
    """
    Foot path during STANCE (foot planted, dragging backward relative to
    the body), as a function of stance-progress s in [0, 1].
    Straight-line, CONSTANT velocity from start_xy to end_xy.

    OPEN ITEM (flagged in chat, deliberately deferred): unlike
    swing_trajectory(), this does NOT ease to zero velocity at its own
    boundaries -- there is a small HORIZONTAL velocity discontinuity at
    the swing<->stance handoff. Accepted for now because the VERTICAL
    touchdown velocity (the dominant cause of a felt/visible jolt) is
    zero either way. Revisit with cycloid-style eased stance ONLY if
    CP6g sim testing actually shows it's a problem -- not preemptively.
    """
    x = start_xy[0] + (end_xy[0] - start_xy[0]) * s
    y = start_xy[1] + (end_xy[1] - start_xy[1]) * s
    z = 0.0
    return np.array([x, y, z])


# =============================================================================
# CP6c -- PHASE STATE
# =============================================================================
def leg_gait_state(global_phase, offset, duty_cycle):
    """
    Given the current global cycle phase (0-1) and one leg's own offset
    + the active gait's duty cycle: is this leg in "swing" or "stance"
    right now, and how far progressed (0-1) WITHIN that phase? This
    progress value is exactly the `s` argument the trajectory functions
    above expect.
    """
    swing_width = 1.0 - duty_cycle
    local = (global_phase - offset) % 1.0

    if local < swing_width:
        return "swing", local / swing_width
    else:
        stance_width = duty_cycle
        return "stance", (local - swing_width) / stance_width


def leg_gait_offset(leg, global_phase, gait_table, step_length_x, step_length_y, step_height):
    """
    Combines phase state (CP6c) with trajectory shape (CP6b) for ONE leg
    under ONE gait's table.

    Anchor points: the step is centered on the neutral foot position --
    stance drags the foot from FRONT (+half step) to BACK (-half step),
    matching how a planted foot appears to move backward as the body
    walks forward over it; swing then carries it back from BACK to FRONT
    in the air. This is what makes stance and swing chain into a
    continuous repeating loop.
    """
    offset = gait_table["offsets"][leg]
    duty_cycle = gait_table["duty_cycle"]
    phase_kind, s = leg_gait_state(global_phase, offset, duty_cycle)

    front = np.array([ step_length_x / 2.0,  step_length_y / 2.0])
    back  = np.array([-step_length_x / 2.0, -step_length_y / 2.0])

    if phase_kind == "swing":
        return swing_trajectory(s, back, front, step_height)
    else:
        return stance_trajectory(s, front, back)


class GaitGeneratorNode(Node):
    """
    Main node. See module docstring for the full pipeline. Runs BOTH
    gaits' phase clocks continuously (whether active or not) so a
    /gait_mode switch can begin blending instantly, with no need to wait
    for any synchronization point -- see CP6f chat discussion.
    """

    def __init__(self):
        super().__init__('gait_generator_node')

        # ---- Latest received teleop values (defaults = safe/idle) ----
        self.latest_cmd_vel = Twist()
        self.latest_body_pose = dict(
            dx=0.0, dy=0.0, dz=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        self.latest_gait_mode = "trot"
        self.latest_step_height = STEP_HEIGHT_DEFAULT

        # ---- CP6c: both gaits' phase clocks, always running ----
        self.phase = {"trot": 0.0, "crawl": 0.0}

        # ---- CP6f: gait-switch blend state ----
        # active_gait = what we're blending TOWARD (matches /gait_mode
        # once any transition finishes). previous_gait = what we're
        # blending FROM. transition_start_time = None means "not
        # currently transitioning" (mirrors xbox_teleop_node.py's
        # reset_start_time pattern for the identical reason: a boolean
        # can't drive an eased curve, only elapsed time can).
        self.active_gait = "trot"
        self.previous_gait = "trot"
        self.transition_start_time = None

        # ---- Smoothed cmd_vel (fixes "jerk on stick release/change") ----
        # /cmd_vel arrives RAW/instant from teleop (correct there, by
        # design -- see xbox_teleop_node.py's docstring). But a sudden
        # speed change mid-gait-cycle would yank step_x/step_y to a new
        # value abruptly, mid-swing or mid-stance, producing a position
        # discontinuity. Smoothing happens HERE instead, once, before
        # anything else reads velocity -- see compute_leg_velocities'
        # docstring for why the smoothing responsibility lives on this
        # (consumer) side rather than the teleop (producer) side.
        self.filt_vx = 0.0
        self.filt_vy = 0.0
        self.filt_wz = 0.0

        # ---- Warm-start state for IK ----
        # Each leg's most recently SOLVED angles, used as next tick's
        # initial_guess instead of always starting from (0,0,0). Without
        # this, the optimizer restarts cold every single tick, which
        # risks tiny tick-to-tick inconsistencies even for a smoothly
        # moving target, on top of being slower than necessary -- folded
        # in now since it's directly relevant to "really smooth," not a
        # separate feature.
        self.last_solved_angles = {leg: (0.0, 0.0, 0.0) for leg in LEG_PARAMS}

        # ---- ROS2 plumbing ----
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.body_pose_sub = self.create_subscription(
            Float64MultiArray, '/body_pose', self.body_pose_callback, 10)
        self.gait_mode_sub = self.create_subscription(
            String, '/gait_mode', self.gait_mode_callback, 10)
        self.step_height_sub = self.create_subscription(
            Float64, '/step_height', self.step_height_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            f"Gait generator node started ({CONTROL_RATE_HZ} Hz control loop). "
            f"Sole publisher of /joint_command as of CP6.")

    # ---- Subscriber callbacks ----

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel = msg

    def body_pose_callback(self, msg):
        # Un-pack the FIXED [dx, dy, dz, roll, pitch, yaw] order -- must
        # match xbox_teleop_node.py's publish_body_pose() exactly (see
        # that function's docstring on why this ordering is a real,
        # accepted fragility rather than something the message enforces).
        dx, dy, dz, roll, pitch, yaw = msg.data
        self.latest_body_pose = dict(
            dx=dx, dy=dy, dz=dz, roll=roll, pitch=pitch, yaw=yaw)

    def gait_mode_callback(self, msg):
        new_gait = msg.data
        if new_gait != self.active_gait and self.transition_start_time is None:
            # Guard: transition_start_time is None means "not currently
            # transitioning." A SECOND switch request arriving mid-
            # transition is deliberately IGNORED here (dropped) rather
            # than restarting the blend clock -- interrupting a blend
            # mid-way with a fresh one is a real open design question
            # (does it restart cleanly? blend-of-a-blend?) NOT solved in
            # this chat. Fine for now since a human double-tapping X
            # within 0.4s is an edge case, not the common path -- revisit
            # if CP6h's live teleop test shows it matters.
            self.previous_gait = self.active_gait
            self.active_gait = new_gait
            self.transition_start_time = time.monotonic()

    def step_height_callback(self, msg):
        # Defensive re-clamp here too, even though xbox_teleop_node.py
        # already clamps at the source -- same "don't fully trust one
        # layer" reasoning already applied elsewhere in this project
        # (e.g. C3 duplicated independently in validate_*.py). Cheap
        # insurance against a bad value from a future/different publisher.
        self.latest_step_height = max(STEP_HEIGHT_MIN, min(STEP_HEIGHT_MAX, msg.data))

    # ---- Blend weight ----

    def compute_blend_weight(self):
        """
        Returns (w, still_transitioning). w=0 -> fully previous_gait,
        w=1 -> fully active_gait. Uses the SAME cosine ease as CP6b's
        swing curve, so the blend weight's own velocity is zero at both
        ends of the transition window -- this is what makes the blended
        POSITION's velocity match "100% old gait" going in and "100% new
        gait" coming out, with no jump at either seam (see CP6f chat
        discussion on why a linear ramp would reintroduce exactly the
        kind of velocity jump CP6b eliminated elsewhere).
        """
        if self.transition_start_time is None:
            return 1.0, False

        elapsed = time.monotonic() - self.transition_start_time
        if elapsed >= GAIT_SWITCH_DURATION:
            return 1.0, False

        t = elapsed / GAIT_SWITCH_DURATION
        w = (1 - np.cos(np.pi * t)) / 2.0
        return w, True

    # ---- Main control loop ----

    def control_loop(self):
        """
        Called every DT (~20ms at 50Hz). See module docstring for the
        full pipeline this implements.
        """
        # ---- Step 0: smooth cmd_vel BEFORE anything else uses it ----
        # See __init__'s note on self.filt_vx/vy/wz for why this exists.
        self.filt_vx = exp_smooth(self.filt_vx, self.latest_cmd_vel.linear.x, DT, CMD_VEL_SMOOTHING_TAU)
        self.filt_vy = exp_smooth(self.filt_vy, self.latest_cmd_vel.linear.y, DT, CMD_VEL_SMOOTHING_TAU)
        self.filt_wz = exp_smooth(self.filt_wz, self.latest_cmd_vel.angular.z, DT, CMD_VEL_SMOOTHING_TAU)

        # CAR-STYLE TURN DIRECTION (Harsh's explicit choice, this chat):
        # the SAME angular.z input should turn the robot the OPPOSITE
        # physical direction when walking backward vs forward -- unlike
        # standard independent-yaw-rate Twist semantics, where angular.z
        # always means the same rotation regardless of linear.x. Flip is
        # based on filt_vx's sign (forward/backward only) -- NOT vy
        # (strafe): only forward/back + turn was reported and tested;
        # extending this to strafe+turn would be guessing without
        # evidence, same reasoning as the earlier linear.x-only sign fix.
        effective_wz = self.filt_wz * turn_direction_multiplier(self.filt_vx, DIRECTION_FLIP_BAND)

        # ---- Step 1: advance BOTH gaits' phase clocks ----
        # Each gait's clock uses THAT gait's own shared cycle_period (see
        # compute_shared_step_params -- period is shared across all 4
        # legs of a given gait, only per-leg STEP LENGTH differs, so
        # phase relationships between legs stay intact even while
        # turning).
        leg_velocities_by_gait = {}
        for gait_name, table in GAIT_TABLES.items():
            leg_vel, period, max_speed = compute_shared_step_params(
                self.filt_vx, self.filt_vy, effective_wz, table["duty_cycle"],
                table["max_step_length"], table["cycle_period_min"], table["cycle_period_max"])
            leg_velocities_by_gait[gait_name] = (leg_vel, period, max_speed)
            self.phase[gait_name] = (self.phase[gait_name] + DT / period) % 1.0

        # ---- Step 2: this tick's blend weight ----
        w, still_transitioning = self.compute_blend_weight()
        if not still_transitioning:
            self.transition_start_time = None   # clear once finished

        # ---- Step 3: current pose -> rotation matrix + CP4 leg targets ----
        # Recomputed fresh every tick since the commanded pose can change
        # continuously while walking (e.g. a tilt held mid-stride).
        pose = self.latest_body_pose
        R = rpy_to_matrix(pose['roll'], pose['pitch'], pose['yaw'])
        pose_targets = body_pose_to_leg_targets(
            pose['dx'], pose['dy'], pose['dz'],
            pose['roll'], pose['pitch'], pose['yaw'])

        # ---- Step 4: per-leg blended gait offset, rotated, combined with pose ----
        final_targets = {}
        for leg in LEG_PARAMS:
            offsets_by_gait = {}
            for gait_name, table in GAIT_TABLES.items():
                leg_vel, period, max_speed = leg_velocities_by_gait[gait_name]
                vx, vy = leg_vel[leg]
                step_x = vx * table["duty_cycle"] * period
                step_y = vy * table["duty_cycle"] * period
                # Gate lift height by commanded speed -- this is the fix
                # for "marching in place": with zero cmd_vel, step_x/y
                # are already correctly zero, but height used to be
                # applied unconditionally regardless. Now it ramps to
                # zero smoothly as speed drops to zero too, so idle legs
                # stay planted instead of lifting with nowhere to go.
                # NOTE: uses self.latest_step_height (live, D-pad
                # controlled), not a fixed constant -- see
                # STEP_HEIGHT_DEFAULT's note above.
                height = self.latest_step_height * speed_ramp(max_speed, SPEED_RAMP_THRESHOLD)
                offsets_by_gait[gait_name] = leg_gait_offset(
                    leg, self.phase[gait_name], table, step_x, step_y, height)

            # Blend in POSITION space (previous_gait -> active_gait), NOT
            # gait-PARAMETER space -- interpolating duty_cycle/offsets
            # directly could produce an invalid intermediate gait with
            # too few feet planted. See CP6f "why blending parameters
            # doesn't work" discussion.
            old_offset = offsets_by_gait[self.previous_gait]
            new_offset = offsets_by_gait[self.active_gait]
            blended_offset = (1 - w) * old_offset + w * new_offset

            # CP6e: rotate the flat, body-neutral-frame gait offset by R
            # -- the SAME rotation the pose controller already uses --
            # so "forward" in the gait means forward relative to the
            # body's CURRENT tilt. Uses R (not R.T): this is a motion
            # command being expressed INTO the tilted frame, the mirror
            # case of pose_controller.py's own R.T (which UNDOES a body
            # motion to find where a planted foot appears from the
            # body's perspective).
            rotated_offset = R @ blended_offset

            # Core CP6e combine: static pose target + time-varying gait
            # offset, added since both are displacements from the same
            # neutral reference point.
            final_targets[leg] = pose_targets[leg] + rotated_offset

        # ---- Step 5: solve IK per leg (warm-started), with a DECIDED
        #              fallback policy for unreachable targets ----
        #
        # DESIGN DECISION (this chat) resolving progress_log.md's open
        # item: "any future caller (e.g. CP6's gait generator) will need
        # to make its own decision about how to handle
        # UnreachableTargetError."
        #
        # CHOSEN POLICY: per-leg best-effort fallback, using the
        # exception's own closest_angles (nearest achievable position the
        # optimizer found) for JUST that leg, while every other leg
        # proceeds normally. DELIBERATELY DIFFERENT from teleop's policy
        # (reject the WHOLE candidate, hold ALL legs at last-good) --
        # because gait targets change every single tick by design (that
        # IS walking), so a momentary near-limit target on ONE leg
        # mid-stride is better absorbed on that leg alone than by
        # freezing all four legs at once, which would look like the
        # whole robot stuttering rather than one leg slightly
        # under-reaching. Revisit if CP6g sim testing shows this produces
        # visibly bad single-leg behavior.
        angles = {}
        for leg, target in final_targets.items():
            try:
                solved_angles, _ = ik_leg(
                    leg, target, initial_guess=self.last_solved_angles[leg])
            except UnreachableTargetError as e:
                self.get_logger().warn(
                    f"[{leg}] unreachable this tick, using closest-approach "
                    f"fallback: {e}")
                solved_angles = e.closest_angles

            angles[leg] = solved_angles
            self.last_solved_angles[leg] = tuple(solved_angles)

        self.publish_angles(angles)

    def publish_angles(self, angles):
        """Builds and publishes a JointState message. Same structure
        xbox_teleop_node.py's v3/v4 used for this -- now living here
        instead, since this node is the sole /joint_command publisher
        as of CP6."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        names = []
        positions = []
        for leg, coxa_name, femur_name, tibia_name in JOINT_ORDER:
            tc, tf, tt = angles[leg]
            names += [coxa_name, femur_name, tibia_name]
            positions += [float(tc), float(tf), float(tt)]
        msg.name = names
        msg.position = positions
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = GaitGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
