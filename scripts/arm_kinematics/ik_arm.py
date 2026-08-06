import numpy as np
from scipy.optimize import least_squares
from fk_arm import fk_arm, ARM_JOINTS, BOUNDS_LOWER, BOUNDS_UPPER

# =============================================================================
# DESIGN DECISION (this chat): wrist_roll is DIRECTLY driven, not solved.
#
# The arm has 5 positioning joints but the teleop scheme gives 4 targets
# (x, y, z, pitch) PLUS a 5th input (LB/RB, "rotate gripper left/right")
# that the person described as spinning the gripper -- i.e. a literal
# joint command, not an abstract tool-orientation target to be solved for.
# Since wrist_roll is ALREADY the last joint in the chain, commanding it
# directly is both simpler and matches that mental model exactly (LB/RB
# spins the gripper, full stop -- no risk of the IK solver doing something
# unexpected with it). It's still correctly included in the FK used for
# the x/y/z/pitch solve below (gripper_frame_link is close to, but not
# exactly on, the wrist_roll rotation axis, so it DOES slightly affect
# position) -- it's just passed in as a known constant rather than solved
# as a 5th unknown. This leaves exactly 4 joints solving exactly 4
# targets: a well-determined system, not an under- or over-constrained one.
# =============================================================================

# Reachability tolerance -- same role, same starting value, as
# ik_leg.py's REACHABILITY_TOLERANCE_M (2mm). Not yet tuned against real
# arm accuracy.
REACHABILITY_TOLERANCE_M = 0.002  # 2 mm
# Companion tolerance for the pitch residual (radians). Starting point,
# not derived/tuned -- same honesty convention as everything else here.
PITCH_TOLERANCE_RAD = 0.02  # ~1.1 degrees

# The 4 joints this module actually solves for (everything except
# wrist_roll, which is fixed per-call -- see design note above).
SOLVED_JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
_SOLVED_IDX = [i for i, j in enumerate(ARM_JOINTS) if j[0] in SOLVED_JOINT_NAMES]
SOLVED_BOUNDS_LOWER = BOUNDS_LOWER[_SOLVED_IDX]
SOLVED_BOUNDS_UPPER = BOUNDS_UPPER[_SOLVED_IDX]

# World-Y is the chosen "pitch axis" -- i.e. "pitch" means "tilt the
# gripper up/down by rotating the home orientation about the arm base's
# own Y axis." SIMPLIFICATION, flagged honestly: this is a fixed axis in
# the arm's BASE frame, not the gripper's own current frame, and doesn't
# track shoulder_pan if that joint rotates far from home. Reasonable for
# the modest ARM_TARGET_LIMIT (+/-0.10m) offsets this teleop scheme
# allows, but revisit if large shoulder_pan excursions make "pitch" feel
# wrong in practice.
_PITCH_AXIS = np.array([0.0, 1.0, 0.0])


def _rot_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# Home orientation (rotation matrix), computed once from fk_arm at the
# all-zero configuration -- "pitch offsets" are defined relative to this.
_R_HOME = fk_arm([0.0, 0.0, 0.0, 0.0, 0.0])[:3, :3]


def _orientation_error_about_axis(R_current, R_target, axis):
    """
    Standard operational-space orientation error: sum of cross products of
    corresponding columns of R_current and R_target approximates the
    rotation vector needed to align R_current with R_target (exact for
    R_current == R_target, i.e. returns the zero vector). Projected onto
    `axis` to get a SCALAR "how much rotation about this one axis is still
    needed" -- since only 1 orientation DOF (pitch) is being solved for
    here, not full 3D orientation (which would over-constrain a system
    that only has 1 remaining unknown after x/y/z).
    """
    err_vec = 0.5 * (
        np.cross(R_current[:, 0], R_target[:, 0])
        + np.cross(R_current[:, 1], R_target[:, 1])
        + np.cross(R_current[:, 2], R_target[:, 2])
    )
    return float(np.dot(err_vec, axis))


class ArmUnreachableTargetError(Exception):
    """Same role as ik_leg.py's UnreachableTargetError -- raised when the
    best solution found still isn't close enough to count as reachable.
    Carries the best-effort solution so the caller can fall back to it."""

    def __init__(self, target_xyz, target_pitch, closest_angles,
                 closest_position, pos_error_m, pitch_error_rad):
        self.target_xyz = np.array(target_xyz)
        self.target_pitch = target_pitch
        self.closest_angles = closest_angles
        self.closest_position = closest_position
        self.pos_error_m = pos_error_m
        self.pitch_error_rad = pitch_error_rad
        super().__init__(
            f"Target xyz={target_xyz}, pitch={target_pitch:.3f} rad is "
            f"unreachable -- closest position {closest_position}, off by "
            f"{pos_error_m*1000:.2f}mm / {np.degrees(pitch_error_rad):.2f} deg."
        )


def ik_arm(target_xyz, target_pitch, wrist_roll, initial_guess):
    """
    Inverse kinematics for the arm's 4 non-wrist_roll joints.

    target_xyz    : desired gripper_frame_link position (arm base_link frame).
    target_pitch  : desired tilt (radians) about the base-frame Y axis,
                    relative to home orientation (see _PITCH_AXIS note).
    wrist_roll    : the CURRENT wrist_roll joint angle -- passed in as a
                    known constant, not solved (see module docstring).
    initial_guess : (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex)
                    to warm-start from -- should be the arm's own
                    last-commanded angles, same "seed from current state"
                    principle as gait_generator_node.py's warm-started
                    per-leg IK.

    Returns: (solved_4_angles, pos_error_m, pitch_error_rad) if within
    tolerance. Raises ArmUnreachableTargetError otherwise, carrying a
    best-effort fallback solution on the exception (same pattern as
    ik_leg.py/UnreachableTargetError).
    """
    R_target = _rot_y(target_pitch) @ _R_HOME

    def residual(theta4):
        full_theta = list(theta4) + [wrist_roll]
        T = fk_arm(full_theta)
        pos_err = T[:3, 3] - np.array(target_xyz)
        pitch_err = _orientation_error_about_axis(T[:3, :3], R_target, _PITCH_AXIS)
        return np.array([pos_err[0], pos_err[1], pos_err[2], pitch_err])

    result = least_squares(
        residual,
        x0=np.array(initial_guess),
        bounds=(SOLVED_BOUNDS_LOWER, SOLVED_BOUNDS_UPPER),
    )

    full_theta = list(result.x) + [wrist_roll]
    T_final = fk_arm(full_theta)
    closest_position = T_final[:3, 3]
    pos_error = np.linalg.norm(closest_position - np.array(target_xyz))
    pitch_error = abs(_orientation_error_about_axis(
        T_final[:3, :3], R_target, _PITCH_AXIS))

    if pos_error > REACHABILITY_TOLERANCE_M or pitch_error > PITCH_TOLERANCE_RAD:
        raise ArmUnreachableTargetError(
            target_xyz, target_pitch, result.x, closest_position,
            pos_error, pitch_error)

    return result.x, pos_error, pitch_error


if __name__ == "__main__":
    # Round-trip test, same structure as ik_leg.py's: pick known angles,
    # run FK to get a target, run IK on that target, confirm FK-of-solved
    # reproduces the same position/pitch -- validates ik_arm() against
    # fk_arm() as independent ground truth (which was itself already
    # cross-checked against yourdfpy in fk_arm.py).
    test_angles_4 = [0.2, -0.3, 0.5, 0.1]
    test_wrist_roll = 0.4
    full_test = test_angles_4 + [test_wrist_roll]

    T_test = fk_arm(full_test)
    target_pos = T_test[:3, 3]
    R_test = T_test[:3, :3]
    # Recover the pitch that PRODUCED this target, for a fair round-trip
    # check (can't just invent an arbitrary pitch -- it has to be
    # consistent with test_angles_4 for this test to be meaningful).
    target_pitch_recovered = _orientation_error_about_axis(
        _R_HOME, R_test, _PITCH_AXIS)
    # Correct for the fact that _orientation_error_about_axis(A,B,axis)
    # gives the rotation FROM A TO B -- here we want home->test, matching
    # how ik_arm defines R_target = Ry(pitch) @ R_HOME.
    # Simplify: just brute-force search small pitch range for a match
    # instead of trusting the linearized error formula for a possibly
    # large angle.
    best_pitch, best_err = 0.0, 1e9
    for p in np.linspace(-2.0, 2.0, 4000):
        Rp = _rot_y(p) @ _R_HOME
        e = np.linalg.norm(Rp - R_test)
        if e < best_err:
            best_err, best_pitch = e, p
    print(f"Recovered pitch: {best_pitch:.4f} rad (match residual {best_err:.6f})")

    print(f"Target position: {target_pos}")
    print(f"Target pitch:    {best_pitch:.4f}")

    solved, pos_err, pitch_err = ik_arm(
        target_pos, best_pitch, test_wrist_roll,
        initial_guess=(0.0, 0.0, 0.0, 0.0))
    print(f"Solved angles:   {solved}")
    print(f"Position error:  {pos_err*1000:.4f} mm")
    print(f"Pitch error:     {np.degrees(pitch_err):.4f} deg")

    check_T = fk_arm(list(solved) + [test_wrist_roll])
    print(f"FK of solved:    {check_T[:3,3]}")
    print(f"Sub-mm match:    {np.allclose(check_T[:3,3], target_pos, atol=1e-3)}")

    print("\n--- Unreachable target test ---")
    far_target = (2.0, 0.0, 0.0)
    try:
        ik_arm(far_target, 0.0, 0.0, initial_guess=(0.0, 0.0, 0.0, 0.0))
        print("Unexpectedly reachable")
    except ArmUnreachableTargetError as e:
        print("Correctly flagged as unreachable.")
        print(f"  Closest position: {e.closest_position}")
        print(f"  Error: {e.pos_error_m*1000:.2f} mm")
