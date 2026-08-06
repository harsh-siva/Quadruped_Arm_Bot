import numpy as np
from fk_leg import fk_leg, rot_axis_angle, LEG_PARAMS
from ik_leg import ik_leg, UnreachableTargetError


def rpy_to_matrix(roll, pitch, yaw):
    """
    Build a single 3x3 rotation matrix representing a body orientation
    described by roll (rotation about x), pitch (about y), and yaw
    (about z), all in radians.

    We reuse rot_axis_angle from fk_leg.py for each individual axis instead
    of writing new x/y/z-specific formulas — roll/pitch/yaw ARE just
    rotations about the x, y, z axes respectively, so rot_axis_angle
    already knows how to build each one.

    Order matters when combining rotations (matrix multiplication is not
    commutative). We use the common convention: apply roll first, then
    pitch, then yaw — written right-to-left as Rz @ Ry @ Rx.
    """
    Rx = rot_axis_angle((1.0, 0.0, 0.0), roll)
    Ry = rot_axis_angle((0.0, 1.0, 0.0), pitch)
    Rz = rot_axis_angle((0.0, 0.0, 1.0), yaw)
    return Rz @ Ry @ Rx


def neutral_foot_positions_body_frame():
    """
    Compute where each leg's foot sits, in the shared BODY frame (relative
    to base_link), at the "neutral stance" — the URDF's rest pose, i.e. all
    joint angles = 0. This is our fixed reference: we treat these four
    points as "planted on the ground" and never moving, regardless of how
    the body itself later moves/tilts.

    Two-step calculation per leg:
      1. fk_leg(leg, 0, 0, 0) gives the foot position relative to THAT
         LEG'S OWN coxa joint (leg-local frame) — this is what fk_leg
         always returns.
      2. Adding hip_offset (that leg's coxa position relative to
         base_link) shifts the point into the shared body frame, so all
         four legs' foot positions can be compared/combined in one
         common coordinate system.

    Recomputed from scratch each call (rather than hardcoded once) so that
    if LEG_PARAMS is ever edited, this automatically stays correct.
    """
    neutral = {}
    for leg, p in LEG_PARAMS.items():
        foot_leg_local = fk_leg(leg, 0.0, 0.0, 0.0)
        neutral[leg] = p["hip_offset"] + foot_leg_local
    return neutral


def body_pose_to_leg_targets(dx, dy, dz, roll, pitch, yaw):
    """
    THE CORE TRANSFORM.

    Input: a desired body pose, relative to neutral —
           dx, dy, dz    = body translation (meters)
           roll,pitch,yaw = body rotation (radians)

    Output: {leg_name: target_xyz}, where each target_xyz is a foot
            position in THAT LEG'S OWN LOCAL FRAME — i.e. already in the
            exact format ik_leg() expects as input. No further conversion
            needed by the caller.

    Physical assumption behind the math: while standing, the feet stay
    fixed on the ground. It's the BODY that moves relative to the
    (unmoving) feet — not the other way around. So if we know where a
    foot sits in the body frame BEFORE any pose change (neutral_feet), we
    can work out where that same (still unmoving) foot appears to be,
    AFTER the body has moved, by applying the INVERSE of the body's
    motion to it.

    Why the inverse: imagine the body moves down by dz. From the body's
    own point of view, the (unmoving) foot appears to move UP by dz — the
    opposite direction. Rotations work the same way: if the body rolls
    one way, the foot appears to counter-roll from the body's perspective.
    This inverse relationship is why we use R.T (transpose = inverse, for
    rotation matrices) and subtract (not add) the translation.
    """
    R = rpy_to_matrix(roll, pitch, yaw)
    translation = np.array([dx, dy, dz])
    neutral_feet = neutral_foot_positions_body_frame()

    targets = {}
    for leg, p in LEG_PARAMS.items():
        foot_body_neutral = neutral_feet[leg]

        # Step 1: undo the body's translation (subtract dx,dy,dz), then
        # undo the body's rotation (multiply by R.T, the inverse rotation).
        # Result: this foot's position, as seen from the NEW posed body
        # frame, still expressed relative to base_link's origin.
        foot_body_posed = R.T @ (foot_body_neutral - translation)

        # Step 2: convert from "relative to base_link" back into
        # "relative to this leg's own coxa joint" — subtract hip_offset —
        # because that's the coordinate frame ik_leg() expects.
        foot_leg_target = foot_body_posed - p["hip_offset"]

        targets[leg] = foot_leg_target
    return targets


def body_pose_to_joint_angles(dx, dy, dz, roll, pitch, yaw):
    """
    Full pipeline, body pose all the way through to servo-ready joint
    angles:

        body pose --(body_pose_to_leg_targets)--> 4 leg-local targets
                  --(ik_leg, once per leg)-------> 4 joint-angle triples

    Returns: {leg_name: (theta_coxa, theta_femur, theta_tibia)}

    Note: if ANY leg's target is outside that leg's reachable workspace,
    ik_leg() raises UnreachableTargetError for that leg. We deliberately
    let it propagate up uncaught here — deciding what to DO about an
    unreachable leg (reject the whole pose? use the closest-reachable
    fallback for just that leg? something else?) is a real design
    decision we haven't made yet, not something to silently paper over.
    """
    targets = body_pose_to_leg_targets(dx, dy, dz, roll, pitch, yaw)
    angles = {}
    for leg, target in targets.items():
        solved_angles, err = ik_leg(leg, target, initial_guess=(0.0, 0.0, 0.0))
        angles[leg] = solved_angles
    return angles


if __name__ == "__main__":
    # Zero-pose sanity check: requesting "no change" (all zeros) must be a
    # no-op — the resulting per-leg targets should exactly equal the
    # neutral leg-local foot positions we'd get straight from fk_leg.
    # This isolates whether the transform machinery itself is wired
    # correctly, before testing any real (non-zero) pose.
    print("=== Zero-pose sanity check ===")
    zero_targets = body_pose_to_leg_targets(0, 0, 0, 0, 0, 0)
    for leg in LEG_PARAMS:
        neutral_leg_local = fk_leg(leg, 0.0, 0.0, 0.0)
        match = np.allclose(zero_targets[leg], neutral_leg_local, atol=1e-9)
        print(f"[{leg}] target={zero_targets[leg]}  neutral={neutral_leg_local}  match={match}")
