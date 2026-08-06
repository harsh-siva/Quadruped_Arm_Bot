import numpy as np

def rot_axis_angle(axis, theta):
    """
    Build a 3x3 rotation matrix for rotating by 'theta' radians about an
    arbitrary unit axis, using Rodrigues' rotation formula.

    Why we need this (instead of separate x/y/z rotation functions):
    each joint in this robot rotates about its OWN axis, which is not
    always simply x, y, or z (e.g. femur/tibia axes are diagonal, like
    (-0.707107, -0.707107, 0.0)). This one function handles any axis.

    axis  : (x, y, z) tuple/array — direction of the rotation axis.
            Does not need to be pre-normalized; we normalize it below.
    theta : rotation angle in radians.
    Returns: 3x3 numpy rotation matrix.
    """
    axis = np.array(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)   # ensure axis is unit length
    x, y, z = axis
    c, s = np.cos(theta), np.sin(theta)
    C = 1 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s,  x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,    y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s,  c + z*z*C]
    ])

# ============================================================================
# LEG_PARAMS: the fixed geometry of the robot, one entry per leg.
# These numbers come directly from measuring the URDF / meshes — they are
# NOT tunable design choices, they describe the physical robot as built.
#
#   coxa_axis, femur_axis, tibia_axis:
#       The rotation axis of each of the 3 joints in a leg, taken from the
#       URDF <axis xyz="..."> tags. Coxa always rotates about world-Z
#       (straight up), but femur/tibia axes are tilted differently per leg
#       because each leg is mounted at a different angle around the body.
#
#   C1, C2, C3:
#       Fixed offset vectors between consecutive joints, each expressed in
#       the LOCAL frame of the joint that comes right before it, measured
#       at that joint's angle = 0:
#         C1 = coxa joint  -> femur joint  (in coxa's local frame)
#         C2 = femur joint -> tibia joint  (in femur's local frame)
#         C3 = tibia joint -> foot tip     (in tibia's local frame)
#       C1/C2 come from the URDF joint <origin xyz="..."> tags.
#       C3 does NOT come from the URDF (there's no URDF "foot" joint) — it
#       was computed by finding the farthest mesh vertex from the tibia
#       joint origin, in the tibia's own local frame (i.e. "where does the
#       physical foot tip sit, relative to the last joint").
#
#   hip_offset:
#       Position of this leg's coxa joint relative to base_link (the body's
#       own origin), taken directly from the URDF's J_Coxa_* <origin
#       xyz="..."> tag. This is NEW compared to the CP3/CP4(a) version of
#       this file — CP3/CP4(a) only needed per-leg-local math, but the body
#       pose controller (CP4 main deliverable) needs to know where each
#       leg's coxa sits relative to the shared body frame, so it can figure
#       out how the body moving affects each leg differently.
#
#   bounds_lower, bounds_upper:
#       Joint angle limits (radians) per joint [coxa, femur, tibia], taken
#       from the URDF <limit lower="..." upper="..."/> tags.
#       IMPORTANT: these are NOT the same across all four legs. FR/BR coxa
#       bounds are symmetric (+-1.308997 rad), but FL and BL are asymmetric
#       AND differ from each other. Always look these up per-leg — never
#       assume one leg's bounds apply to another.
# ============================================================================
LEG_PARAMS = {
    "FR": dict(
        coxa_axis=(0.0, 0.0, 1.0),
        femur_axis=(-0.707107, -0.707107, 0.0),
        tibia_axis=(0.707107, 0.707107, 0.0),
        C1=np.array([-0.055296, 0.015981, -0.0141]),
        C2=np.array([-0.049243, 0.049244, -0.01866]),
        C3=np.array([0.012303, 0.041153, -0.145200]),
        bounds_lower=np.array([-1.308997, -1.570796, -1.308997]),
        bounds_upper=np.array([ 1.308997,  0.785398,  1.570796]),
        hip_offset=np.array([-0.0749, 0.0542, 0.0528]),
    ),
    "BR": dict(
        coxa_axis=(0.0, 0.0, 1.0),
        femur_axis=(-0.707107, 0.707107, 0.0),
        tibia_axis=(0.707107, -0.707107, 0.0),
        C1=np.array([0.015981, 0.055296, -0.0141]),
        C2=np.array([0.049243, 0.049244, -0.01866]),
        C3=np.array([0.041154, -0.012304, -0.145200]),
        bounds_lower=np.array([-1.308997, -1.570796, -1.308997]),
        bounds_upper=np.array([ 1.308997,  0.785398,  1.570796]),
        hip_offset=np.array([0.0549, 0.0542, 0.0528]),
    ),
    "FL": dict(
        coxa_axis=(0.0, 0.0, 1.0),
        femur_axis=(0.707107, -0.707107, 0.0),
        tibia_axis=(-0.707107, 0.707107, 0.0),
        C1=np.array([-0.015981, -0.055296, -0.0141]),
        C2=np.array([-0.049243, -0.049244, -0.01866]),
        C3=np.array([-0.041154, 0.012304, -0.145200]),
        bounds_lower=np.array([-1.047198, -1.570796, -1.308997]),
        bounds_upper=np.array([ 1.570796,  0.785398,  1.570796]),
        hip_offset=np.array([-0.0749, -0.0942, 0.0528]),
    ),
    "BL": dict(
        coxa_axis=(0.0, 0.0, 1.0),
        femur_axis=(0.642788, 0.766044, 0.0),
        tibia_axis=(-0.642788, -0.766044, 0.0),
        C1=np.array([0.056478, -0.011101, -0.0141]),
        C2=np.array([0.053348, -0.044764, -0.01866]),
        C3=np.array([-0.008670, -0.042070, -0.145200]),
        bounds_lower=np.array([-1.396263, -1.570796, -1.308997]),
        bounds_upper=np.array([ 1.221730,  0.785398,  1.570796]),
        hip_offset=np.array([0.0549, -0.0942, 0.0528]),
    ),
}

def fk_leg(leg, theta_coxa, theta_femur, theta_tibia):
    """
    Forward kinematics: given a leg name and its 3 joint angles, compute
    where the foot ends up, relative to that leg's OWN coxa joint origin
    (i.e. "leg-local" coordinates, NOT relative to the robot body/base_link).

    leg: one of 'FR', 'BR', 'FL', 'BL'.
    theta_coxa, theta_femur, theta_tibia: joint angles in radians.

    How it works — chain the joints together one at a time:
      1. Rotate C1 (coxa->femur offset) by the coxa's current rotation.
         This gives femur_pivot: where the femur joint physically is now.
      2. Rotate C2 (femur->tibia offset) by BOTH the coxa's rotation and
         the femur's rotation (rotations compose by matrix multiplication,
         applied in chain order: Rc @ Rf), then add it on top of
         femur_pivot. This gives tibia_pivot.
      3. Same idea for C3 (tibia->foot offset), composing all three
         rotations (Rc @ Rf @ Rt), added on top of tibia_pivot. This gives
         the final foot position.
    This mirrors exactly how the physical leg is built: each joint's motion
    also carries along everything attached further down the chain.
    """
    p = LEG_PARAMS[leg]
    Rc = rot_axis_angle(p["coxa_axis"], theta_coxa)
    Rf = rot_axis_angle(p["femur_axis"], theta_femur)
    Rt = rot_axis_angle(p["tibia_axis"], theta_tibia)

    femur_pivot = Rc @ p["C1"]
    tibia_pivot = femur_pivot + Rc @ Rf @ p["C2"]
    foot        = tibia_pivot + Rc @ Rf @ Rt @ p["C3"]
    return foot

if __name__ == "__main__":
    # Sanity check: at all-zero joint angles, every rotation matrix is the
    # identity matrix (rotating by 0 radians = no rotation), so fk_leg should
    # reduce to simply C1 + C2 + C3 added together. This confirms the chain
    # math above is wired correctly, independent of the rotation logic.
    for leg, p in LEG_PARAMS.items():
        foot0 = fk_leg(leg, 0.0, 0.0, 0.0)
        expected = p["C1"] + p["C2"] + p["C3"]
        print(f"[{leg}] Foot at (0,0,0):   {foot0}")
        print(f"[{leg}] Expected (C1+C2+C3): {expected}")
        print(f"[{leg}] Match: {np.allclose(foot0, expected)}")
        print(f"[{leg}] Distance from coxa origin at rest: {np.linalg.norm(foot0):.6f} m")
        print()
