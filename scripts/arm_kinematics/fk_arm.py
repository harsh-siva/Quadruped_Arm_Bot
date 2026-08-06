import numpy as np

# =============================================================================
# VERIFICATION NOTE (read this before trusting/changing anything below):
# This file's math was cross-checked against yourdfpy (an independent,
# third-party URDF-parsing library) BEFORE being used anywhere else --
# same "verify against a known-good reference" instinct as ik_leg.py's own
# round-trip test at the bottom of this file. Checked at TWO configurations
# (all-zero, and an arbitrary non-zero test pose) -- both position AND full
# rotation matrix matched yourdfpy's output exactly (not approximately --
# bit-identical at printed precision). Re-run the __main__ block below if
# this file is ever edited, to catch any regression the same way.
# =============================================================================


def rpy_to_matrix(roll, pitch, yaw):
    """
    URDF's <origin rpy="..."/> convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    (fixed-axis / extrinsic X-Y-Z composition). Unlike fk_leg.py's legs
    (whose per-joint axis vectors already had any such rotation folded in,
    because each leg's axis was hand-measured directly), the SO-101 URDF
    (auto-generated from CAD) gives every joint's axis as the same simple
    (0,0,1) but ROTATES THE FRAME ITSELF via a non-trivial <origin rpy>
    first -- so this conversion is required here in a way it wasn't for
    the legs.
    """
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_z(theta):
    """Every SO-101 joint's own axis is (0,0,1) IN ITS OWN (already-rotated
    by origin rpy) local frame -- so the joint's own contribution is always
    just a simple Z rotation, applied AFTER the fixed origin rotation."""
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def homog(R, t):
    """Pack a 3x3 rotation + 3-vector translation into a 4x4 homogeneous
    transform."""
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


# =============================================================================
# ARM_PARAMS: fixed geometry of the SO-101's 5-joint positioning chain,
# taken DIRECTLY from so101_new_calib.urdf's <origin xyz="..." rpy="..."/>
# tags -- NOT measured/guessed. All 5 joints are revolute about a LOCAL
# (0,0,1), per URDF (see rpy_to_matrix's docstring for why the rpy still
# matters despite that).
#
# Ordered base -> tip: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
# wrist_roll. Does NOT include the `gripper` joint (the jaw) -- that's a
# SIBLING branch off gripper_link, not part of the chain to the end
# effector (gripper_frame_link), confirmed directly from the URDF's
# parent/child links. Jaw open/close is handled entirely separately (see
# xbox_teleop_node.py's /gripper_target).
# =============================================================================
ARM_JOINTS = [
    # (name, origin_xyz, origin_rpy, bound_lower, bound_upper)
    ("shoulder_pan",  (0.0388353, -8.97657e-09, 0.0624),
     (3.14159, 4.18253e-17, -3.14159), -1.91986, 1.91986),
    ("shoulder_lift", (-0.0303992, -0.0182778, -0.0542),
     (-1.5708, -1.5708, 0.0), -1.74533, 1.74533),
    ("elbow_flex",    (-0.11257, -0.028, 1.73763e-16),
     (-3.63608e-16, 8.74301e-16, 1.5708), -1.69, 1.69),
    ("wrist_flex",    (-0.1349, 0.0052, 3.62355e-17),
     (4.02456e-15, 8.67362e-16, -1.5708), -1.65806, 1.65806),
    ("wrist_roll",    (5.55112e-17, -0.0611, 0.0181),
     (1.5708, 0.0486795, 3.14159), -2.74385, 2.84121),
]

# Final FIXED transform from gripper_link (wrist_roll's child) to
# gripper_frame_link (the actual end-effector / TCP frame CP3 mounted the
# camera on) -- gripper_frame_joint in the URDF, type="fixed".
GRIPPER_FRAME_FIXED_XYZ = (-0.0079, -0.000218121, -0.0981274)
GRIPPER_FRAME_FIXED_RPY = (0.0, 3.14159, 0.0)

JOINT_NAMES = [j[0] for j in ARM_JOINTS]
BOUNDS_LOWER = np.array([j[3] for j in ARM_JOINTS])
BOUNDS_UPPER = np.array([j[4] for j in ARM_JOINTS])


def fk_arm(theta):
    """
    Forward kinematics: given the 5 joint angles (shoulder_pan,
    shoulder_lift, elbow_flex, wrist_flex, wrist_roll, IN THAT ORDER),
    return the 4x4 homogeneous transform of gripper_frame_link relative
    to the arm's own base_link.

    Chains joints exactly as the physical arm is built: each joint's
    transform is (fixed origin translation+rotation from the URDF) THEN
    (that joint's own rotation about its local Z), composed in order from
    base to tip -- same "carry everything downstream along with you"
    principle as fk_leg.py, just using full 4x4 transforms instead of
    fk_leg's simplified pre-rotated-axis shortcut, because this URDF's
    per-joint origin rotations are non-trivial (see rpy_to_matrix).
    """
    T = np.eye(4)
    for (name, xyz, rpy, lo, hi), th in zip(ARM_JOINTS, theta):
        T_origin = homog(rpy_to_matrix(*rpy), np.array(xyz))
        T_joint = homog(rot_z(th), np.zeros(3))
        T = T @ T_origin @ T_joint
    T_tip = homog(rpy_to_matrix(*GRIPPER_FRAME_FIXED_RPY),
                   np.array(GRIPPER_FRAME_FIXED_XYZ))
    return T @ T_tip


if __name__ == "__main__":
    # Cross-check against yourdfpy (independent URDF parser) -- run
    # `pip install yourdfpy --break-system-packages` first if it's not
    # already installed. This is the verification referenced in the note
    # at the top of this file.
    import yourdfpy

    robot = yourdfpy.URDF.load(
        "../../ros2_ws/src/so101_description/so101_new_calib.urdf")

    test_configs = [
        [0.0, 0.0, 0.0, 0.0, 0.0],
        [0.3, -0.5, 0.9, 0.2, -1.1],
    ]
    for cfg_list in test_configs:
        cfg = dict(zip(JOINT_NAMES, cfg_list))
        robot.update_cfg(cfg)
        T_ref = robot.get_transform(
            frame_to="gripper_frame_link", frame_from="base_link")
        T_mine = fk_arm(cfg_list)
        pos_match = np.allclose(T_ref[:3, 3], T_mine[:3, 3], atol=1e-6)
        rot_match = np.allclose(T_ref[:3, :3], T_mine[:3, :3], atol=1e-6)
        print(f"config={cfg_list}")
        print(f"  position match: {pos_match}  rotation match: {rot_match}")
        if not (pos_match and rot_match):
            print(f"  MISMATCH -- mine:\n{T_mine}\n  ref:\n{T_ref}")
