#!/usr/bin/env python3
"""
CP4(c) — Isaac Sim Validation of the Body Pose Controller

Extends CP4(b)'s validate_foot_position.py to validate ALL 4 legs at once
against a single commanded body pose (translation + roll/pitch/yaw),
instead of validating one leg's foot target at a time.

WHY THIS SCRIPT EXISTS / DESIGN DECISIONS (from chat discussion):

CP4(b) validated single-leg IK using C3 (a MEASURED mesh constant — the
farthest mesh vertex from the tibia joint origin) as an INDEPENDENT ground
truth for "predicted" foot position. Duplicating C3 here (not importing
it) mattered because it's a physical fact, separate from the FK/IK
algorithm being tested — if the algorithm and the validator both trusted
the SAME possibly-wrong number, the validation would be meaningless.

For a body POSE, there is no equivalent independent measured constant.
The "predicted" foot target for a posed body comes from
body_pose_to_leg_targets() — a coordinate TRANSFORM (rotation + 
translation), not a physical measurement. Hand-re-deriving that same
rotation math a second time inside this validator wouldn't add real
independence; it would just be retyping the same formula, with a real
risk of introducing an unrelated transcription bug. So here we IMPORT
body_pose_to_leg_targets() directly from pose_controller.py.

What STAYS independent, unchanged from CP4(b):
  - C3 (measured mesh offset, tibia joint -> foot tip) — duplicated here,
    not imported.
  - The TF/quaternion read-back from Isaac Sim — sim's actual reported
    pose is the real ground truth we're checking against.

HOW TO RUN:
  1. Isaac Sim open, robot loaded, CP1 ROS2 bridge (/joint_command) active.
  2. Compute joint angles for the desired body pose using
     pose_controller.body_pose_to_joint_angles(dx, dy, dz, roll, pitch, yaw),
     then command all 4 legs' angles via /joint_command (either one publish
     with all 12 joint names/positions, or four separate publishes).
  3. Wait ~1-2 seconds for the sim to physically settle into that pose.
  4. Run this script with the SAME pose values you just commanded, e.g.:
       python3 validate_body_pose.py 0.015 -0.01 -0.03 0 0 0
     (args are: dx dy dz roll pitch yaw — same order body_pose_to_leg_targets
     expects)
  5. Script prints, per leg: raw TF (translation + quaternion), the
     predicted foot position (from body_pose_to_leg_targets, converted to
     body frame), the actual foot position (from TF + C3), their
     difference, the error in mm, and pass/fail against the 2mm tolerance.
     Once all 4 legs have reported in, it prints an overall PASS/FAIL.
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# Make sure this script can `import pose_controller` regardless of which
# directory it's run from — insert this script's own folder
# (scripts/kinematics/) onto the Python import path.
sys.path.insert(0, __file__.rsplit('/', 1)[0])
from pose_controller import body_pose_to_leg_targets, LEG_PARAMS

# --------------------------------------------------------------------------
# C3: tibia joint -> foot tip offset, in the tibia's own local frame.
# Same values as LEG_PARAMS in fk_leg.py — DELIBERATELY DUPLICATED, not
# imported. This is the one piece of ground truth in this whole validation
# that is a MEASURED PHYSICAL FACT (from mesh geometry), not math we're
# trying to verify. Keeping it independent means: if pose_controller.py or
# fk_leg.py had a bug in how C3 gets used, this script wouldn't
# accidentally inherit and hide that bug.
# --------------------------------------------------------------------------
C3 = {
    "FR": np.array([0.012303, 0.041153, -0.145200]),
    "BR": np.array([0.041154, -0.012304, -0.145200]),
    "FL": np.array([-0.041154, 0.012304, -0.145200]),
    "BL": np.array([-0.008670, -0.042070, -0.145200]),
}

# Same standard used throughout the project (REACHABILITY_TOLERANCE_M in
# ik_leg.py) — 2mm placeholder, pending real hardware accuracy needs.
TOLERANCE_M = 0.002  # 2 mm

LEGS = ["FR", "BR", "FL", "BL"]


def quaternion_to_matrix(x, y, z, w):
    """
    Standard quaternion -> 3x3 rotation matrix conversion.
    TF reports link orientation as a quaternion; our C3 math needs a
    rotation matrix (same representation fk_leg.py's rot_axis_angle()
    produces). This is a pure representation conversion, not new physics —
    unchanged from CP4(b)'s validate_foot_position.py.
    """
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class BodyPoseValidator(Node):
    def __init__(self, pose, predicted_targets):
        """
        pose: the (dx, dy, dz, roll, pitch, yaw) tuple you commanded to
              the sim — kept around only so we can print it in the summary.
        predicted_targets: {leg: xyz} in LEG-LOCAL frame, exactly as
              body_pose_to_leg_targets() returns it (this is the format
              ik_leg() consumes).
        """
        super().__init__('body_pose_validator')
        self.pose = pose

        # ------------------------------------------------------------
        # CONVERT predicted targets from leg-local frame to BODY frame.
        #
        # body_pose_to_leg_targets() SUBTRACTS hip_offset at the very end
        # of its own pipeline, to convert its internal body-frame result
        # into the leg-local frame that ik_leg() needs as input. We're
        # not calling ik_leg() here — we need to compare against TF,
        # which reports base_link -> tibia_XX_1, i.e. BODY frame. So we
        # UNDO that last step by adding hip_offset back on.
        #
        # Concretely: if we skipped this and compared TF's actual_foot
        # (body frame) directly against the raw leg-local target, every
        # single leg would show a large "error" equal to that leg's own
        # hip_offset (~5-19cm depending on the leg) — not because the
        # pose math is wrong, but because we'd be comparing two numbers
        # expressed in different coordinate frames. This step makes sure
        # predicted and actual are apples-to-apples.
        # ------------------------------------------------------------
        self.predicted_body_frame = {
            leg: LEG_PARAMS[leg]["hip_offset"] + predicted_targets[leg]
            for leg in LEGS
        }

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Collects finished per-leg results as they come in. We don't
        # print anything until ALL 4 legs have reported — a body pose
        # validation isn't meaningful leg-by-leg, we need to see whether
        # the pose transform is self-consistent across the whole robot.
        self.results = {}

        # Poll every 1 second, same cadence as CP4(b)'s validator, giving
        # TF time to publish after the sim settles.
        self.timer = self.create_timer(1.0, self.try_lookup_all)

    def try_lookup_all(self):
        """
        Called once per second. Tries to look up TF for every leg that
        hasn't already succeeded. Once all 4 legs have valid TF data,
        prints the full summary and shuts down.
        """
        for leg in LEGS:
            # Skip legs we've already got data for — don't requery them,
            # only chase the stragglers.
            if leg in self.results:
                continue

            child_frame = f"tibia_{leg}_1"
            try:
                # time=Time() (t=0) asks for the LATEST available
                # transform, not a specific timestamp — fine for this
                # manual, "sim has settled into a fixed pose" check.
                t = self.tf_buffer.lookup_transform(
                    'base_link', child_frame, Time())
            except Exception:
                # TF for this leg isn't available yet — try again next
                # timer tick. Not an error, just "not ready."
                self.get_logger().info(f"[{leg}] TF not ready yet, retrying...")
                continue

            tx = t.transform.translation
            rq = t.transform.rotation
            translation = np.array([tx.x, tx.y, tx.z])

            # Same core computation as CP4(b): actual foot position =
            # tibia's real translation + (tibia's real rotation applied
            # to the measured C3 offset).
            R_actual = quaternion_to_matrix(rq.x, rq.y, rq.z, rq.w)
            actual_foot = translation + R_actual @ C3[leg]

            predicted = self.predicted_body_frame[leg]
            diff = actual_foot - predicted
            error_m = np.linalg.norm(diff)

            # Store everything we might want to print later, not just
            # the final error — so the summary can show raw TF data too,
            # same "don't just trust a verdict, show the inputs" approach
            # as CP4(b).
            self.results[leg] = dict(
                translation=translation,
                quat=(rq.x, rq.y, rq.z, rq.w),
                actual=actual_foot,
                predicted=predicted,
                diff=diff,
                error_m=error_m,
            )

        # Only proceed once every leg has reported in.
        if len(self.results) == len(LEGS):
            self.timer.cancel()
            self.print_summary()
            rclpy.shutdown()

    def print_summary(self):
        """Prints per-leg diagnostics, then one overall PASS/FAIL line."""
        dx, dy, dz, roll, pitch, yaw = self.pose
        print(f"\n=== Body pose commanded: dx={dx} dy={dy} dz={dz} "
              f"roll={roll} pitch={pitch} yaw={yaw} ===")

        all_pass = True
        for leg in LEGS:
            r = self.results[leg]
            passed = r["error_m"] <= TOLERANCE_M
            all_pass &= passed

            print(f"\n--- {leg} ---")
            print(f"raw TF translation = {r['translation']}")
            print(f"raw TF quaternion  = {r['quat']}")
            print(f"predicted (body frame) = {r['predicted']}")
            print(f"actual    (body frame) = {r['actual']}")
            print(f"difference               = {r['diff']}")
            print(f"error (mm)                = {r['error_m']*1000:.3f}")
            print(f"within {TOLERANCE_M*1000:.0f}mm tolerance: {passed}")

        print(f"\n=== Overall: {'PASS' if all_pass else 'FAIL'} "
              f"(all 4 legs within {TOLERANCE_M*1000:.0f}mm) ===")


def main():
    if len(sys.argv) != 7:
        print("Usage: python3 validate_body_pose.py dx dy dz roll pitch yaw")
        print("  dx, dy, dz    : body translation in meters")
        print("  roll,pitch,yaw: body rotation in radians")
        sys.exit(1)

    pose = tuple(float(a) for a in sys.argv[1:7])
    dx, dy, dz, roll, pitch, yaw = pose

    # Compute predicted leg-local targets ONCE, up front, for all 4 legs —
    # this replaces CP4(b)'s workflow where predicted values had to be
    # copy-pasted per leg by hand.
    predicted_targets = body_pose_to_leg_targets(dx, dy, dz, roll, pitch, yaw)

    rclpy.init()
    node = BodyPoseValidator(pose, predicted_targets)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
