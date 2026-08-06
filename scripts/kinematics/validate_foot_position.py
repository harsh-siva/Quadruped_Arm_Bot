#!/usr/bin/env python3
"""
CP4(b) — Isaac Sim Validation of 4-Leg IK
Read-back script: compares PREDICTED foot position (from our fk_leg.py /
LEG_PARAMS math) against ACTUAL foot position (derived from Isaac Sim's
real TF data for the tibia link).

WHY THIS SCRIPT EXISTS (see chat discussion):
Our URDF has no dedicated "foot" link/joint — the last real link per leg is
tibia_XX_1. The foot tip is a fixed offset (C3, from LEG_PARAMS) measured
from the tibia joint origin, computed from mesh geometry, not from the URDF
directly. So to get the ACTUAL foot position from the sim, we can't just
look up a foot frame in TF — we have to:
  1. Get the tibia link's REAL pose (translation + rotation) relative to
     base_link, as Isaac Sim reports it via TF.
  2. Apply that REAL rotation + translation to the (trusted, geometry-
     derived) C3 offset ourselves.
This checks whether the URDF's actual joint origins/axes, as Isaac Sim
interprets them, agree with what LEG_PARAMS assumes -- which is exactly
what CP4(b) is supposed to validate.

HOW TO RUN:
  1. Make sure Isaac Sim is open, the robot is loaded, and the CP1 ROS2
     bridge (/joint_command) is active (confirmed working in this chat).
  2. Command the desired joint angles for ONE test case via:
       ros2 topic pub /joint_command sensor_msgs/msg/JointState \
         "{name: [...], position: [...]}" --once
     (use the angles from our test case table for that case)
  3. Wait ~1-2 seconds for the sim to physically settle into that pose.
  4. Run this script with the leg name and that case's PREDICTED foot
     position (body frame) as arguments, e.g.:
       python3 validate_foot_position.py FR -0.166436 0.106378 -0.15796
     (predicted = hip_offset + FK_check, from our test case table)
  5. Script prints: actual TF data (raw), actual foot position it computes,
     predicted foot position (what you passed in), and the difference.

This script does NOT trust the sim blindly -- it prints the raw TF
translation/quaternion it read, so you can see the real input, not just
a final match/no-match verdict.
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

# Same C3 (tibia joint -> foot tip) values as LEG_PARAMS in fk_leg.py.
# Duplicated here (not imported) so this script has zero dependency on
# your kinematics code being on the ROS2 Python path -- keeps the sim-side
# validation tool independent from the thing it's validating.
C3 = {
    "FR": np.array([0.012303, 0.041153, -0.145200]),
    "BR": np.array([0.041154, -0.012304, -0.145200]),
    "FL": np.array([-0.041154, 0.012304, -0.145200]),
    "BL": np.array([-0.008670, -0.042070, -0.145200]),
}

# Reachability-style tolerance, matching REACHABILITY_TOLERANCE_M in
# ik_leg.py, so "does this pass" uses the same standard you've already
# been using elsewhere in this project.
TOLERANCE_M = 0.002  # 2 mm


def quaternion_to_matrix(x, y, z, w):
    """
    Standard quaternion -> 3x3 rotation matrix conversion.
    TF reports orientation as a quaternion; our C3 math needs a rotation
    matrix (same representation fk_leg.py's rot_axis_angle() produces).
    This is a pure representation conversion -- not new physics.
    """
    n = np.sqrt(x*x + y*y + z*z + w*w)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class FootValidator(Node):
    def __init__(self, leg, predicted_foot):
        super().__init__('foot_position_validator')
        self.leg = leg
        self.predicted_foot = np.array(predicted_foot)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Give TF a moment to receive at least one message before we try
        # to look anything up.
        self.timer = self.create_timer(1.0, self.try_lookup)
        self.done = False

    def try_lookup(self):
        child_frame = f"tibia_{self.leg}_1"
        try:
            # time=Time() (i.e. t=0) asks TF2 for the LATEST available
            # transform rather than a specific timestamp -- fine for this
            # kind of manual, settled-pose check.
            t = self.tf_buffer.lookup_transform(
                'base_link', child_frame, Time())
        except Exception as e:
            self.get_logger().info(f"TF not ready yet ({e}), retrying...")
            return

        self.done = True
        self.timer.cancel()

        tx = t.transform.translation
        rq = t.transform.rotation
        translation = np.array([tx.x, tx.y, tx.z])

        print(f"\n=== Raw TF: base_link -> {child_frame} ===")
        print(f"translation = ({tx.x:.6f}, {tx.y:.6f}, {tx.z:.6f})")
        print(f"quaternion  = (x={rq.x:.6f}, y={rq.y:.6f}, "
              f"z={rq.z:.6f}, w={rq.w:.6f})")

        R_actual = quaternion_to_matrix(rq.x, rq.y, rq.z, rq.w)
        actual_foot = translation + R_actual @ C3[self.leg]

        diff = actual_foot - self.predicted_foot
        error_m = np.linalg.norm(diff)

        print(f"\n=== Foot position comparison [{self.leg}] ===")
        print(f"predicted (body frame) = {self.predicted_foot}")
        print(f"actual    (body frame) = {actual_foot}")
        print(f"difference              = {diff}")
        print(f"error (m)                = {error_m:.6f}  "
              f"({error_m*1000:.3f} mm)")
        print(f"within {TOLERANCE_M*1000:.0f}mm tolerance: "
              f"{error_m <= TOLERANCE_M}")

        rclpy.shutdown()


def main():
    if len(sys.argv) != 5:
        print("Usage: python3 validate_foot_position.py LEG px py pz")
        print("  LEG: FR, BR, FL, or BL")
        print("  px py pz: predicted foot position (body frame, meters)")
        sys.exit(1)

    leg = sys.argv[1]
    predicted = (float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))

    rclpy.init()
    node = FootValidator(leg, predicted)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
