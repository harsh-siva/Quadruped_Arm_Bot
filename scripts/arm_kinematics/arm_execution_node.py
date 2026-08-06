#!/usr/bin/env python3
"""
CP4 (continued) -- Arm IK / Execution Node.

Sole owner of the ARM portion of /joint_command (the gait node remains
sole owner of the LEG portion -- both publish JointState messages naming
only the joints they command, confirmed safe live: `ros2 topic pub
/joint_command ... {name: ['shoulder_pan'], ...}` moved the arm joint
with zero interference, see progress_log.md CP4 notes).

Subscribes to:
  /arm_target     (std_msgs/Float64MultiArray, [dx, dy, dz, pitch, wrist_roll])
    from xbox_teleop_node.py -- OFFSETS from this node's own defined
    "home" end-effector pose (all-zero joint config), NOT absolute values.
  /gripper_target (std_msgs/Float64, radians) from xbox_teleop_node.py --
    direct target for the `gripper` (jaw) joint, already clamped to its
    real URDF range by the teleop node.

Publishes:
  /joint_command (sensor_msgs/JointState) -- 6 joint names: shoulder_pan,
  shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.

INTERNAL STATE, NOT SIM FEEDBACK (explicit decision, this chat): this
node has NO subscription to any real joint-state feedback from Isaac Sim
(no /joint_states topic exists yet -- confirmed via `ros2 topic list`).
It tracks its own belief of "current" joint angles internally, starting
at all-zero (URDF neutral, confirmed OK to assume as a starting point --
Harsh to verify visually once running), and updates that belief to
whatever it last COMMANDED, assuming commanded == achieved. Same
no-feedback-loop pattern the rest of this codebase already uses
(gait_generator_node.py's warm-started per-leg IK works the same way).

IK APPROACH: numerical (scipy.optimize.least_squares), warm-started from
this node's own last-solved angles each tick -- NOT solved from scratch,
specifically to avoid visible solution-branch flips (e.g. elbow-up vs
elbow-down) between one tick and the next, which a closed-form solver
could produce. See ik_arm.py for the full math and its own verification
notes (FK cross-checked against yourdfpy; IK round-trip self-tested
against that FK).
"""

import sys
import os
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fk_arm import fk_arm, JOINT_NAMES
from ik_arm import ik_arm, ArmUnreachableTargetError, SOLVED_JOINT_NAMES

CONTROL_RATE_HZ = 70.0  # matches xbox_teleop_node.py / gait_generator_node.py
DT = 1.0 / CONTROL_RATE_HZ

# Home end-effector position -- FK at the all-zero configuration. /arm_target's
# dx/dy/dz are offsets FROM this point (see xbox_teleop_node.py's CP4
# docstring section -- this node is the "downstream node" that section
# refers to as the one defining what "home" means).
_HOME_TRANSFORM = fk_arm([0.0, 0.0, 0.0, 0.0, 0.0])
HOME_POSITION = _HOME_TRANSFORM[:3, 3]

# Real, URDF-sourced gripper (jaw) joint limits -- same values
# xbox_teleop_node.py already clamps to; re-clamped here too (defensive,
# same "trust but verify upstream values" convention as
# gait_generator_node.py's own step-height re-clamp).
GRIPPER_MIN = -0.174533
GRIPPER_MAX = 1.74533


class ArmExecutionNode(Node):
    def __init__(self):
        super().__init__('arm_execution_node')

        # Internally tracked belief of joint state -- see module
        # docstring's "INTERNAL STATE, NOT SIM FEEDBACK" section.
        self.last_solved_4 = np.array([0.0, 0.0, 0.0, 0.0])  # shoulder_pan,
        # shoulder_lift, elbow_flex, wrist_flex -- warm-start seed.
        self.wrist_roll = 0.0
        self.gripper_angle = 0.0

        # Latest raw requests from the teleop node. Defaults represent
        # "no input yet received" -- home pose, neutral wrist_roll,
        # closed-ish-neutral gripper -- so this node behaves sanely even
        # if started before the teleop node.
        self.latest_arm_target = dict(dx=0.0, dy=0.0, dz=0.0, pitch=0.0, wrist_roll=0.0)
        self.latest_gripper_target = 0.0

        self.arm_target_sub = self.create_subscription(
            Float64MultiArray, '/arm_target', self.arm_target_callback, 10)
        self.gripper_target_sub = self.create_subscription(
            Float64, '/gripper_target', self.gripper_target_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)

        self.timer = self.create_timer(DT, self.control_loop)

        self.get_logger().info(
            f"Arm execution node started ({CONTROL_RATE_HZ} Hz). "
            f"Home end-effector position (arm base frame): {HOME_POSITION}")

    def arm_target_callback(self, msg):
        if len(msg.data) != 5:
            self.warn_once_bad_arm_target()
            return
        dx, dy, dz, pitch, wrist_roll = msg.data
        self.latest_arm_target = dict(dx=dx, dy=dy, dz=dz, pitch=pitch,
                                       wrist_roll=wrist_roll)

    def gripper_target_callback(self, msg):
        self.latest_gripper_target = msg.data

    def warn_once_bad_arm_target(self):
        # Defensive: /arm_target's field order is a hand-shake convention
        # with no message-level enforcement (same accepted fragility as
        # /body_pose -- see xbox_teleop_node.py's publish_body_pose()
        # docstring). If the array length is ever wrong, fail loudly
        # instead of silently unpacking garbage.
        self.get_logger().warn(
            "Received /arm_target with unexpected length -- ignoring "
            "this message, holding last known target.")

    def control_loop(self):
        t = self.latest_arm_target
        target_xyz = HOME_POSITION + np.array([t['dx'], t['dy'], t['dz']])
        target_pitch = t['pitch']
        # wrist_roll is DIRECTLY commanded, not solved -- see ik_arm.py's
        # module docstring for why. Defensive re-clamp (same "trust but
        # verify upstream" convention as elsewhere), even though
        # xbox_teleop_node.py already clamps this before publishing.
        self.wrist_roll = float(np.clip(t['wrist_roll'], -2.74385, 2.84121))

        try:
            solved_4, pos_err, pitch_err = ik_arm(
                target_xyz, target_pitch, self.wrist_roll,
                initial_guess=self.last_solved_4)
        except ArmUnreachableTargetError as e:
            # BEST-EFFORT FALLBACK POLICY (matches gait_generator_node.py's
            # per-leg policy, not xbox_teleop_node.py's reject-and-hold
            # policy): the arm target changes continuously while
            # teleoperating (same reasoning gait_generator_node.py uses
            # for its own targets), so absorbing a momentary near-limit
            # target with the closest achievable solution is preferable
            # to freezing the whole arm.
            self.get_logger().warn(
                f"Arm target unreachable this tick, using closest-approach "
                f"fallback: {e}")
            solved_4 = e.closest_angles

        self.last_solved_4 = np.array(solved_4)

        # Gripper (jaw) -- direct pass-through, defensively re-clamped.
        self.gripper_angle = float(
            np.clip(self.latest_gripper_target, GRIPPER_MIN, GRIPPER_MAX))

        self.publish_joint_command()

    def publish_joint_command(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = SOLVED_JOINT_NAMES + ['wrist_roll', 'gripper']
        msg.position = [float(a) for a in self.last_solved_4] + \
                        [self.wrist_roll, self.gripper_angle]
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = ArmExecutionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
