#!/usr/bin/env python3
"""
CP6f side-check -- IK timing measurement.

WHY THIS EXISTS: before locking in a control-loop rate for the CP6 gait
node (which solves 4-leg IK every tick), we want a REAL measured number
for how long that actually takes on Harsh's machine, rather than assuming
it fits inside a 50Hz (20ms) or 30Hz (33ms) budget. This is a pure
timing test -- it does NOT test correctness (that's already covered by
ik_leg.py's own round-trip tests) and does NOT require ROS2, Isaac Sim,
or any hardware. Just plain Python + the existing kinematics files.

HOW TO RUN:
  Run this from the SAME folder as pose_controller.py, fk_leg.py, and
  ik_leg.py (e.g. scripts/kinematics/), so the imports below resolve:

    python3 timing_check_ik.py

WHAT IT MEASURES:
  Calls body_pose_to_joint_angles() (the exact function both the teleop
  node's reachability-check and the future gait node's IK-solve step
  use -- so this is measuring the real cost, not a synthetic stand-in)
  repeatedly with varied, non-trivial poses, and reports:
    - average time per call (i.e. per tick, solving all 4 legs)
    - worst-case (max) time per call, which matters more than average
      for a real-time control loop -- a loop that's fast on average but
      occasionally spikes above the tick budget will still stutter
    - what that implies for 30Hz (33.3ms) vs 50Hz (20ms) budgets
"""

import time
import numpy as np
from pose_controller import body_pose_to_joint_angles
from ik_leg import UnreachableTargetError

N_TRIALS = 200

# Vary the pose each trial (small sinusoidal sweep across a realistic
# range) rather than calling with the exact same pose every time -- a
# numerical optimizer's iteration count can depend on how far the target
# is from the initial guess, so a single fixed pose could give a
# misleadingly optimistic (or pessimistic) number.
def pose_for_trial(i):
    t = i / N_TRIALS * 2 * np.pi
    return dict(
        dx=0.02 * np.sin(t),
        dy=0.02 * np.cos(t * 1.3),
        dz=-0.01 + 0.01 * np.sin(t * 0.7),
        roll=0.05 * np.sin(t * 1.7),
        pitch=0.05 * np.cos(t * 0.9),
        yaw=0.05 * np.sin(t * 1.1),
    )

def main():
    times = []
    failures = 0

    # One "warm-up" call, uncounted -- Python/numpy/scipy can have
    # one-time import/JIT-ish overhead on the very first call that
    # wouldn't be representative of steady-state control-loop cost.
    _ = body_pose_to_joint_angles(**pose_for_trial(0))

    for i in range(N_TRIALS):
        pose = pose_for_trial(i)
        start = time.perf_counter()
        try:
            body_pose_to_joint_angles(**pose)
        except UnreachableTargetError:
            # Shouldn't happen at these small pose magnitudes, but if it
            # does, don't let it silently corrupt the timing -- count and
            # skip it rather than pretending it succeeded.
            failures += 1
            continue
        elapsed = time.perf_counter() - start
        times.append(elapsed)

    times = np.array(times)
    print(f"=== IK timing check ({len(times)} successful calls, "
          f"{failures} unreachable/skipped) ===")
    print(f"mean   : {times.mean()*1000:.3f} ms per call (4-leg solve)")
    print(f"median : {np.median(times)*1000:.3f} ms per call")
    print(f"max    : {times.max()*1000:.3f} ms per call  <-- worst case, matters most")
    print(f"min    : {times.min()*1000:.3f} ms per call")
    print()

    budget_30hz = 1000.0 / 30.0
    budget_50hz = 1000.0 / 50.0
    print(f"30Hz tick budget = {budget_30hz:.2f} ms  -- "
          f"{'OK on worst case' if times.max()*1000 < budget_30hz else 'WOULD EXCEED on worst case'}")
    print(f"50Hz tick budget = {budget_50hz:.2f} ms  -- "
          f"{'OK on worst case' if times.max()*1000 < budget_50hz else 'WOULD EXCEED on worst case'}")
    print()
    print("NOTE: this measures body_pose_to_joint_angles() alone, i.e. "
          "the IK-solving cost only. The real control loop also does "
          "gait-offset math, smoothing, and message pub/sub each tick -- "
          "small compared to IK, but not exactly zero. Treat this as the "
          "dominant cost, not the ENTIRE tick cost.")

if __name__ == "__main__":
    main()
