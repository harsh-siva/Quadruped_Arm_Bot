from pose_controller import body_pose_to_leg_targets, body_pose_to_joint_angles
from fk_leg import fk_leg, LEG_PARAMS
import numpy as np

print("=== Combined pose test: dz=-0.02, roll=0.1, pitch=0.1 ===")
targets = body_pose_to_leg_targets(0, 0, -0.02, 0.1, 0.1, 0)

# Approximate prediction: sum of each isolated-test z-change, per leg.
# These numbers come from the earlier isolated height/roll/pitch tests.
isolated_dz = {
    "FR": (0.02, -0.01541, -0.01606),
    "BR": (0.02, -0.01399, +0.01673),
    "FL": (0.02, +0.01924, -0.01747),
    "BL": (0.02, +0.01981, +0.01620),
}

for leg in LEG_PARAMS:
    neutral = fk_leg(leg, 0, 0, 0)
    actual = targets[leg]
    actual_dz = actual[2] - neutral[2]
    approx_dz = sum(isolated_dz[leg])
    diff = abs(actual_dz - approx_dz)
    print(f"[{leg}] actual_dz={actual_dz:+.5f}  naive_sum_approx={approx_dz:+.5f}  diff={diff:.5f}")
print()

print("=== Full pipeline through IK ===")
angles = body_pose_to_joint_angles(0, 0, -0.02, 0.1, 0.1, 0)
for leg, a in angles.items():
    check = fk_leg(leg, *a)
    target = targets[leg]
    match = np.allclose(check, target, atol=1e-6)
    print(f"[{leg}] solved={a}  FK_of_solved={check}  target={target}  match={match}")
