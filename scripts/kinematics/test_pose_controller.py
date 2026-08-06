from pose_controller import body_pose_to_leg_targets, body_pose_to_joint_angles
from fk_leg import fk_leg, LEG_PARAMS
import numpy as np

print("=== Pure height drop test: dz=-0.02, no rotation ===")
targets = body_pose_to_leg_targets(0, 0, -0.02, 0, 0, 0)
for leg in LEG_PARAMS:
    neutral = fk_leg(leg, 0, 0, 0)
    predicted = neutral.copy()
    predicted[2] += 0.02   # hand-derived prediction: target_z = neutral_z + 0.02
    actual = targets[leg]
    match = np.allclose(actual, predicted, atol=1e-9)
    print(f"[{leg}] neutral  ={neutral}")
    print(f"[{leg}] predicted={predicted}")
    print(f"[{leg}] actual   ={actual}")
    print(f"[{leg}] matches hand prediction: {match}")
    print()

print("=== Full pipeline test: same pose, through IK ===")
angles = body_pose_to_joint_angles(0, 0, -0.02, 0, 0, 0)
for leg, a in angles.items():
    check = fk_leg(leg, *a)
    print(f"[{leg}] solved angles={a}")
    print(f"[{leg}] FK of solved ={check}  (should match target above)")
    print()
