from pose_controller import body_pose_to_joint_angles
angles = body_pose_to_joint_angles(0.01, 0.01, -0.025, 0.08, -0.08, 0.1)
for leg, a in angles.items():
    print(leg, a)