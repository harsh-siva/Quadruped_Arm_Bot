# Quadruped_Arm_Bot

A bio-inspired quadruped robot fitted with a small robotic arm and end-effector camera, built in **NVIDIA Isaac Sim + ROS2**, aimed at collecting synchronized multimodal demonstration data for a small vision-based manipulation policy (VLA-style pipeline).

This project extends an earlier quadruped teleop platform ([Bio-Inspired Quadruped Spiderbot](https://harsh-siva.framer.website/projects/bio-inspired-autonomous-spiderbot-(on-going))) by re-mounting a sensorless, flat-top version of the chassis with a robotic arm, so the same walking robot can also see and manipulate objects.

https://github.com/user-attachments/assets/c0f81288-0462-474b-ba61-b59d8fef8450

---

## Project Status

| Checkpoint | Description | Status |
|---|---|---|
| CP1 | Reproduce quadruped teleop pipeline on new arm-ready URDF | ✅ Complete |
| CP2 | Select and mount a small robotic arm | ✅ Complete |
| CP3 | End-effector camera | ✅ Complete |
| CP4 | Arm teleop node | ✅ Complete |
| CP5 | Synchronized data collection pipeline | Functional (built & verified, no manipulable objects yet) |
| CP6 | Add scene objects for pick-and-place | ✅ Complete |
| CP7 | Small policy training (imitation learning) | ⬜ Not started |
| CP8 | Policy execution / evaluation | ⬜ Not started |

The robot currently walks, has a working teleoperated arm with an eye-in-hand camera, and can record synchronized episodes of camera + joint data to disk. It can now grasp objects in the simulated scene, though gripper physics tuning and the full pick-**and-place** success test (placing the object elsewhere, and repeating on all 3 cubes) are still in progress.

---

## What's Been Built

### 1. Quadruped base (CP1)
- New URDF variant (`Spiderbot_Without_Nav_description`) — sensors and LiDAR removed, top plate flattened to make room for the arm.
- Full teleop pipeline ported from the original Spiderbot project and re-verified against the new URDF: 4-leg IK/FK, body pose control (roll/pitch/yaw/height), gait generator, Xbox teleop.
- Rebuilt the OmniGraph joint-command bridge (ROS2 ⇄ Isaac Sim articulation controller) from scratch for the new robot.
- Verified full walking parity with the original robot.

### 2. Arm integration (CP2)
- Evaluated Isaac Sim's default arm library against the chassis's real mass/footprint (pulled from the URDF and mesh data).
- Selected the **SO-ARM100 (SO101 variant)** — small, sim-ready, actively maintained.
- Diagnosed and fixed a mounting bug where the arm was only visually parented to the chassis but not physically constrained to it (confirmed via Isaac Sim's Physics Inspector, fixed by re-pointing the arm's root joint to the chassis body).

### 3. End-effector camera (CP3)
- Real Isaac Sim `Camera` prim mounted on the gripper's fixed reference frame (not the moving jaw, so image pose stays consistent as the gripper opens/closes).
- Streamed over ROS2 (`/rgb`, `/camera_info`) at 224×224 resolution, chosen with future policy input in mind.
- End-to-end verified: render product generation, live topic publishing (~21–24 Hz), correct image content, and valid camera intrinsics.

### 4. Arm teleop (CP4)
- Second Xbox control mode (toggle button) that hands the sticks/triggers over to the arm and freezes the base.
- Fully-determined 5-DOF control scheme: 3 translational targets (dx/dy/dz), pitch, and wrist roll, plus an independent gripper jaw command.
- Custom forward/inverse kinematics for the 5-joint arm chain, cross-checked against an independent third-party URDF parser and round-trip self-tested.
- Reach limits empirically derived from real IK sweeps (not guessed), including a combined-axis sweep to find the arm's true reachable envelope.

### 5. Data collection pipeline (CP5)
- ROS2 node recording synchronized episodes to HDF5: RGB frames, joint positions, timestamps, and a per-episode language instruction label.
- Tooling to sanity-check recordings: contact-sheet image, joint-trajectory plot, and MP4 playback per episode.
- Several real bugs found and fixed during testing, including a race condition between the recording thread and the ROS2 callback thread, and a hardcoded image-shape bug caught by comparing against the real camera output.
- 3 verified test episodes recorded — real, non-static trajectories with correctly synced image and joint data. These validate the pipeline only; they aren't manipulation demonstrations yet, since the scene has nothing to pick up.

### 6. Scene objects for pick-and-place (CP6 — in progress)
- Three 4 cm cubes (Red/Blue/Green, distinct materials) sized against the gripper's real measured jaw opening (0.14 m) and the arm's verified reach envelope from CP4.
- Real rigid-body physics per cube — box/convex-hull collider, mass grounded in plastic density — not visual placeholders. Fixed positions for now; randomized placement deferred.
- Scene built as a separate, self-contained USD file and referenced into the robot's stage.
- Extensive gripper physics tuning to get reliable grasps: gripper max drive force grounded in the SO-101's real servo stall-torque spec, raised solver velocity iterations to fix grasp jitter and slow-creep drops, retuned friction and torsional patch radius for grip stability on the cube surfaces.
- **Still open:** a small visible rest-offset gap between gripper and cube at rest (fix identified, not yet applied), occasional jaw penetration into the cube under a heavy squeeze (suspected solver-type issue, under investigation), and the full CP6 success test — verified placing (not just grasping), and re-testing on the Blue and Green cubes specifically, since most tuning so far has been validated on the Red cube only.

---

## Tech Stack

- **Simulation:** NVIDIA Isaac Sim, OmniGraph
- **Middleware:** ROS2
- **Arm:** SO-ARM (SO101), 5-DOF + gripper
- **Kinematics:** Custom Python IK/FK (`scipy.optimize.least_squares`), cross-validated against `yourdfpy`
- **Data:** HDF5, OpenCV, ffmpeg
- **Language:** Python

---

## Repository Structure

```
Quadruped_Arm_Bot/
├── isaac_sim/          # Isaac Sim stage/OmniGraph assets
├── ros2_ws/src/        # ROS2 packages (robot description, teleop, kinematics)
├── scripts/            # Kinematics, teleop, and data collection scripts
├── data/episodes/      # Recorded HDF5 demonstration episodes
├── Quadruped_Arm_bot_project_instructions.md   # Full project roadmap and scope
└── QAB_progress.md     # Detailed engineering log, checkpoint by checkpoint
```

For the full engineering log — including debugging notes, design decisions, and open items for each checkpoint — see [`QAB_progress.md`](./QAB_progress.md).

---

## What's Next

- **Close out CP6:** fix the gripper rest-offset gap, resolve jaw penetration under heavy squeeze, verify full place-down (not just grasp) success, and re-confirm grasping on the Blue and Green cubes.
- **CP7:** Train a small imitation-learning policy (vision + joint state + instruction → action) on the collected data.
- **CP8:** Run the trained policy in sim in place of teleop and evaluate task success.

---

## Related Work

This project builds on [**Bio-Inspired Quadruped Spiderbot**](https://harsh-siva.framer.website/projects/bio-inspired-autonomous-spiderbot-(on-going)), which built the original quadruped chassis, teleop pipeline, and sim2real workflow from scratch.
