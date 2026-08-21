## CP1 — Reproduce Teleop Pipeline on the New Arm-Ready URDF
Status: COMPLETE

- (a) Repo scaffolded, Spiderbot_Without_Nav_description package builds via colcon.
      URDF regenerated from xacro (hand-maintained version had unresolved
      $(find ...) mesh paths — fixed). Verified against real xacro output:
      0 mismatches on joints/links (compare_urdf.py), MD5-identical leg
      meshes (coxa/femur/tibia) vs old robot; only base_link.stl differs.
- (b) Imported to Isaac Sim at /Spiderbot_Without_Nav (Base Type: Mobile,
      Robot Type: Quadruped). Fixed empty-mesh import (caused by unresolved
      xacro mesh paths, same root cause as above). Set joint drives
      (stiffness 1e5, damping 1e3, all 12 leg joints) — import left these
      at 0 with no warning-driven default, robot ragdolled until fixed.
      Rebuilt joint_states_control OmniGraph from scratch (old graph's
      hardcoded /World/prjct_spider_bot paths don't apply): ROS2Context,
      ROS2SubscribeJointState, IsaacArticulationController,
      IsaacComputeTransformTree, ROS2PublishTransformTree, OnPlaybackTick.
      targetPrim -> /Spiderbot_Without_Nav; transform-tree parentPrim ->
      .../Geometry/base_link; targetPrims -> 4 tibia links. Verified live:
      ros2 topic pub /joint_command moves the commanded joint in sim.
- (c) fk_leg.py/ik_leg.py/pose_controller.py/gait_generator_node.py/
      xbox_teleop_node.py ported with zero changes. Verified safe: joint
      origins/axes/limits match new URDF exactly; leg meshes MD5-identical
      (used for C3 foot-tip offsets). No hardcoded old-package references
      found in any script.
- (d)/(e) Body pose controller + Xbox teleop/gait confirmed working live
      on new robot (visual test by Harsh, full walking parity achieved).

Open items / notes for later checkpoints:
- IMU and ROS_Camera OmniGraphs from the old joint-controller USD were
  NOT ported (sensors removed on this URDF variant, correctly out of
  scope for CP1).
- isaac_sim/ output folder structure under Quadruped_Arm_bot mirrors the
  old project's spider_bot_base pattern.

---

## CP2 — Select and Mount a Small Robotic Arm
Status: COMPLETE

- Evaluated Isaac Sim's default arm library (UFactory lite6, UR series, Franka,
  Techman TM12) against real chassis data pulled directly from the URDF/STL
  (base_link ~175mm x 170mm footprint, 5.615 kg mass, sourced from
  Spiderbot_Without_Nav.urdf's <inertial> tag and base_link.stl bounding box).
  Lite6 (7.2kg total, 440mm reach) judged oversized/overweight relative to a
  5.6kg base — ruled out. Dofbot flagged as possibly deprecated as of Isaac
  Sim 4.2.0 release notes — not pursued.
- Selected SO-ARM100's SO101 variant (sim-ready URDF, from
  TheRobotStudio/SO-ARM100 repo, Simulation/SO101/so101_new_calib.urdf) over
  the original SO100 URDF, which has a documented open GitHub issue (#54:
  missing joint limits, inverted axes, no decomposed collision meshes).
  Downloaded and handed off as so101_description package (URDF + relative
  assets/ mesh folder, mirroring Spiderbot_Without_Nav_description's package
  convention).
- Standard Import dialog failed to import into the existing stage (opened a
  blank new stage instead); worked around via drag-and-drop from Content
  Browser into the viewport. Flag for later if this recurs on other assets.
- Imported with Robot Type: Manipulator, Base Type: Fixed, Colliders left at
  default (URDF already defines per-link mesh collision for base_link,
  shoulder_link, upper_arm_link, lower_arm_link, wrist_link, gripper_link,
  moving_jaw_so101_v1_link — "Collision From Visuals" not needed).
- Drive gains were 0/0 on import (same root cause as CP1's leg joints — Isaac
  Sim import doesn't reliably carry over drive gains). Set manually on all 6
  revolute joints to stiffness 1000 / damping 50 as a starting point — NOT
  derived/tuned against link mass/inertia, just enough to stop the ragdoll.
- Initial mount attempt was prim-parenting so101_new_calib under
  base_link/Geometry — this only set visual position and created NO physics
  constraint (arm's own root_joint stayed fixed to world, not to the
  quadruped). Symptom: arm appeared "mounted" visually but passed through
  robot geometry during motion, since it was actually anchored in world space
  independent of the moving chassis.
- Diagnosed via Physics Inspector: selecting /Spiderbot_Without_Nav showed a
  single merged articulation (21 joints — all 6 arm revolute joints + 2 arm
  fixed joints + 12 leg joints + root_joint, all together). Confirmed no
  physics joint spanned both robots via a stage-wide joint scan script
  (checked Body0/Body1 targets of every UsdPhysics.Joint against the arm's
  subtree path).
- Fixed by script: repointed root_joint's Body0 target from world to the
  quadruped's base_link, computing the local position/rotation offset from
  the live (already visually-correct) world transforms of both prims via
  UsdGeom.XformCache, so the existing visual placement became the actual
  physical mount offset. Re-verified via Physics Inspector click-to-select
  on the arm after Stop/Play — Harsh confirmed it now reads as connected.
  NOT independently re-verified joint-count-by-joint-count in chat after the
  fix — worth a harder re-check before CP5 data collection if anything looks
  physically off.

Open items / notes for later checkpoints:
- Arm/leg mesh pass-through during robot motion — explicitly deferred, not
  fully root-caused (working theory: was a symptom of the world-anchored
  root_joint bug above, not a separate collision-geometry problem, but this
  was NOT independently confirmed after the root_joint fix). Revisit before
  CP4 teleop or CP5 data collection — a real interpenetration bug there could
  corrupt collected demonstrations.
- Drive gains (1000/50) are a working starting point, not derived/tuned.
- The arm's top-level Xform (so101_new_calib) may still carry the earlier
  manual translate/rotate used for visual placement, on top of the new
  root_joint-derived offset — check for double-offset/compounding next time
  the arm is touched; zero out the Xform's transform if so, since the joint
  now fully defines the mount position.

---

## CP3 — End-Effector Camera
Status: COMPLETE

- Open decision resolved: real Isaac Sim **Camera prim** (not cosmetic mesh) —
  required since CP5's data pipeline depends on actual image output, not just
  visual placeholder geometry.
- Mount point decision: initially considered `moving_jaw_so101_v1_link`, ruled
  out because it's the gripper's moving jaw (has actual Mesh children) — a
  camera there would shift pose every time the gripper opens/closes, corrupting
  consistent vision data. Used `gripper_frame_link` instead — confirmed as a
  pure reference/TCP frame (only a `Scope` child called `origin`, no Mesh
  geometry under it), sibling to the moving jaw under `gripper_link`.
  Full path: /Spiderbot_Without_Nav/Geometry/base_link/so101_new_calib/
  Geometry/base_link/shoulder_link/upper_arm_link/lower_arm_link/wrist_link/
  gripper_link/gripper_frame_link
- Camera prim created directly under `gripper_frame_link` via right-click →
  Create → Camera, inheriting parent transform (stays correct relative to
  gripper regardless of arm motion).
- Orientation was wrong on creation (pointing up). Fixed by setting the
  camera as active view (Stage right-click → Set as Active Camera) and
  visually inspecting the live feed rather than guessing rotation values —
  confirmed correct by Harsh via direct viewport check. Final orientation:
  X 180.0 / Y 0.0 / Z 90.0 (translate 0,0,0, scale 1,1,1 — camera sits exactly
  at the gripper_frame_link origin).
- Read-out method decision resolved: **ROS2 topic**, not direct Isaac Sim
  Python API — chosen for consistency with CP1's existing ROS2 pipeline
  pattern (joint states, TF tree), and because CP5 will need to synchronize
  camera data with joint/base state that's also coming over ROS2 topics.
- OmniGraph built using Isaac Sim's built-in camera nodes: On Playback Tick →
  Isaac Run One Simulation Frame → Isaac Create Render Product → ROS2 Camera
  Helper + ROS2 Camera Info Helper (both gated through a shared ROS2 Context).
- Isaac Create Render Product node: `cameraPrim` set to the gripper_frame_link
  Camera path; resolution set to **224x224** (changed from the 1280x720
  default) — chosen with CP6's eventual small-policy training input in mind
  rather than leaving the viewport-native default in place.
- ROS2 Camera Helper: `topicName` = `/rgb`, `type` = rgb, `frameId` =
  sim_camera, `renderProductPath` wired from the render product node's output.
- ROS2 Camera Info Helper: `topicName` = `camera_info`, `frameId` =
  sim_camera, `renderProductPath` wired the same way.
- Verified end-to-end (not just assumed from graph wiring):
  - `Isaac Create Render Product`'s `renderProductPath` **output** confirmed
    populated after Play (`/Render/OmniverseKit/HydraTextures/Replicator...`),
    proving the render product itself is being generated before checking
    anything downstream.
  - `ros2 topic list` confirmed `/rgb` and `/camera_info` both present
    alongside the existing CP1 topics (body_pose, cmd_vel, joint_command, tf,
    etc.).
  - `ros2 topic hz /rgb` confirmed live publishing at ~21-24Hz (not just a
    registered-but-dead topic).
  - `ros2 run rqt_image_view rqt_image_view` on `/rgb` confirmed actual image
    content is correct and matches the gripper's-eye view seen earlier in the
    Isaac Sim viewport-camera preview — ruled out blank/garbage frames.
  - `ros2 topic echo /camera_info --once` confirmed width/height both 224,
    non-zero intrinsics matrix `k`, and principal point (112, 112) correctly
    centered at half of 224x224 — ruled out a zeroed-out/garbage CameraInfo
    message.

Open items / notes for later checkpoints:
- Resolution (224x224) was chosen anticipating CP6's training input, not
  derived from an actual confirmed model input spec — revisit if CP6 lands on
  a different expected resolution.
- Arm/leg mesh pass-through during motion (from CP2) — still deferred,
  unresolved. Revisit before CP4 teleop or CP5 data collection.
- Drive gains (1000/50, from CP2) — still just a starting point, not tuned.

Next: CP4 — Arm Teleop Node. Open decision to resolve at the START of that
chat (per roadmap, not to be assumed): how arm control is exposed given the
Xbox controller is already fully mapped to base velocity + body pose — e.g. a
mode-switch button handing the sticks to the arm vs. base, vs. a second input
device.

## CP4 — Arm Teleop Node
Status: COMPLETE

### Mode toggle
- Button hunt: AGR/AGL (grip paddles) produced NO /joy event at all under
  joy_node -- confirmed via joy_mapping_sniffer.py, not usable. Found
  buttons[7] (center-tab button) instead, confirmed working the same way.
- Behavior: toggle (press = switch base<->arm, not hold).
- Base freeze while in arm mode (all continuously republished, not just
  left alone): /cmd_vel forced to zero every tick; /body_pose republished
  with self.pose UNCHANGED (holds exact last pose); /gait_mode and
  /step_height likewise republished unchanged -- explicit decision that
  button 7 fully freezes the base, X and D-pad do nothing while in arm
  mode.

### Control scheme -- fully determined system
5 positioning joints (shoulder_pan, shoulder_lift, elbow_flex, wrist_flex,
wrist_roll) solving 5 explicit targets -- no leftover unconstrained DOF:
  - Left stick fwd/back    -> dz (vertical)
  - Right stick left/right -> dy (sideways)
  - Right stick fwd/back   -> dx (in/out)
  - RT held (depth) = pitch down / LT held (depth) = pitch up -> pitch
  - LB held = rotate gripper left / RB held = rotate gripper right -> wrist_roll
  - A held = gripper open / B held = gripper close -> separate jaw target
All rate-based (exp_smooth + integrate), hold-to-move / release-to-hold.
RT/LT/LB/RB/A/B are dual-purpose by mode (body pitch/roll/height in base
mode, arm pitch/rotation/gripper in arm mode) -- confirmed no runtime
conflict, branches are mutually exclusive.

### Y-button reset -- MODE-SCOPED, not shared (Harsh's explicit choice)
Base mode: Y unchanged from CP1-3 (resets self.pose only).
Arm mode: Y smoothly resets self.arm_target (dx/dy/dz/pitch/wrist_roll)
to neutral over the same RESET_DURATION/smoothstep easing as body pose,
via a COMPLETELY SEPARATE reset state machine (self.arm_reset_active
etc.) -- pressing Y in one mode never touches the other mode's pose.
Gripper JAW untouched by the arm-mode reset (Harsh's explicit
distinction: "the jaw" stays, "the gripper's rotation" (wrist_roll)
resets). Interruptible mid-ease by new stick/trigger/bumper input, same
as body pose's reset. Live-verified both directions.

### Reach limits -- GROUNDED in real IK sweeps, not guessed, TWO PASSES
First pass (initial CP4 build) used a single symmetric placeholder
(+/-0.10m on dx/dy/dz), explicitly flagged as ungrounded at the time.
Once ik_arm.py existed, replaced with 6 direction-specific bounds from a
real per-axis reachability sweep against the verified IK (dx/dy/dz swept
independently, other offsets held at 0, pitch=0):
  dx: [-0.18, +0.05]   dy: [-0.19, +0.19]   dz: [-0.25, +0.06] (interim)
This sweep revealed the old symmetric clamp was simultaneously too TIGHT
on -dz and too LOOSE on +dx (real +dx reach is only 0.06m -- the old
+/-0.10 clamp would have let a target get commanded past what the IK
could actually solve).
SECOND PASS (live feedback: "needs to reach further down" -- the interim
-0.25 still wasn't enough): the FIRST sweep only varied dz in isolation
(dx=0, pitch=0) and undersold real reach. A combined sweep (dz + dx +
pitch varied TOGETHER -- all three are simultaneously stick/trigger-
controllable during actual teleop, not hypothetical) found comfortable,
safety-margined reachability down to dz=-0.34m when paired with dx
around -0.15m and pitch around +0.75 rad (both well within their OWN
existing clamps) -- verified that specific combined target solves with
~0mm error, not at the boundary (true combined optimum found was closer
to -0.40m; -0.34m leaves real margin).
FINAL: dx: [-0.18, +0.05]   dy: [-0.19, +0.19]   dz: [-0.34, +0.06]
IMPORTANT CAVEAT for whoever tests this next: getting the deeper reach
REQUIRES combining dz with dx and pitch together (pull right stick back
+ pitch down while pushing dz down) -- pure vertical-only motion will
still hit real kinematic resistance well before -0.34m. This is a
genuine kinematic property of the arm's geometry, not a bug.
ALSO UNRESOLVED: all of the above is in the ARM's OWN base_link frame,
NOT relative to the ground. If combined-axis reach still doesn't reach
the ground physically, that points to the arm's actual MOUNTING HEIGHT
on the robot (hardware/mounting question) rather than anything a teleop
clamp can fix -- flagged, not investigated (no way to check the real
mount height from this chat).

### Topics
/arm_target (Float64MultiArray, [dx,dy,dz,pitch,wrist_roll], offsets from
a downstream-defined "home" pose) and /gripper_target (Float64, radians,
clamped to real URDF gripper limits [-0.174533, 1.74533]). pitch/
wrist_roll clamped to real single-joint wrist_flex/wrist_roll URDF limits
as a starting ceiling (not yet verified against true 5-joint-chain
achievable range for those two specifically).

### Architecture decision: teleop publishes raw targets only
Mirrors the CP6 precedent (two owners fighting over /joint_command).
Confirmed live (not assumed): /joint_command ALREADY drives arm joints
with zero new OmniGraph work -- CP2's root_joint fix merged the arm into
the SAME 21-joint articulation as the legs, so IsaacArticulationController
resolves arm joint names the same way it resolves leg joint names.

### NEW FILES: scripts/arm_kinematics/
- fk_arm.py -- forward kinematics for the 5-joint chain, built from
  so101_new_calib.urdf's origin xyz+rpy tags. CROSS-CHECKED against
  yourdfpy (independent third-party URDF parser) at 2 configurations --
  position AND full rotation matrix matched exactly.
- ik_arm.py -- numerical IK (scipy.optimize.least_squares, same tool
  ik_leg.py uses), warm-started each call from the arm's own last-solved
  angles. DESIGN DECISION: wrist_roll is DIRECTLY driven, not solved --
  matches "LB/RB spins the gripper" literally. Solves 4 joints for
  exactly 4 targets (x,y,z,pitch) -- well-determined. "Pitch" defined as
  rotation of home orientation about the ARM BASE's fixed world-Y axis
  (flagged simplification, doesn't track shoulder_pan if that rotates far
  from home). Round-trip self-tested against fk_arm.py -- sub-mm/
  sub-degree match; unreachable case correctly raises with fallback.
- arm_execution_node.py -- subscribes /arm_target + /gripper_target,
  tracks joint-state belief INTERNALLY (no /joint_states topic exists in
  this project -- confirmed via `ros2 topic list`; assumes commanded ==
  achieved, same pattern gait_generator_node.py already uses). Starts at
  all-zero joint config (Harsh confirmed OK visually). Unreachable-target
  policy: best-effort closest-approach fallback (matches
  gait_generator_node.py's per-leg policy). Publishes JointState (6
  names) to /joint_command, sharing that topic with the leg gait node.

### Verified live (all confirmed by Harsh)
Base mode unchanged from CP3. Mode-switch fires correctly. Base fully
frozen in arm mode. All 5 arm axes + gripper jaw move the real arm in
Isaac Sim. Arm-mode Y-reset eases to neutral, jaw untouched,
interruptible. Base-mode Y-reset unchanged. ik_arm.py's standalone
self-test reproduced identically on Harsh's machine. Reach improvement
(v1, -0.25) confirmed better but insufficient; final version (-0.34) not
yet independently re-confirmed live by Harsh as of this log entry --
worth a quick re-check next session, low risk given the grounded
derivation, but not yet actually watched move.

### Open items / notes for next checkpoint
- Combined-axis reach caveat above: worth confirming Harsh actually finds
  the deeper reach usable in practice (three-axis combination isn't the
  most intuitive teleop motion) -- may want an easier "reach down" macro
  later if it proves awkward.
- Ground-relative reach (mount height) genuinely unresolved -- see above.
- Stick/trigger/bumper SIGNS for all 5 arm axes: presumed correct by
  absence of complaint, never explicitly confirmed/denied one-by-one.
- ARM_TARGET_RATE (0.03 m/s) -- with dz's range now 0.34m instead of
  0.25m, reaching the new limit takes longer (~11s held). Not yet raised;
  flagged as a feel/tuning call, not a correctness issue.
- ARM_PITCH_LIMIT/RATE, ARM_GRIPPER_ROT_RATE, GRIPPER_RATE,
  PITCH_TOLERANCE_RAD, REACHABILITY_TOLERANCE_M: still untuned starting
  points.
- No /joint_states feedback topic exists -- both leg and arm nodes run
  open-loop.
- /joint_command has two publishers (gait node, arm node) -- worked in
  live testing, no dedicated simultaneous-stress test done.

---

## CP5 — Synchronized Data Collection Pipeline
Status: FUNCTIONAL (pipeline built and verified) — NOT producing meaningful
demonstrations yet, since no manipulable objects exist in the scene (see
CP6 below, inserted specifically to address this).

Open decisions resolved:
- Storage format: HDF5 (matches prior Spiderbot RL rollout-logging
  experience, avoids a rosbag->HDF5 conversion step before training).
- Sync method: camera-driven callback -- on every /rgb frame, sample the
  latest cached /joint_states message and stamp it. No message_filters
  needed since joint state has no acquisition lag.
- Instruction labels: manual per-episode entry via input() prompt at
  episode start.

Schema (as implemented): per-episode .h5 file with attrs (instruction,
episode_index, recorded_at, joint_names) and datasets images [N,H,W,3]
uint8 (H/W captured dynamically from the real camera frame), joint_positions
[N,num_joints] float32 (num_joints captured dynamically -- currently 18,
full merged articulation not arm-only), timestamps [N] float64.

OmniGraph work: built a new standalone Action Graph (/Graph/joint_states_
publish), separate from the existing working joint_states_control graph.
On Playback Tick -> Isaac Read Joint State Node + ROS2 Publish Joint State
(both directly from Tick, in parallel) -> ROS2 Context -> Publish node's
Context -> all 7 Read-node outputs wired 1:1 into matching Publish-node
inputs (current preferred pattern; Publish node's own targetPrim left
empty).

KEY DEBUGGING FINDING (relevant to future work too): Isaac Read Joint
State Node's Prim Path must be the EXACT prim carrying
UsdPhysics.ArticulationRootAPI -- for this robot that's
/Spiderbot_Without_Nav/base_link, NOT the top-level /Spiderbot_Without_Nav
Xform. The older IsaacArticulationController (used in the separate,
already-working joint_states_control graph) tolerates the Xform path via
a more forgiving resolution method (dynamic_control) -- that graph
"working" was NOT proof the Xform path was correct for tensor-based nodes.
/joint_states now publishes correctly with the base_link path.

Scripts written:
- scripts/data_collection/episode_recorder.py -- ROS2 node, subscribes
  /rgb + /joint_states, records to HDF5 on Enter-key toggle (TEMPORARY --
  not wired to a joy button yet; RECORD_BUTTON_INDEX was never confirmed
  against xbox_teleop_node.py's real mapping, so keyboard was used to
  unblock testing). Filenames: ep{NNN}_{instruction_slug}.h5,
  auto-incrementing across runs (scans existing files for the next index).
- scripts/data_collection/visualize_episode.py -- loads an episode .h5,
  outputs a contact-sheet PNG (8 sampled frames), a joint-position-over-
  time plot PNG, and a playback .mp4 (encoded via ffmpeg/libx264/yuv420p
  for player compatibility -- original OpenCV mp4v output wasn't playable
  in standard players).

Bugs hit and fixed during testing:
1. h5py not installed on target Python -- resolved via apt install
   python3-pip + python3 -m pip install h5py --break-system-packages
   (Ubuntu 24 externally-managed-environment behavior).
2. Image dataset hardcoded to 480x640 (a guess) -- actual camera is
   224x224. Fixed to capture real shape dynamically from the first frame.
3. Race condition: stop_episode() (keyboard thread) could close/null the
   HDF5 file while camera_cb (ROS executor thread) was mid-write, crashing
   with "Invalid dataset identifier." Fixed with a threading.Lock()
   guarding all state transitions and writes.
4. mp4v-encoded video wouldn't play in standard players -- switched to
   piping raw frames to ffmpeg (libx264 + yuv420p).
5. Cosmetic-only: double-shutdown traceback on Ctrl+C -- wrapped in
   try/except, no data-loss risk (episodes are closed/flushed on stop
   before this point regardless).

Test episodes recorded: ep001_turn_right.h5, ep002_turn_right.h5,
ep003_turn_left.h5 -- verified real, changing joint trajectories (not
frozen/static), correct image shape, video plays back correctly. These
are NOT usable as real manipulation demonstrations (no objects in scene
to interact with) -- kept only as pipeline-correctness verification, not
CP7 training data.

Open items / notes for next checkpoint:
- Episode trigger is keyboard-only -- confirm a safe joy-button index and
  wire it in properly.
- Base state is not yet logged at all (schema doesn't currently have a
  base_state field) -- CP5's original planning draft mentioned this as
  likely-frozen/minimal since arm mode freezes the base, but this was
  never explicitly revisited or implemented. Revisit if base_state ends
  up mattering for CP7 training.
- No volume/scale testing yet -- only 3 short manual test episodes exist.
- arm_execution_node.py was NOT modified for CP5 (the originally-planned
  /arm_joint_state_log publisher approach inside that node was abandoned
  in favor of the OmniGraph-based real /joint_states -- a legitimate
  architecture change from the original CP5 plan, noted explicitly per
  the roadmap's requirement to flag such changes rather than carry them
  forward silently).

---

## ROADMAP CHANGE (explicit confirmation given)
Inserted new CP6 -- "Add Scene Objects for Pick-and-Place" -- between CP5
and the old CP6, because CP5's recording pipeline is functional but has
nothing real to record: no manipulable objects exist in the Isaac Sim
scene yet, so CP7 (policy training, formerly CP6) cannot be meaningful
without this. Renumbered everything from the old CP6 onward by +1: old
CP6 (policy training) -> CP7, old CP7 (evaluation) -> CP8, old CP8+ ->
CP9+. This was done with Harsh's explicit confirmation, per the roadmap
file's own rule that renumbering requires explicit sign-off, not
inference. See the updated project instructions file for the new CP6's
full scope and open decisions (object shape/graspability, physics
properties, placement/randomization, visual distinctiveness).

Next: CP6 -- Add Scene Objects for Pick-and-Place. Open decisions to
resolve at the START of that chat: object shapes/sizes (graspable within
real gripper jaw range + arm's grounded reach envelope from CP4), object
physics (collider/mass/friction, not just visuals), placement method
(fixed/scripted vs. randomized), and visual distinctiveness (needed if
instructions should reference specific objects, e.g. "pick up the red
cube").
