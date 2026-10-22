# Plan: IRobots v2 — a hardware-free, reproducible LLM-manipulation research artifact

- **Status:** IN PROGRESS — Phase 0 complete (gates C01, C01c, C04 pass; C02 pending; C03 blocked on an API key)
- **Tier:** L (cross-module, new public interfaces, new persistence/schema, concurrency, safety-critical, long horizon)
- **Repo state:** branch `feature/phase0-spikes`, base commit `5b926b2`, 2026-10-22
- **Spec:** operator messages in the planning session (D1–D15 below)
- **Operator:** Devansh Gaur · **Agent:** opencode
- **Staleness rule:** if `config/robot.yaml`, `config/scene.yaml`, or `ros2_ws/src/` move before Phase 1 starts, re-verify §C and §E.

### Phase 0 outcome (2026-10-22)

The gates are recorded in [`spike/REPORT.md`](spike/REPORT.md) and the numbers in
[`docs/RESULTS.md`](docs/RESULTS.md). In short: the grasp gate passes at 100%
(250 trials, mean slip < 2 mm), the analytic and simulated kinematics agree to
0.0000 um, and the full stack fits in 1815 MB against a 2500 MB budget.

**Six design parameters changed as a direct result of the gates**, and the plan
is updated to match reality rather than the other way round:

| Parameter | Plan said | Now | Why |
|---|---|---|---|
| Workspace | disk, r=0.22 at (0, 0.17) | disk, r=0.10 at (0, 0.20) | The larger disk reached 495 mm from the base against a 397 mm arm, and wrapped *behind* the base, needing a 172 deg yaw against a 170 deg limit. The arm also cannot fold tighter than 104 mm, so the near edge has to stay clear of that. |
| Table | 0.50 x 0.40 at the origin | 0.55 x 0.50 at (0, 0.12) | The workspace did not fit on the table, so the arm had to reach *over the rim* to reach the objects. |
| Camera pose | 0.55 m up, 35 deg tilt | (0, -0.185, 0.55) -> (0, 0.20, 0) | Follows the workspace centre; the tilt and height are unchanged. |
| Elbow limit | +/-150 deg | +/-160 deg | At 150 deg the arm could not fold to within 104 mm of its base, so part of the workspace was unreachable. |
| Gripper max opening | 0.045 m (measured) | 0.070 m (design) | A jaw that opens only 5 mm wider than a 40 mm cube has no tolerance for a 3 mm positioning error: the pads clip the object on the way down. The 70 mm figure needs measurement on the real arm. |
| Joint limits in config | degrees | degrees, explicitly | They were first written in radians while the loader read degrees, which silently limited `base_yaw` to +/- 3 deg. |

---

## A. Problem, goal, success criteria

**Problem.** The existing repo cannot run. `orchestrator.py:18` imports `get_image`, which exists only at `robotic arm schematics/get_image.py`; there is no `requirements.txt` (though `README.md:187` says to install one); `test_artifacts/` and `handshake/` do not exist, so even `--offline` raises `FileNotFoundError`; `agent_harness.py:120/128` raises `UnboundLocalError` on any model that emits content without reasoning tokens; `config.py` is duplicated verbatim at `src/vision-AI/config/config.py`; the arm firmware (`V1_arm_controller_full_status.ino:166`) is a blocking `delay()` sweep with no network, no IK, no per-joint limits and no watchdog.

**Three structural defects** are deeper than the missing files, and they are what the rebuild must fix:

1. **No identity.** `sam2_processor.py:132-148` emits masks with no class label, so "the red block" cannot be bound to an object. The architecture doc's central rule (model selects by identity) was unimplementable.
2. **Wrong coordinate frame.** `sam2_processor.py:120-121` computes `cm = px / 30.0` with the origin at the image corner. `PIXELS_PER_CM=30.0` has no source, and 30 px/cm is arithmetically inconsistent with the shipped 320x240 camera (`CameraWebServer.ino:98`).
3. **Invented Z.** No depth, no class, so height was guessed — the single largest cause of grasp failure.

**Why now.** The operator has 6–8 weeks and has chosen research as the goal, so the deliverable is evidence, not a demo. Git history contains **no** IK, homography, WebSocket, URDF or simulation in any of 17 commits — the hard parts were never implemented, so a rebuild inherits zero prior work but also loses nothing. [FACT `git log --all`]

**Goal.** A reproducible artifact in which: a closed loop (perceive -> plan -> one skill -> verified motion -> re-perceive) runs end to end; four research questions are answered with numbers; every headline result reproduces on a machine with **no arm, no camera, no GPU**; and the path to the real ESP32 arm is specified and protocol-tested without the arm.

**Success criteria (observable, numeric).**

| ID | Criterion | How measured |
|---|---|---|
| SC1 | Scripted reference policy reaches **>=80%** on L1 | `eval` metrics, 30 seeded scenes |
| SC2 | Detection **>=95%** and p95 XY error **<=8 mm** | 200-frame frozen set vs ground truth |
| SC3 | 100 scripted grasp attempts succeed **>=80%** at mu=0.5, <=2 s/episode | P0 spike benchmark |
| SC4 | 150+ episodes complete unattended with **zero hangs** | eval runner, per-trial subprocess + wall-clock deadline |
| SC5 | A fresh clone reproduces headline numbers within **+/-1 trial** | `reproduce.sh` in CI on the pinned image |
| SC6 | `docs/RESULTS.md` answers RQ0-RQ3 with tables + one figure each | written report |
| SC7 | CI runs headless in **<10 min** with no network and no GPU | GitHub Actions |

**Non-goals (explicit).** MoveIt 2; multi-arm; a claim of generalization to arbitrary arms; any real-arm grasp claim; VLA training; web UI; voice; ROS 2 nodes for the *algorithms* (only the skin); a device-side IK solver; real-hardware bring-up as a critical-path item.

**Constraints.** CPU only (no GPU); 5.9 GiB RAM available, agent stack must keep running; 29 GB disk; Ubuntu 24.04 / Python 3.12; ROS 2 Jazzy + Gazebo Harmonic (Harmonic is the correct pairing for Jazzy; Jetty pairs with Rolling/Lyrical); OpenRouter key; 1 developer, 6-8 weeks.

**Reversibility.** Two-way. New code lands in new paths; legacy is frozen under `legacy/` and deleted only at cutover; each phase is one commit on `main` (D9) and independently revertable.

---

## B. Glossary

**Action encoding** — the vocabulary the LLM may use. *Skills* (discrete, parameterized) vs *raw coordinates* (continuous). RQ1 compares them.

**Scene** — one frame plus the detection list derived from it, with a timestamp and a frame id.

**Identity** — a persistent, tracked id for a physical object across frames. Distinct from a per-frame detection index.

**Skill** — one atomic, parameterized robot action the host can execute and verify (`move_above`, `grasp`, `lift`, `release`, `retreat`, `home`).

**Verification** — the typed observation of what actually happened after a skill, computed by comparing before/after scenes.

**Privilege** — a metric computed from simulator ground truth (object pose) rather than from perception. Always labelled.

**Height tiers** — the three ways the system gets an object's Z: probe (measure), class prior (assume), vanishing point (estimate).

**Joint-space wire protocol** — the firmware receives angles, not Cartesian targets.

**Episode** — one task attempt: seed -> scene -> loop -> outcome -> metrics.

---

## C. Architecture

### C.1 The world as it is today

```
operator -> orchestrator.py (rich TUI)
             |- get_image.py        HTTP GET /capture from ESP32-CAM  [BROKEN: not at repo root]
             |- sam2_processor.py   Ultralytics SAM -> masks, no labels
             |                      cm = px / 30.0, origin = image corner   [WRONG FRAME]
             +- agent_harness.py    LM Studio + tool calls
                                    validate_and_execute_tool(): bounds check, then
                                    returns "SUCCESS: Arm moved" without sending  [FAKE]
               +- arm               never reached - no network code exists in either sketch
```

Only the camera firmware and the mechanical CAD are real assets. `robotic arm schematics/5-dof-robotic-arm-5.snapshot.3.zip` contains 10 STLs whose bounding boxes were measured: Arm 1 = 194 mm (servo flanges 140 mm apart), Arm 2 = 130 mm, Arm 3 = 34 mm, Circular Base 124 mm, Gripper Link 39 mm, Grip 8.5 mm. [FACT STL bbox analysis] The SolidWorks parts and `.SLDASM` are opaque binaries with zero readable strings, so **joint axes, offsets and limits are not recoverable** — they must be *specified*, and that specification becomes a design assumption on the record. [FACT `strings Assembly.SLDASM` -> 7,651 gibberish lines]

### C.2 Target architecture

```
                         +------------- pure-Python core (no ROS imports) -------------+
  task --> planner --> worker --> skill executor --> safety supervisor --> arm backend
           (LLM)      (LLM)        (deterministic)      (deterministic)        |
              ^            |              |                                  |
              |            |              v                                  v
              |            |        +---------+                        MuJoCo sim node
              |            |        |perception|  <- renders 640x480           (ROS 2 node)
              |            |        |detector |                            publishes
              |            |        |tracker  |                            sensor_msgs
              |            |        |height   |                            /Image + JointState
              |            |        |grasp    |                                 |
              |            |        +---------+                                 |
              |            |                                                   |
              +- verification +<- outcome predicates <- next scene -------------+
                                                    |
                                          ESP32 emulator / real ESP32 (TCP, joint space)
```

**Invariants the code must enforce (not merely document):**

- **I1** The LLM never emits a Cartesian coordinate. Coordinates are derived by deterministic code from a scene, at execution time.
- **I2** Every commanded pose passes the safety supervisor before execution; a rejection is a first-class outcome the planner sees.
- **I3** One skill per observation. The worker never chains skills in one turn.
- **I4** Every episode terminates: per-skill deadline, per-episode deadline, per-trial hard kill.
- **I5** No metric mixes privileged and perceived quantities.
- **I6** The physics/sim is stepped deterministically; the LLM's wall-clock latency never changes how much physics time elapses.

**Conventions.** `ruff` + `mypy --strict` on `irobots_core`; `pytest`; every config file is a typed YAML with a schema; no module-level mutable state; every random draw goes through a seeded `numpy.random.Generator` threaded through the call chain; no hardcoded IP/port/path in source (all from config/env); ROS 2 packages are `ament_python` except `irobots_msgs` (`ament_cmake`).

---

## D. Design decisions

### D.1 The instrument is not the subject — simulator choice (D2)

The research questions are about the **LLM / perception / control architecture**. The simulator is a measuring instrument.

| | MuJoCo | gz-harmonic |
|---|---|---|
| Contact model | documented `condim`/friction, mature for small-object grasping | DART/PGS, 100 iterations, no error redistribution; no force control on the jaws |
| Determinism | deterministic by construction at fixed `dt` | ROS 2 clock coupling breaks it unless `use_sim_time` **and** pause-on-think are both right |
| Episode cost | ~1-2 s | ~10 s+ |
| Install | `pip install mujoco`, ~200 MB | 4-8 GB image, 20-60 min pulls, lavapipe-in-Docker is a known sink |
| RAM | ~0.2 GB | 0.8-1.3 GB (server + ogre2/lavapipe) |

**Decision:** MuJoCo is the primary instrument; ROS 2 Jazzy is the integration skin with the simulator *running as a node* that publishes `sensor_msgs/JointState` and `sensor_msgs/Image` (no `ros_gz_bridge` needed); gz-harmonic is an **optional** secondary backend for a cross-simulator transfer check. This supersedes the earlier "gz + ROS 2 from day one" choice, which was right about the *interface* and wrong about the *instrument*. The thin-wrapper decision (D3) is what makes this cheap: if ROS 2 later proves unnecessary, the core and the sim node survive untouched.

### D.2 Action encoding: skills, and a *fair* control condition (D16)

The LLM emits **one skill per turn** from a fixed library; the host derives every coordinate.

But the obvious ablation is **rigged**: the raw-coordinate arm would also lack a good Z policy, so the comparison would measure "who gets to pick Z", not "discrete vs continuous action space". So the control condition gets **the same Z policy, the same perception, the same safety supervisor, the same turn budget, the same retry budget, and the same paired scenes**; the *only* difference is the action vocabulary. The raw-coordinate condition is the legacy encoding (`move_arm(x,y,z)` + `set_claw`).

**Honest framing of the LLM's contribution.** Per turn the decision space is ~7 skills x ~4 objects = ~28 options, and for L1 the correct sequence is largely determined by the scene: the LLM contributes roughly 2-4 bits per episode. So the headline question is **RQ0: how much does the LLM add over a hand-written skill script, per unit of latency and cost, and how often does it loop or mis-select an identity?** A null result is a result. The artifact should not claim "an LLM controls a robot"; it should claim "we measured what the LLM adds, and it is/isn't worth its cost".

### D.3 Monocular Z: probe first, and fix the D/2 bias (D6)

The operator chose monocular Z (no depth camera) — good, it keeps the real hardware path open. But the Z hierarchy must be inverted from the first draft:

- **Tier P (probe) — primary.** Descend the gripper to a known Z, detect the image row where the jaw's leading edge begins to occlude the object's top edge, and read the object's height from that. This is a *measurement* that does not require knowing the footprint depth, and it reuses the descent the grasp needs anyway.
- **Tier C (class prior) — fallback.** Per-class height table in `config/perception.yaml`, used when probing is refused (out of budget) or ambiguous.
- **Tier V (vanishing point) — deprioritized.** The silhouette equation `h_px = f(H*cos(theta) + D*sin(theta))/Z` has two unknowns; inverting a 4 cm cube at theta=35 deg, f=554, Z=0.66 m gives 6.9 cm (**+72%**). The cube's own edges lean ~4 px over its height, so the VP cannot be recovered from the image — it would have to come from calibrated extrinsics, i.e. a calibration we would then trust absolutely. Kept as a measured-but-disabled path so the ablation can report *why* it was rejected.

**The D/2 bias is explicit, not hand-waved.** The bottom edge of an axis-aligned box resting on a table is the *front*-bottom edge, so back-projecting it lands **D/2 toward the camera** — 2.0 cm for a 4 cm cube. The system therefore estimates D (from the mask's visible top-face quad, or from the class prior) and subtracts D/2. The claim is "the contact point is *on the table plane*, offset toward the camera by D/2", not "exactly correct". P0 spike S2 measures this bias and validates the correction before anything else is built.

**Tilt, not top-down.** A strictly overhead camera cannot observe height at all (a vertical object's height projects to nearly no pixels), and the legacy `docs/hardware.md` specifies straight-down. Since no mount exists (D12), the plan specifies the geometry: **camera 0.55 m above the table, tilted 35 deg from vertical, looking at the workspace centre, HFOV 60 deg**, with the FOV stored as a calibrated parameter rather than assumed. At 640x480 a 4 cm cube at the table centre is ~34x47 px — above the size at which open-vocabulary detectors retain AP; at 320x240 it is 17x24 px, which is **not** usable. So 640x480 is the floor, and the ESP32-CAM's native 320x240 is recorded as a known degradation case, measured on the real image set (D14), not assumed.

### D.4 Identity must be tracked, not indexed

A per-frame detection index is broken: "the red block" is ID 2 this frame and ID 3 next. The perception module therefore maintains **persistent identities** via an IoU + centroid tracker with appearance confirmation, and the skill API takes a *tracked id* that survives frames. When the tracker loses an id, the skill is refused with `unknown_target` rather than silently re-targeted. This is a hard invariant: a mis-selected id is the most dangerous failure mode in the system.

### D.5 The workspace is a disk, not a square (D13)

Measured reach: L1 194 + L2 130 + L3 34 = 358 mm from the shoulder, minus the 39 mm gripper link and 8.5 mm grip => **~310 mm usable radius** for an object centre. The legacy 35x35 cm square anchored at a base corner has its far corner 495 mm away — **physically unreachable**. The workspace is therefore specified as a **disk of radius 0.22 m centred 0.17 m in front of the base**, on a 0.50x0.40 m table, with the base at the origin. All of it lives in `config/scene.yaml` with a `provenance` field per value (`measured` | `assumed` | `design`).

### D.6 The robot is a *specified* artifact, and that is stated (D13)

Joint axes, offsets, limits and the end-effector convention are unrecoverable from the CAD, so the plan *defines* them:

| Joint | Axis | Limits (design) | Provenance |
|---|---|---|---|
| `base_yaw` | Z | +/-170 deg | design |
| `shoulder_pitch` | Y (after base yaw) | -20 deg .. +95 deg | design |
| `elbow_pitch` | Y | -150 deg .. +150 deg | design |
| `wrist_yaw` | Z (vertical post, 34 mm ~ "Arm 3") | +/-170 deg | design (matches "Arm 3" 34 mm) |
| `wrist_pitch` | Y | +/-60 deg | design |
| `gripper` | prismatic pair (2 fingers, no `MimicJoint`) | 0.008 .. 0.045 m | from "Grip" 8.5 mm, "Gripper Link" 39 mm |

A 5-joint arm with a **yaw-then-pitch wrist** is what makes the jaw's closing direction independently controllable (a yaw-pitch-roll wrist cannot set jaw yaw independently of base yaw, which would make corner-grasps unavoidable). `MimicJoint` is explicitly avoided — it is a known integration trap; the two fingers are driven symmetrically from one controller.

**Every grasp claim in the artifact is a claim about this specified robot.** The report carries an explicit *"not verified without the physical arm"* list: the arm can grasp; the e-stop is safe; the limp-vs-hold change; any real-time or throughput claim; any torque or grasp-force statement; servo deadband, backlash and no-load speed.

### D.7 Determinism: promise less, precisely (D21)

What is **not** reproducible and must not be claimed: the LLM (OpenRouter is nondeterministic and the model may be silently updated); pixels across machines/drivers; the *number* of steps, since perception-driven control flow changes it.

What **is** reproducible and will be claimed, with a verifiable artifact:

- **Physics**: identical given the same binary, machine and step sequence.
- **Policy**: given a logged transcript, the action sequence is identical.
- **Verification**: an **action-log hash** (SHA-256 over the ordered skill calls + arguments + verdicts) is recorded per episode; `--replay` re-executes from a transcript and must reproduce the hash.
- **Cross-machine**: statistical only (confidence-interval overlap), explicitly not bit-identical.

Two engineering rules make the above true, and both were missed in the first draft:

1. **Every node uses `use_sim_time:=true`**, and the sim is **paused while the LLM thinks**. Otherwise `joint_trajectory_controller` interpolates on wall-clock time: a 3 s trajectory completes within ~5 ms of sim time, the arm teleports the whole path, and the cube is swept off the table. The "LLM latency" would silently be "trajectory timing error".
2. **Perception reads a frozen frame** — the sim is paused during capture, so two trials see identical pixels.

### D.8 Grading is privileged, and labelled (D17)

"Did it land in the bin?" needs the object's world pose — simulator ground truth. Every metric is tagged `privileged` or `perceived`, and the report shows both, because a perception-based grader ("the object is no longer visible at its old location and the gripper is above the target") is a *different, weaker* claim.

### D.9 The environment: container last, not first (D15)

The pinned Docker image is the **canonical, reproducible environment** — that is what makes SC5 checkable. But the build order is inverted from the first draft: **build the core, the sim and a scripted policy on the host in a venv; containerize last.** The first Docker build costs 4-8 GB and 20-60 min per dependency change, and it is paid *before* anything is known to work. The image is built in Phase 8, and CI then uses it.

RAM budget (measured in P0/S4, not assumed): MuJoCo ~0.2 GB; torch import ~0.25-0.35 GB; YOLO-World weights 30-130 MB (prefer ONNX export, ~150 MB total); activations 300-600 MB at 640^2; agent node ~0.3 GB => **~1.2-1.8 GB**, inside the 5.9 GiB available with the agent stack left running. Two mitigations regardless: the eval runs in a cgroup with `memory.max=3G` so it OOMs *itself* rather than letting the host OOM-killer pick the agent stack's qdrant/postgres; and perception runs in its own process so a leak cannot take the sim down.

### D.10 The one piece of hardware you have gets used (D14)

The ESP32-CAM firmware works and needs no assembly. P2 captures **~300 real frames** of a printed ChArUco board plus the task objects, with the camera in the *specified* pose. That validates the calibration path and the detector on real images — with exposure drift, JPEG artifacts and sensor noise that renders do not have. It converts "all perception numbers are from renders" (optimistic) into a partly hardware-backed claim, and it is the only honest sim-to-real datapoint obtainable without an arm.

### D.11 Firmware, on the side (D8)

The ESP32 rewrite is specified and protocol-tested, but **nothing on the critical path depends on it**. The wire protocol (TCP, newline-delimited JSON, joint-space, sequence ids, per-joint hard limits, watchdog that holds the last pose, e-stop that **holds** rather than goes limp — the legacy firmware drops PWM to zero at `V1_arm_controller_full_status.ino:313-323`) is implemented twice: once as the real firmware, once as a **host-side Python emulator** that speaks the identical protocol. Both must pass the same conformance suite, so the firmware is verifiable with no hardware and no servo inertia.

### D.12 What is deliberately not done

No MoveIt 2 (for 5 DOF, analytic IK is ~60 lines and fully testable). No device-side IK (it would duplicate kinematic constants onto the MCU and make them untestable; the firmware executes joint trajectories and owns the limits — defense in depth, with the host as the single source of truth). No WebSocket (NDJSON over plain TCP is ~50 lines of Arduino and debuggable with `nc`). No STL meshes in the sim (primitive geometry keeps software rendering cheap; the STLs stay in the repo as the provenance for the link lengths). No `GripperAction` mimic joint. No open-vocabulary *headline* RQ (it is kept as a *measurement* — held-out class detection rate — not a research claim).

---

## E. Change list

Ordered. Each row is independently reviewable and revertable by `git revert` of its phase commit. Trace tag convention: `PLAN:irobots-v2:C##`.

### Phase 0 — Feasibility spikes (2 days, host venv, no ROS 2, no Docker) — **GATES EVERYTHING**

| ID | WHAT (file::symbol) | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C01 | `spike/grasp_bench.py` | MuJoCo 5-DOF arm + 4 cm cube + 2-finger jaw; 50 seeded random cube poses; scripted policy on **ground-truth** poses; record grasp rate, slip/ejection rate, mu sweep, settle time, wall-clock/episode | Ground-truth-pose scripted policy isolates *physics* from *perception*, so a failure tells you which to fix | SC3 gate: >=80% grasp @ mu=0.5, <=2 s/ep | med |
| C02 | `spike/perception_bench.py` | Render 640x480 from the specified camera pose for the same 50 poses; YOLO-World at `imgsz=640`; measure detection rate and front-bottom-edge homography XY error vs ground truth; then apply the D/2 correction and re-measure | Rendering the *same* poses the physics spike used makes the two gates comparable | SC2 gate: p95 XY <=8 mm after correction, detection >=95% | med |
| C03 | `spike/llm_probe.py` | 20 prompts x 2 OpenRouter candidates; measure tool-call schema validity, vision handling, median latency, cost/call | A model that fails the schema at 80% is not usable in a loop | SC: >=95% schema-valid, <=5 s median | low |
| C04 | `spike/mem_bench.py` | Measure steady-state RSS of mujoco + torch + YOLO-World together; assert <2.5 GB | Measuring beats assuming, and it is the binding constraint on this box | RAM <2.5 GB | low |
| C05 | `spike/REPORT.md` | One page: four gate outcomes + the chosen camera geometry + the chosen model | The single most likely reason this project dies is discovering a gate failure in week 6 | — | low |

**Gate.** If C01 fails -> change the object (bigger cube), the jaw geometry, or the physics before writing anything else. If C02 fails -> change camera geometry before building the perception module. If C03 fails -> change model. **Do not proceed to Phase 1 until C01 and C02 pass.**

### Phase 1 — Pure-Python core (no ROS 2)

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C06 | `irobots_core/robot_model.py` | Load `config/robot.yaml`; typed `RobotModel` with per-field `provenance`; assert the FK reach is consistent with the scene disk | Provenance tags make the "specified robot" claim auditable instead of implicit | T01 link-lengths-from-STL assert; T02 provenance required | low |
| C07 | `irobots_core/kinematics.py` | Analytic IK: yaw -> planar 2R (law of cosines) -> wrist yaw/pitch; branches for elbow-up/down; returns `unreachable` + closest feasible point; FK as the inverse | 5 DOF closed-form is ~60 lines and exactly testable; iterative solvers add failure modes | T03 FK(IK(p))==p <1e-6; T04 unreachable -> clamped, no NaN; T05 singularity r->0 | med |
| C08 | `irobots_core/trajectory.py` | Trapezoidal joint-space profiles, per-joint speed/accel limits, deterministic duration from the profile (never wall-clock) | The legacy firmware snapped angles and used `delay()`; snap = swept objects | T06 monotonic, respects limits, duration is a pure function of inputs | low |
| C09 | `irobots_core/safety.py` | Workspace bounds, joint limits, speed limits, self-collision boxes, table-plane check; `check(pose) -> Verdict`; **never** a no-op path | The supervisor is the only thing between a confused LLM and a broken arm; it must be a pure function so it is exhaustively testable | T07 every out-of-bounds pose rejected; T08 table-plane violation rejected; T09 no NaN passes | med |
| C10 | `irobots_core/skills.py` | Skill dataclasses + `SkillSpec` registry; each skill declares its params, its preconditions, its verification predicate | A registry makes the action space explicit and gives the worker a schema it cannot exceed | T10 every skill has a precondition and a predicate | low |
| C11 | `irobots_core/config.py` | Typed settings from YAML + env; no defaults that hide a missing key | The legacy config had hardcoded Tailscale IPs and duplicated itself in two files | T11 unknown key fails loudly; T12 no literal IP/port in source (grep) | low |

### Phase 2 — Perception (no ROS 2) + the real-image set

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C12 | `irobots_core/perception/detector.py` | Adapter over Ultralytics YOLO-World (`yolov8s-world.pt` / `-v2`), `set_classes(vocab)`, ONNX export preferred for RAM; returns label+box+conf+optional mask | YOLO-World gives the *label* the legacy pipeline lacked; SAM alone cannot | T13 known object detected; T14 unknown class -> no detection, not a hallucination | low |
| C13 | `irobots_core/perception/homography.py` | Two calibrations behind one interface: `AnalyticCalibration` (exact, from the scene/camera model) and `MarkerCalibration` (4+ point correspondences, ChArUco). Both -> table-plane XY in the arm base frame. Report reprojection RMS; refuse above a threshold | The legacy `px/30` was neither; the marker path is what real hardware will use | T15 analytic recovers known XY exactly; T16 marker path error vs analytic on synthetic frames; T17 refuses on RMS > threshold | med |
| C14 | `irobots_core/perception/tracker.py` | IoU + centroid tracker with appearance confirmation; persistent ids across frames; `lost` state | A per-frame index mis-selects the object — the most dangerous failure mode | T18 id survives a 3-frame reorder; T19 id lost -> skills refuse | med |
| C15 | `irobots_core/perception/height.py` | Tier P probe (jaw-occlusion row -> height), Tier C class prior, Tier V vanishing point (measured, disabled). Returns `HeightEstimate(value, source, confidence)` | Probe first: it is a measurement; the VP is confounded by the footprint depth | T20 probe recovers H within 5 mm on synthetic cube; T21 prior path returns its table value; T22 VP path reports its bias, is disabled by default | med |
| C16 | `irobots_core/perception/grasp.py` | Grasp candidates from (XY, H, mask principal axis): contact point, grasp height, jaw yaw, required opening vs the object's width at that height | Deriving the jaw yaw from the mask's principal axis is what makes the grasp non-corner | T23 candidate's jaw opening >= object width; T24 candidate XY equals the corrected contact point | med |
| C17 | `irobots_core/perception/outcomes.py` | Typed outcome predicates: `object_lifted`, `object_moved_with_gripper`, `object_knocked_away`, `no_target`, `gripper_at_pose`, `table_hit` | The verification edge is the whole point of the system; a typed vocabulary makes it testable and makes "lifted vs knocked away" unambiguous (table rim, C18) | T25 each predicate on synthetic before/after scenes | med |
| C18 | `config/scene.yaml` | Table **with a 20 mm rim** (so "knocked away" is distinguishable from "lifted"), base at origin, workspace disk, camera pose/FOV, active rim | Without a rim, "no longer on the table" is ambiguous between lifted and knocked | T26 object resting outside the rim -> `object_knocked_away` | low |
| C19 | `tools/capture_real_frames.py` + `docs/hardware/CAMERA_CAPTURE.md` | Flash the existing ESP32-CAM sketch, mount at the specified pose, capture ~300 frames of a printed ChArUco board + task objects; store with ground-truth marker positions | The only physical hardware available needs no assembly; renders have no exposure drift or JPEG artifacts | T27 300 frames captured, >=95% ChArUco detected | med |
| C20 | `irobots_core/perception/real_eval.py` | Run the detector + calibrations on the real frames; report detection rate, XY error, and the gap vs the rendered set | Converts a sim-only perception claim into a partly hardware-backed one | SC2 on real frames; report both numbers | med |

### Phase 3 — Simulation node + ROS 2 skin

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C21 | `irobots_core/sim/mujoco_sim.py` | MuJoCo model built from `config/robot.yaml`; `step(dt)`, `render(camera)`, `set_joint_targets`; **fixed-step only** | Fixed-step is the precondition for every reproducibility claim | T28 same seed + same actions -> identical final state hash | med |
| C22 | `irobots_core/sim/scene_builder.py` | Seeded scene sampling: object poses, distractors, bin positions; **the same 30 scene seeds reused across every condition** | Unpaired scenes make the ablation underpowered | T29 the same seed yields the same scene in two processes | low |
| C23 | `irobots_nodes/sim_node.py` | Publishes `sensor_msgs/JointState` + `sensor_msgs/Image`; owns the pause/step discipline; `use_sim_time:=true` | The sim as a node keeps ROS 2 as a thin skin and removes the need for a bridge | T30 image timestamp == sim time; T31 paused sim -> identical frames across two captures | med |
| C24 | `irobots_msgs/msg/*.msg` | `Detection`, `Scene`, `SkillCall`, `SkillResult`, `Plan`, `ArmState`, `TaskResult` | Typed messages make the contract explicit and greppable | T32 msg round-trip test | low |
| C25 | `irobots_nodes/control_node.py` | Skill -> trajectory -> `joint_trajectory_controller` with `use_sim_time`; per-skill deadline; e-stop service | The deadline must be enforced by the driver, not the controller, or an unreachable waypoint hangs forever | T33 unreachable target -> deadline fires, episode continues | med |
| C26 | `irobots_nodes/safety_node.py` | Wraps `safety.py`; publishes rejections as outcomes; cannot be bypassed | The supervisor must sit between the worker and the arm, in the graph | T34 a rejected step never reaches the controller | med |
| C27 | `irobots_nodes/recorder_node.py` | rosbag2 for images/joints + JSONL for structured events; bounded image rate; per-episode index | rosbag2 is the standard; JSONL is what the metrics tooling actually reads | T35 a recorded episode replays from JSONL alone | low |
| C28 | `irobots_bringup/launch/*` | `eval.launch` (headless, fixed-step), `demo.launch` (realtime, GUI off), `perception_bench.launch` | One source of truth for how the system is started, so the eval is exactly what the demo runs | T36 both launch files boot and publish | low |

### Phase 4 — Agent layer (no ROS 2 needed for tests)

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C29 | `irobots_core/llm/client.py` | OpenAI-compatible client; profiles `openrouter` (default), `lmstudio`, `scripted`; every request/response logged with a hash | One client, three backends; the legacy code hardcoded a Tailscale IP and one model id | T37 all three profiles answer the same prompt shape | low |
| C30 | `irobots_core/llm/schema.py` | Strict JSON-schema validation of the tool call; one repair retry; then fall back to the scripted policy and record it | The legacy `agent_harness.py:120` crashed on a model without reasoning tokens; and `tool_call_chunks[0]` assumed index 0 | T38 malformed/partial/index-1 tool calls handled; T39 fallback engaged and logged | med |
| C31 | `irobots_core/agent/planner.py` | Task -> ordered skill plan; the plan is *validated against the current scene* before approval (every target id exists) | A plan referring to a non-existent object is the classic infinite-loop generator | T40 plan referencing an absent object -> rejected at approval | med |
| C32 | `irobots_core/agent/worker.py` | (scene, plan, memory) -> exactly one skill call; persistent tracked ids; repeat detection (same skill+target twice -> escalate) | The legacy turn cap of 10 (`orchestrator.py:156`) silently truncated *successful* long episodes | T41 one call per turn; T42 repeat detected; T43 unknown id refused | med |
| C33 | `irobots_core/agent/memory.py` | Episode memory: what was tried, what failed, the current hypothesis | "Learn from mistakes" was in the original notes and never built | T44 memory survives across turns, not across episodes | low |
| C34 | `irobots_core/agent/policy.py` | The **scripted reference policy** — a hand-written skill script, no LLM | It is both the RQ0 control arm and the ceiling every LLM condition is measured against; without it, "the LLM works" is uninterpretable | T45 scripted policy reaches >=80% on L1 (SC1) | med |

### Phase 5 — Closed loop + operator interface

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C36 | `irobots_core/loop.py` | observe -> plan/worker -> skill -> safety -> execute -> verify -> memory; **one observation per skill**; per-skill and per-episode deadlines | The feedback edge is the entire point; the legacy loop had none | T46 a full L1 episode completes; T47 the loop never issues two skills per observation | med |
| C37 | `irobots_nodes/tui_node.py` | `rich` TUI: annotated view, tracked-object table, plan tree, current skill + progress, arm state, safety banner; modes `plan` (approve) / `run` / `step` / `replay`; keys `p s e r q` | The legacy TUI was the best part of the repo; keep the ergonomics, rebuild the internals | T48 approve-then-autonomous flow; T49 e-stop key works | med |
| C38 | `irobots_core/replay.py` | `--replay <episode.jsonl>` re-executes the action sequence and must reproduce the action-log hash | This is the verifiable form of the reproducibility claim | T50 replay reproduces the hash; T51 a tampered transcript is detected | med |
| C39 | `irobots_core/estop.py` | E-stop as a first-class interrupt: cancel the trajectory, halt the arm, mark the episode aborted | The legacy e-stop is polled only between `delay(1000)` blocks, so it can take ~1 s to act | T52 e-stop aborts within one control cycle | med |

### Phase 6 — Evaluation + experiments

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C40 | `irobots_eval/runner.py` | Each trial in a **subprocess with a hard kill** and a wall-clock deadline; per-trial checkpointing; scenes from the shared seed set | One hung trial must not cost the other 29; a hang must not lose the run | SC4: 150+ episodes, zero hangs | med |
| C41 | `irobots_eval/metrics.py` | task_success, grasp_success, steps, wall_time, llm_calls, invalid_tool_calls, retries, safety_rejections, violations; every metric tagged privileged/perceived | Grading via simulator ground truth must be labelled, not blended | T53 success predicate unit-tested on synthetic scenes (in bin / near bin / held in air) | med |
| C42 | `irobots_eval/stats.py` | Paired design; McNemar on discordant pairs; bootstrap CIs; a power note stating 30 pairs resolve only ~25-30 pp | 30 unpaired trials cannot detect a 20 pp difference; claiming otherwise is a fabricated result | T54 the stats functions are unit-tested on synthetic contingency tables | med |
| C43 | `irobots_eval/exp_rq0.py` | RQ0: LLM vs the scripted policy, same scenes; measure task success, LLM contribution, latency, cost, loop/mis-select rate | The honest headline question given the LLM's ~2-4 bit/episode contribution | RQ0 table | med |
| C44 | `irobots_eval/exp_rq1.py` | RQ1: skills vs raw-coordinate encoding, **matched Z policy**, paired scenes, McNemar | Without the matched Z policy the result measures Z selection, not the encoding | RQ1 table | med |
| C45 | `irobots_eval/exp_rq2.py` | RQ2: probe vs class prior vs VP; **primary metric** abs(H_hat - H) on a frozen 200-frame set; secondary grasp success with step-count padding; plus "measure-only" vs "measure-and-act" | Tiers 1/2 are priors and tier 3 is a measurement that also costs steps; padding makes them comparable | RQ2 table | med |
| C46 | `irobots_eval/exp_rq3.py` | RQ3: failure taxonomy — wrong object, wrong target, grasp miss, collision, invalid tool call, planner loop, timeout, knocked away | Naming the failure modes is more useful than a single success number | RQ3 table | low |
| C47 | `docs/RESULTS.md` | RQ0-RQ3 answered with tables, one figure each, the "not verified without hardware" list, the AGPL-3.0 disclosure for Ultralytics | The artifact is the report; the code is the instrument | SC6 | low |

### Phase 7 — Firmware (parallel, non-blocking)

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C48 | `firmware/esp32_arm/` | Non-blocking `millis()` scheduler; TCP server, NDJSON; joint-space commands with sequence ids; per-joint hard limits; watchdog holds the last pose; e-stop **holds** rather than drops PWM | The legacy sketch is a blocking `delay()` sweep with no network at all | T55 compile + host-side unit tests of the pure logic | med |
| C49 | `tools/gen_firmware_config.py` | `config/robot.yaml` -> `firmware/esp32_arm/generated/robot_config.h` | One source of truth for the channel map and limits; a conformance test asserts the header matches | T56 generated header == config (checked in CI) | low |
| C50 | `irobots_core/arm/emulator.py` | Host-side Python implementation of the firmware state machine and protocol | Makes the firmware testable with no hardware and no servo inertia | T57 emulator passes the same conformance suite as the firmware | med |
| C51 | `irobots_core/arm/protocol.py` | Shared protocol definition + a conformance suite (scenarios: valid move, out-of-limit, sequence gap, watchdog timeout, e-stop, malformed line) | One suite, two implementations — the only way the firmware is verifiable now | T58 both implementations pass the suite | med |
| C52 | `irobots_nodes/arm_bridge_node.py` | ROS 2 <-> TCP bridge; the same node drives the emulator or the real ESP32 | The backend swap is the whole point of the emulator | T59 bridge<->emulator round-trip | med |

### Phase 8 — Packaging & reproducibility

| ID | WHAT | HOW | WHY not the alternative | Test | Risk |
|---|---|---|---|---|---|
| C60 | `Dockerfile` + `docker-compose.yml` | Pinned base, pinned deps, `memory.max=3G`, `OMP_NUM_THREADS=4`; `--network=host` for DDS | The image is what makes SC5 checkable and the artifact portable to a laptop | T60 image builds from a clean cache | low |
| C61 | `reproduce.sh` | One command: build image -> run unit tests -> run the P0 gates -> run the headline eval -> write the results table | "Reproducible" must be one command, not a wiki page | SC5 | low |
| C62 | `tools/weight_hashes.py` | SHA-256 of every model weight, recorded in every results file | A results file without the exact weights is not a result | T63 hash recorded | low |
| C63 | `.github/workflows/ci.yml` | ruff, mypy, pytest, ament lint, a headless physics smoke test; **no rendering in CI** | Rendering in CI on a software rasteriser is slow and flaky; perception tests run nightly | T64 CI <10 min, no GPU, no network | low |
| C64 | `README.md` rewrite + `legacy/` freeze | New README (what it is, how to run, results pointer, honest status table); the 4 legacy Python files + 2 sketches moved to `legacy/` with a note on why; CAD and camera firmware retained | Preserves the honest status-table style of the old README, which was the repo's best documentation asset | T65 legacy paths referenced nowhere | low |

---

## F. Files affected

| Path | Change | IDs | Why | Depended on by |
|---|---|---|---|---|
| `PLAN.md` | new | — | the plan | you |
| `ros2_ws/src/irobots_core/` | new (pure Python, no ROS imports) | C06-C34, C36-C39, C50-C51 | all algorithms, unit-testable without a graph | every node, the eval |
| `ros2_ws/src/irobots_nodes/` | new (thin rclpy) | C23, C25, C26, C27, C37, C52 | ROS 2 skin | launch files, TUI |
| `ros2_ws/src/irobots_msgs/` | new (ament_cmake) | C24 | typed contract | all nodes |
| `ros2_ws/src/irobots_eval/` | new | C40-C46 | the experiment harness | `docs/RESULTS.md` |
| `ros2_ws/src/irobots_bringup/` | new (launch/config/urdf) | C28 | reproducible startup | CI, `reproduce.sh` |
| `config/robot.yaml` | new | C06, C49 | the single source of robot geometry/limits | IK, firmware, safety |
| `config/scene.yaml` | new | C18, C21, C22 | table, workspace, camera pose, rim | sim, calibration, perception |
| `config/perception.yaml` | new | C12, C15 | vocabulary, thresholds, class height priors | detector, height |
| `config/tasks/l1.yaml`, `l2.yaml` | new | C22, C40 | task definitions | eval |
| `firmware/esp32_arm/` | new | C48, C49 | the rewritten controller | arm bridge |
| `tools/` | new | C19, C49, C62 | calibration, config generation, hashing | several |
| `spike/` | new (temporary) | C01-C05 | the gates; archive after Phase 1 | nothing |
| `docs/RESULTS.md` | new | C47 | the artifact | — |
| `docs/ARCHITECTURE.md` | new | — | the design contract the code follows | — |
| `docs/hardware/CAMERA_CAPTURE.md` | new | C19 | the real-frame protocol | — |
| `legacy/` | new (move) | C65 | freeze the old code with a note | nothing |
| `agent_harness.py`, `orchestrator.py`, `sam2_processor.py`, `config.py` | **moved** -> `legacy/` | C65 | superseded | nothing |
| `src/micro_controller/ardunio/...ino` | **moved** -> `legacy/` | C65 | superseded by the firmware rewrite | nothing |
| `src/micro_controller/esp32/robot_arm_esp32.ino` | **moved** -> `legacy/` | C65 | superseded | nothing |
| `src/vision-AI/vison/` | **moved** -> `firmware/esp32_cam/` | C19 | kept, de-duplicated, needed for the real-frame capture | C19 |
| `src/vision-AI/config/config.py` | **delete** (duplicate of root `config.py`) | C65 | two sources of truth | nothing |
| `robotic arm schematics/` | keep | — | the only source of link lengths; STLs are the provenance | `config/robot.yaml` |
| `README.md`, `docs/architecture.md` | rewrite | C65, C47 | the old docs describe a design that was never built | — |

---

## G. Alternatives and decisions

| ID | Decision | Options considered | Chosen | Why | Rejected because | What changes my mind |
|---|---|---|---|---|---|---|
| D1 | Goal | demo slice / research artifact / framework / production | research artifact | operator | the others are not falsifiable | — |
| D2 | Simulator | MuJoCo / gz-harmonic / both to parity / MuJoCo-no-ROS | MuJoCo primary + ROS 2 skin; gz optional | contact quality, determinism, iteration speed, RAM all point the same way; the research questions are not about the simulator | gz: DART/PGS for a 4 cm cube; ROS clock coupling; Docker+lavapipe sink | a gz grasp benchmark beating MuJoCo's on C01 |
| D3 | ROS 2 depth | nodes-only / node graph / thin wrappers | thin wrappers over pure-Python core | the research code must test in ms without a graph; ROS 2's per-hop cost is <1% of a turn, so it cannot be justified on latency | it earns its place as the interface contract for a future real arm and as tooling | needing distributed nodes before Phase 6 |
| D4 | Budget | 1-2 / 3-4 / 6-8 weeks | 6-8 weeks, with an honest 10-13 if gates fail | — | — | — |
| D5 | LLM at eval scale | local-only / hosted / two tiers | hosted OpenRouter for all eval runs | 3600 calls x 30 s on a 12B CPU model = 30 h of generation | local-only is a schedule trap | OpenRouter cost/latency making hosted worse |
| D6 | Perception | YOLO-World+depth / YOLO-World+monocular / closed-set+SAM | YOLO-World + monocular | operator; keeps the real camera path open | depth would have been easier but adds hardware | — |
| D7 | Plan path | docs/plans / docs/design / root | `PLAN.md` at root | operator | — | — |
| D8 | Firmware | sim-only / include firmware / full real loop | include, but parallel and protocol-tested against an emulator | the operator wants the firmware; the emulator removes its hardware dependency | — | — |
| D9 | Git | branch+PR / commit to main / new repo | commit to main per phase | operator | — | — |
| D10 | Tasks | L1 / L2 / L3 / L4 | L1 + L2 graded, L3 stretch | operator | — | — |
| D11 | Autonomy | per-step / plan-then-autonomous / fully autonomous | plan-then-autonomous | operator; keeps the auditability argument | — | — |
| D12 | Camera mount | tilt allowed / top-down / no mount | plan specifies 35 deg tilt at 0.55 m | operator; a top-down camera cannot observe height at all | — | — |
| D13 | Arm | fabricate 5-DOF from CAD | 5-DOF with a yaw-pitch wrist, specified in `config/robot.yaml` with provenance | the CAD's joint axes/limits are unrecoverable; a yaw-pitch wrist is what makes the jaw yaw controllable | — | — |
| D14 | Real camera data | yes / no / deferred | yes, ~300 frames in Phase 2 | it is the only hardware available and needs no assembly; renders are optimistic | — | — |
| D15 | Environment | Docker-first / native-first / container-last | container **last**, image as the canonical env | the first Docker build costs 4-8 GB and 20-60 min *before* anything is known to work | — | — |
| D16 | Action encoding | skills only / raw coords only / both as a matched experiment | skills, with raw coords as the *matched* control | I1 + a fair RQ1 | — | — |
| D17 | Grading | perceived only / privileged only / both labelled | both, labelled | the truth needs ground truth; the transferable claim needs perception | — | — |
| D18 | Firmware e-stop | hold pose (new) / drop PWM (legacy) | hold pose | dropping PWM drops the arm and the object; holding is safer for a desk arm | the legacy behaviour is at `V1_arm_controller_full_status.ino:313-323` | a real e-stop must be verified on hardware — flagged as unverified |
| D19 | Wire protocol | WebSocket / TCP+NDJSON / UDP | TCP + NDJSON, joint space, sequence ids | ~50 lines of Arduino, debuggable with `nc`; UDP's loss is unacceptable for commands | — | — |
| D20 | IK location | host / device | host | one source of truth, exactly testable; the firmware owns limits as defense in depth | — | — |
| D21 | Determinism promise | bit-identical everywhere / action-log hash / statistical | action-log hash + statistical cross-machine | the LLM is nondeterministic and the model may change under you | — | — |
| D22 | CI | with rendering / logic+physics only | logic + physics in CI; rendering nightly | a software rasteriser in CI is slow and flaky | — | — |
| D23 | Cut order | drop P7 first / drop P5 first / drop P6 first | P8 -> P7 -> P5-reduced -> P6 | P5 is the differentiator (it is the *research*), P6 is a parallel workstream | — | — |
| D24 | Licensing | ignore / disclose | disclose AGPL-3.0 (Ultralytics) in the report and the README | a public artifact must not launder a copyleft dependency | — | — |
| D25 | Success criteria | vague / numeric | the seven in section A | an artifact needs falsifiable numbers | — | — |

---

## H. Tests and validation

**What exists today:** nothing. No test files, no test runner, no CI, no `requirements.txt`; `docs/TODO.md` is empty. Baseline = 0% coverage of everything.

| ID | Level | Scenario | Expected | Proves |
|---|---|---|---|---|
| T01-T02 | unit | robot.yaml loads; provenance present; link lengths match the STL measurements | 194/130/34 mm; no `assumed` value without a tag | C06, section D.6 |
| T03-T05 | unit | FK o IK round-trip; unreachable target; r->0 singularity | <1e-6; clamped, no NaN | C07 |
| T06 | unit | trajectory profile | monotonic, within limits, duration a pure function | C08 |
| T07-T09 | unit | safety supervisor | every out-of-bounds pose rejected; table hit rejected; NaN never passes | C09, I2 |
| T10 | unit | every skill has a precondition and a predicate | registry is total | C10 |
| T11-T12 | unit | unknown config key; grep for literal IPs | fails loudly; zero hits | C11 |
| T13-T14 | unit | known object detected; unseen class | detected; no hallucinated label | C12 |
| T15-T17 | unit | analytic vs marker calibration on synthetic frames | analytic exact; marker within 8 mm; refuses on high RMS | C13 |
| T18-T19 | unit | tracker id across reordering; id lost | id persists; skills refuse `unknown_target` | C14, I1 |
| T20-T22 | unit | probe height on a synthetic cube; prior path; VP path | within 5 mm; table value; bias reported and disabled | C15 |
| T23-T24 | unit | grasp candidate opening and contact point | opening >= width; XY = corrected contact | C16 |
| T25 | unit | every outcome predicate | correct on synthetic before/after scenes | C17 |
| T26 | unit | object outside the rim | `object_knocked_away` | C18 |
| T27-T28 | integration | real frame capture; same seed -> same final state | 300 frames; identical hash | C19, C21 |
| T29-T32 | integration | scene reproducibility; sim time == image stamp; paused -> identical frames; msg round-trip | all equal | C22-C24 |
| T33-T35 | integration | unreachable target deadline; rejection not forwarded; JSONL-only replay | deadline fires; no forwarded rejection; replay works | C25-C27 |
| T36 | integration | both launch files boot and publish | yes | C28 |
| T37-T39 | unit | three LLM profiles; malformed/partial/index-1 tool calls; fallback | all answer; all handled; fallback logged | C29, C30 |
| T40-T43 | unit | plan naming an absent object; one call per turn; repeat detection; unknown id | rejected; one; escalated; refused | C31, C32 |
| T44-T45 | integration | memory scope; **scripted policy >=80% on L1** | 30 seeded scenes | C33, C34, SC1 |
| T46-T47 | integration | full L1 episode; one skill per observation | completes; never two | C36, I3 |
| T48-T49 | integration | approve-then-autonomous; e-stop key | both work | C37 |
| T50-T52 | integration | replay hash; tampered transcript; e-stop abort | identical hash; detected; aborts in one cycle | C38, C39 |
| T53 | unit | success predicate on synthetic scenes | in-bin != near-bin != in-air | C41, I5 |
| T54 | unit | McNemar / bootstrap on synthetic tables | matches hand-computed values | C42 |
| T55-T59 | integration | firmware compile + logic; generated header matches config; emulator passes the suite; both implementations pass; bridge round-trip | all pass | C48-C52 |
| T60-T65 | CI | image build; reproduce.sh; weight hash; CI time; legacy unreferenced | pass; recorded; <10 min; zero refs | C60-C65 |

**Non-test validation (manual/metric):** the P0 gate report (C05) is the go/no-go artifact; `docs/RESULTS.md` tables are the scientific output; the real-frame comparison is the only cross-domain check.

**Statistical honesty note, to be printed in the report before any data is collected:** with 30 paired scenes, McNemar resolves only ~25-30 pp differences. A 10-15 pp effect will be reported as inconclusive, not as a trend. This is stated *before* running, so it cannot be rationalised afterwards.

---

## I. Edge cases (derived from this design, not generic)

| Case | Where it bites | Expected behaviour | Handled by |
|---|---|---|---|
| Detection id shifts between frames | `worker.py` skill args | tracked id used; a lost id -> `unknown_target`, never a re-target | C14, T18-T19 |
| Zero detections (a class the detector misses) | perception -> worker | explicit `no_target` outcome; the planner may re-observe, but the episode deadline bounds it | C17, C32, I4 |
| Planner names an object that does not exist | `planner.py` | plan rejected at approval; replan requested | C31, T40 |
| Every step is rejected by the supervisor (e.g. a yaw convention error) | `safety_node.py` | a distinct `rejected_infeasible` counter, and a **pre-flight feasibility check of the whole nominal plan** before the episode starts | C09, C26 |
| LLM repeats the same skill+target | `worker.py` | repeat detection escalates to replan; the turn budget is calibrated from the scripted run, not fixed at 10 | C32, C34 |
| Object knocked off the table | outcome vocabulary | `object_knocked_away`, distinguishable from `object_lifted` because the table has a rim | C17, C18 |
| Arm/link intersects the table | MuJoCo | the safety supervisor checks link-vs-table, not just the TCP; a hit is rejected *before* execution (a NaN divergence is a sim-failure symptom, not a first line of defence) | C09, T08 |
| Sim diverges (NaN) | MuJoCo | detect "no pose change for N steps" -> abort the episode, mark `sim_diverged` | C40 |
| An unreachable waypoint hangs the controller | `control_node.py` | per-skill deadline enforced by the driver; the episode continues | C25, T33 |
| One trial of 30 hangs | `runner.py` | subprocess + hard kill + per-trial checkpoint; the other trials survive | C40, SC4 |
| Solver ejecting a grasped cube | physics | a mu and contact-stiffness sweep in C01, **reported**, not hidden; slip/ejection rate is an output | C01 |
| "Attach the object on contact" temptation | physics | **rejected as dishonest** — an attached object moves by construction, so "moved with the gripper" becomes a tautology and grasp success would measure the attach trigger. If ever used it must be a separately labelled upper-bound arm, excluded from every perception-driven claim | section D.1, C01 |
| L2's bins: the interior of a container is not a table-plane footprint | perception | a second, harder perception problem with no height tier; L2's success is graded on the object's *final world pose* (privileged) and the drop target is specified as a marked surface region, not a detected container | C22, C41, I5 |
| Residual contacts/objects persist between trials | scene reset | one sim instance per trial (~5-10 s) is the cost; a rigorous spawn/clean procedure is implemented and tested | C22, C40 |
| OpenRouter model silently updated | reproducibility | the model id **and** its provider fingerprint are recorded per episode; a provider change invalidates comparison and is reported | C29, C62 |
| Ultralytics AGPL-3.0 in a public artifact | licensing | disclosed in the README and the report | C47 |
| LLM is nondeterministic between two runs of the same seed | reproducibility | replay-from-transcript is the reproducibility mechanism; the live-model numbers are reported with CIs, never as bit-identical | C38, D21 |
| The arm is not assembled; axes/limits are a design choice | all claims | every grasp claim is scoped to the specified robot; a "not verified without the physical arm" list is published | section D.6, C47 |

---

## J. Impact, second-order effects, compatibility

**Blast radius.** The legacy Python (4 files) and both sketches are superseded and frozen; `docs/architecture.md`, `docs/hardware.md`, `docs/roadmap.md` and the old README describe a system that never ran and must be rewritten or archived. `config.py` is duplicated at `src/vision-AI/config/config.py` — the duplicate is deleted. The camera firmware is moved and de-duplicated (it is currently byte-identical in two places, and `src/vision-AI/vison/` is missing the gitignored `camera_index.h`, so it cannot compile as-is).

**Compatibility.** No external consumer exists (no releases, no tags, one branch, no downstream). The `robotic arm schematics/` CAD and the ESP32-CAM firmware are retained unmodified except for the camera mount spec.

**Concurrency.** The perception node and the control node run in separate processes (so a leak cannot take the sim down). The sim is the single writer of joint state; the safety node is the only path to the controller; the recorder writes to a bounded ring. No shared mutable state across processes except via ROS 2 topics and the JSONL.

**Performance and cost.** Per turn: 200-400 sim steps (0.2-0.4 s) + detection (0.2-0.6 s) + one hosted LLM call (2-5 s) => ~3-6 s/turn, 12-20 turns/episode => 60-120 s/episode, ~150 episodes => 2.5-5 h and a few USD of API spend. RAM 1.2-1.8 GB. Disk: image 4-8 GB + weights <1 GB + episodes <2 GB.

**Second-order effects.** (1) The "no coordinate from the model" rule buys precision at the cost of strategy expressiveness (no approach side, no re-grasp orientation) — this must be reported, not hidden. (2) A skills-only interface makes the system harder to extend with novel behaviours than raw coordinates would — mitigated by the fact that a new skill is ~40 lines and the planner's tool schema grows automatically. (3) The simulator becomes a hidden dependency of every claim: a MuJoCo version bump changes the numbers, so the version and the weight hashes are pinned and recorded. (4) The thin-wrapper structure means ROS 2 can be deleted later without touching the science — but only if the discipline holds from Phase 1; a single `import rclpy` in the core breaks that property, so CI enforces it. (5) Building a robot we specified rather than one we measured means the artifact's transfer to real hardware is unproven; the published limitation list is the mitigation, and the "not verified" list must be honoured in the writeup, not just in a footnote.

---

## K. Rollout, rollback, observability

**Rollout order.** P0 gates -> P1 core -> P2 perception (+ the real-frame capture) -> P3 sim node + ROS skin -> P4 agent -> P5 closed loop + TUI -> P6 eval/experiments -> P7 firmware (parallel) -> P8 packaging. Each phase ends with a commit on `main` (D9) whose CI is green.

**Rollback.** Every phase is one commit => `git revert <sha>`. The pinned image is rebuilt from the Dockerfile, so a bad dependency is a one-line pin change. The point of no return is **Phase 6**: once results are published, changing the perception or action encoding invalidates them and requires a re-run. Before Phase 6, any design can be changed for the cost of one phase.

**Observability.**

- Every episode writes: `episode.jsonl` (per-turn: sim time, scene digest, detection list, skill call, safety verdict, outcome, LLM request/response + latency + tokens), `metrics.json`, and the action-log SHA-256.
- Every run writes a `run.json` with: git SHA, image digest, MuJoCo version, model id + weight hash, seed set, and the wall-clock cost.
- The TUI shows the same live. A `runs/` directory is the only place numbers come from; nothing is transcribed by hand.
- A failure signal is defined: a phase is "not done" until its gate table (C05 + the SC table) is green in `runs/`.

---

## L. Risks, assumptions, open questions

| ID | Type | Description | LxI | Mitigation / verification | Owner | Status |
|---|---|---|---|---|---|---|
| R1 | risk | **Grasping a 4 cm cube in simulation is the project.** Contact/slip/ejection may not reach 80% | highxhigh | C01 gate on day 1; fallbacks in order: bigger/boxier object, wider jaw, higher mu, different sim | operator | open |
| R2 | risk | The LLM adds little over a scripted policy -> a null headline | medxhigh | RQ0 is *framed* to make null informative; the artifact reports the delta, not a win | operator | open |
| R3 | risk | Detection at 640x480 misses small objects | medxmed | C02 gate; vocab tuning; the real-frame set as a second opinion; a larger object is an acceptable task change | operator | open |
| R4 | risk | Host OOM-killer picks the agent stack, not our container | medxhigh | cgroup `memory.max=3G`; perception in its own process; ONNX weights | operator | open |
| R5 | risk | 6-8 weeks is optimistic (the honest estimate is 10-13 if gates fail) | highxmed | the Phase 0 gates exist to fail *early*; the cut order (D23) drops P8, then P7, then halves P6 | operator | open |
| R6 | risk | Determinism claims are overstated | medxhigh | D21 promises exactly what is true, with a hash-based check; the "not reproducible" list is published | operator | closed by design |
| R7 | risk | ROS 2 ceremony eats the schedule | medxmed | core is developed host-first (D15); ROS 2 is only the skin; `colcon` touches few files | operator | closed by design |
| R8 | risk | The sim arm is a fabricated robot, so the transfer claim is unproven | highxmed | the "not verified without the physical arm" list is published and the real hardware path is protocol-specified | operator | accepted |
| R9 | assumption | MuJoCo's contact model reaches 80% grasp at mu=0.5 for a 4 cm cube | — | verified by C01, day 1 | operator | to verify |
| R10 | assumption | YOLO-World `s` runs at >=3 fps on CPU at 640^2 with <=1.5 GB | — | verified by C02/C04, day 1-2 | operator | to verify |
| R11 | assumption | The chosen OpenRouter model yields >=95% schema-valid tool calls | — | verified by C03, day 1 | operator | to verify |
| R12 | assumption | The specified 5-DOF arm with a yaw-pitch wrist can reach every point of the workspace disk | — | verified by C06/C07 asserts + a workspace-reachability sweep | operator | to verify |
| R13 | assumption | The ESP32-CAM can be mounted at the specified 35 deg pose and captured programmatically | — | C19; the sketch already serves `/capture` | operator | to verify |
| R14 | question | Which OpenRouter model is the default for the worker vs the planner? | — | resolved by C03's comparison; a fast model for the worker, a stronger one for the planner | operator | open, non-blocking |
| R15 | question | Is the L2 "bin" a marked surface region or a modelled container? | — | default: a marked surface region (a container's interior is a second perception problem with no height tier) | operator | open, non-blocking |

**Pre-mortem: assume this shipped and failed badly. Ranked causes.**

1. **The grasp never worked.** 80% on L1 was never reached; the artifact reports a loop that gestures at objects. *Early signal:* C01's grasp rate. *Mitigation:* C01 gates everything; the fallback ladder (bigger object -> jaw geometry -> mu -> simulator) is decided at that point, when a day has been spent, not six weeks in.
2. **The LLM contributed nothing and the report says so.** *Early signal:* RQ0's first run. *Mitigation:* it is framed as a measurement; a null result with a cost/latency accounting is a publishable outcome, and the scripted policy remains a useful artifact.
3. **The numbers were not reproducible.** *Early signal:* a replay that does not reproduce the action hash. *Mitigation:* the hash check is a test (T50) and `reproduce.sh` runs in CI.
4. **The perception numbers were optimistic.** *Early signal:* the real-frame set scoring materially worse than the renders. *Mitigation:* the real-frame evaluation is in Phase 2, before any claim is made; both numbers are reported.
5. **The scope ate the schedule and the firmware never landed.** *Early signal:* Phase 3 running late. *Mitigation:* the cut order drops P8 -> P7 -> halves P6; the firmware is explicitly the first thing to go.
6. **A reviewer attacks "a simulation of a robot that does not exist".** *Early signal:* the writeup. *Mitigation:* the robot is fully specified and published (`config/robot.yaml` with provenance); the "not verified" list is explicit; the cross-simulator check (optional) and the real-frame perception check are the honest hedges.
7. **The safety supervisor blocked everything and the agent looked broken.** *Early signal:* `rejected_infeasible` counters > 0. *Mitigation:* the pre-flight feasibility check of the whole nominal plan before the episode starts; the counter is a first-class metric.

---

## M. Decision log

- **D1-D15** as tabulated in section G, decided by the operator across five question rounds (2026-10-22), with D2, D6-priority, D15, D16 and D21 revised after the red-team pass.
- **Reconsiderations on the record:** the operator initially chose "gz-sim + ROS 2 Jazzy from day one" and "monocular Z"; the red team's evidence moved D2 (MuJoCo primary) and inverted the Z priority (probe first, VP last). Both changes are recorded with their reasons rather than silently applied.
- **Rejected ideas and why:** (1) *Attach the object on contact* — dishonest and it makes the verification tautological. (2) *MoveIt 2* — 5 DOF does not need it. (3) *Device-side IK* — duplicates kinematic constants onto an MCU and makes them untestable. (4) *WebSocket* — NDJSON over TCP is smaller and debuggable. (5) *STL meshes in the sim* — expensive to software-render; the STLs stay as provenance. (6) *`MimicJoint` for the parallel jaw* — a known integration trap. (7) *A fixed 10-turn cap* — truncates successful long episodes; the budget is calibrated per task from the scripted run. (8) *The legacy 35x35 cm square workspace* — physically unreachable at 495 mm. (9) *A hardcoded turn cap as the only loop guard* — the episode deadline and the divergence detector are the real guards.

**Changelog**

- Round 1: intake, tier proposal, recon (4 parallel investigators) -> 13 findings that changed the design.
- Round 2: 7 decisions (goal, stack, budget, LLM, hardware scope, perception, plan path).
- Round 3: 7 decisions (ROS 2 depth, camera tilt, arm state, cloud LLM, task suite, autonomy, git).
- Round 4: environment + RAM (Docker, laptop irrelevance, 5.9 GiB).
- Round 5: red-team pass -> 6 reversals, 3 new open questions, simulator and LLM-scale decisions re-opened and settled.
- Round 6: plan drafted; 4 investigators' claims spot-checked against source.

---

## N. Approval record

**Readiness checklist:** every blocking question answered (D1-D15; R14/R15 are non-blocking and defaulted); every success criterion maps to a test or a gate; every change has file, what, how, why-not-alternative, test and risk; every file in section F appears in section E and vice versa; before/after flows reference verified symbols (`sam2_processor.py:120-121`, `agent_harness.py:120/128`, `orchestrator.py:18`, `V1_arm_controller_full_status.ino:166/313-323`, `CameraWebServer.ino:98`); four real alternatives compared including "do less" (D23 cut order) and the simulator swap; edge cases derived from this codebase; the test list covers section I and L; rollback and observability stated; assumptions have day-1 verification steps; cold-read pass run.

**Accepted risks (explicit):** R8 (the robot is specified, not measured — the transfer claim is unproven and the limitation list is published); R5 (6-8 weeks is optimistic); R2 (the LLM may add little; that is a publishable null); the AGPL-3.0 dependency is disclosed rather than avoided.

**Operator approval:** 2026-10-22 — "write the plan and other .md files and start the full implimentation".
