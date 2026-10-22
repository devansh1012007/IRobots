# Phase 0 gate report

**Date:** 2026-10-22 · **Plan:** [`../PLAN.md`](../PLAN.md) §E, changes C01-C05
**Machine:** `devansh-ThinkBook-16-G7-ARP`, Ubuntu 24.04, Ryzen 5 7535HS,
12 vCPU, 12 GiB RAM (5.9 GiB available), 29 GB free disk, **no GPU**
(Mesa lavapipe software Vulkan).

Phase 0 exists to answer one question before any ROS 2, any LLM, or any
perception code is written: **can this arm pick a 4 cm cube up?** If not,
nothing downstream matters. The gates are cheap, they run in minutes, and they
are the reason the hard problems were found on day one instead of in week six.

---

## Gate results

| ID | Gate | Threshold | Result | Verdict |
|---|---|---|---|---|
| C01 | Grasp rate, 4 cm cube, ground-truth poses | >= 80% at mu=0.5 | **100%** (50/50) | **PASS** |
| C01b | Grasp rate across the friction range | informative | 0% at mu=0.1; 100% at mu >= 0.2 | see below |
| C01c | Analytic FK vs simulated FK | < 1e-6 m | **0.0000 um** over 300 random poses | **PASS** |
| C02 | Detection rate and XY error on rendered frames | >= 95%, p95 <= 8 mm | *not yet run* | PENDING |
| C03 | LLM tool-call schema validity | >= 95%, <= 5 s median | *blocked: no `OPENROUTER_API_KEY` in the environment* | BLOCKED |
| C04 | Peak RSS of the full stack | < 2500 MB | **1815 MB** | **PASS** |
| C05 | This report | - | - | done |

**C01, C02 and C04 are the gates that gate the project. C01 and C04 pass. C02
is the next thing to run; C03 needs an API key.**

---

## C01 - the grasp benchmark

`spike/grasp_bench.py`. The policy uses **ground-truth** object poses on
purpose: that isolates *physics* from *perception*, so a low score tells you
which of the two to fix. The perception half is measured separately by C02.

The object is **never attached to the gripper**. A grasp succeeds because
friction holds it. Attaching the object would make "the object moved with the
gripper" a tautology and would measure the attach trigger rather than
grasping, so it is not an option.

| mu (effective, both geoms) | grasp rate | lifted | held | mean slip | wall time |
|---|---|---|---|---|---|
| 0.10 | **0.0%** | 0.0% | 100% | 1.7 mm | 4.6 s / 50 trials |
| 0.20 | **100.0%** | 100.0% | 100% | 1.5 mm | 3.5 s / 50 trials |
| 0.35 | **100.0%** | 100.0% | 100% | 1.9 mm | 3.4 s / 50 trials |
| 0.50 | **100.0%** | 100.0% | 100% | 1.7 mm | 3.1 s / 50 trials |
| 0.80 | **100.0%** | 100.0% | 100% | 1.6 mm | 3.5 s / 50 trials |

Raw data: `runs/spike/grasp/grasp_bench.json` (250 trials, every field).

### What the numbers mean

**The physics is not the bottleneck.** At realistic friction the grasp is
reliable and the mean slip is under 2 mm. That is the good news for the
project, and it is also a warning: the difficulty in this artifact has to come
from somewhere else, and it must come from **perception and decision making**,
which is where the research questions live. The ablations inject calibration
noise and perception error for exactly this reason.

**There is a sharp friction cliff between mu=0.1 and mu=0.2.** At mu=0.1 the
grasp rate is 0%, not 50%: the object slips out of the jaws immediately. This
is a real property of the physics, it is reported rather than hidden, and it
sets the boundary of the operating envelope.

**The mu sweep initially measured nothing, and that is worth recording.** The
first version of the sweep set the *cube's* friction and left the finger pads
at 1.4. MuJoCo combines two contacting geoms' sliding coefficients by taking
the **maximum**, so the pads masked the sweep entirely and every mu reported
100%. The sweep now sets both geoms. A parameter sweep that cannot fail is not
a measurement.

### A note on how hard this task is

A 100% grasp rate means the *physical* part of pick-and-place is solved in
simulation. That is the correct starting point for a research artifact: the
physics provides a stable substrate, and the interesting variance is injected
by the perception and decision layers where the research questions live. If a
later phase needs a harder task, the levers are object size, payload mass, and
friction - not a weaker controller.

---

## C01c - kinematics agreement

The analytic kinematics (`irobots_core.kinematics`) and the MuJoCo model
(`irobots_core.sim.mujoco_model`) are independent implementations of the same
chain, and they must agree exactly. Over 300 random joint poses:

* max tip position error: **0.0000 um**
* max jaw-closing-direction error: **0.0000 um**

This is a much stronger result than the 1e-6 m the plan required, and it is
not free: five separate bugs had to be found to get it (see below). The test
lives in `ros2_ws/src/irobots_core/tests/test_kinematics.py`.

### The five bugs this gate caught

Each of these was a silent failure - the code ran, the arm moved, and the
numbers were wrong:

1. **Joint limits were in the wrong unit.** `config/robot.yaml` stored limits
   in radians, but the loader read them as degrees, so `base_yaw` was limited
   to +/- 3 degrees and nothing was reachable. Fixed by storing limits in
   degrees (human-readable) and converting in the solver.
2. **The IK fixed the elbow branch a priori.** Reporting `joint_limit` when a
   valid solution existed on the other branch is a false negative that makes a
   robot look broken. The solver now tries both branches and picks by
   joint-limit feasibility, preferring the higher elbow.
3. **The wrist-yaw solver bisected a discontinuous function.** The azimuth
   error `wrap(closing - target)` jumps at 180 degrees, so over a 340 degree
   joint range the signs at the ends could match even with a valid solution in
   between, and the solver returned a joint limit instead of the answer. It
   now solves the exact signed angle about the approach axis, and snaps to the
   nearest *achievable* closing direction when the request is out of range.
4. **The MJCF put each joint on the wrong body.** MJCF allows one joint per
   body, and a body's joint rotates that body. The original chain attached
   the shoulder joint to the body whose origin is the elbow, so the upper arm
   never rotated. The arm still compiled and still moved - just wrongly.
5. **Adjacent links overlapped at the joints, and the arm fought itself.**
   Capsules of radius 12 mm and 11 mm meeting at a shared origin penetrate,
   and the resulting contact torque held the base yaw at zero no matter what
   the actuator commanded. Fixed with explicit `<contact><exclude>` pairs.

Two more geometry bugs were found by the same gate:

6. **The finger pads' inner faces were 8 mm closer than commanded**, so the
   jaws opened 37 mm - narrower than the 40 mm cube - and the gripper shoved
   the object away instead of straddling it. The jaw stroke was also widened
   from 45 mm to 70 mm: a gripper that opens only 5 mm wider than its payload
   has no tolerance for a 3 mm positioning error.
7. **The on-axis housing and the wrist sphere sat exactly at the grasp
   point**, so they hit the top of the object before the jaws could reach its
   middle. The housing now stops 34 mm short of the grasp point and the pads
   reach back to meet it, which is how a real parallel-jaw gripper is built.

And one benchmark bug:

8. **The settle criterion ignored the fingers**, so a "close the jaws" phase
   returned after ~25 steps - long before the jaws had closed - and the lift
   failed for a reason that had nothing to do with the grasp.

---

## C04 - the memory budget

The binding constraint on this machine is RAM, not disk: 12 GiB total with
about 5.9 GiB available, and the Docker agent stack must keep running.

| Stage | RSS |
|---|---|
| python baseline | 12 MB |
| + numpy | 27 MB |
| + mujoco | 48 MB |
| + core (robot, scene) | 50 MB |
| + mujoco scene + physics | 65 MB |
| + torch | 255 MB |
| + YOLO-World (loaded) | 1060 MB |
| + YOLO-World (inference) | 1075 MB |
| **peak (transient during inference)** | **1815 MB** |

Budget 2500 MB. **PASS**, with 685 MB of headroom. The eval harness will
additionally run inside a cgroup with `memory.max=3G`, so that an out-of-memory
event kills the eval rather than the host OOM-killer picking the agent
stack's qdrant or postgres as the victim.

---

## C03 - blocked

`spike/llm_probe.py` needs `OPENROUTER_API_KEY`. There is no such variable in
the environment, so the model-comparison gate cannot be run yet. The plan
already assumed an OpenRouter key (D5); it just is not exported in this shell.

**To unblock:** export the key, then run `python spike/llm_probe.py`. Nothing
else in Phase 0 depends on it.

---

## Environment findings worth keeping

* **`torchvision` must come from the CPU index and must match `torch`.** The
  PyPI build of the same version is the CUDA one, and ultralytics
  auto-installs it when it is missing, which produces the misleading error
  `operator torchvision::nms does not exist` at inference time.
  `tools/install_deps.sh` now installs it with `--reinstall-package
  torchvision` from the CPU index, and ultralytics' auto-sync is disabled.
* **Installs go through `tools/install_deps.sh`**, which retries with
  exponential backoff, keeps a persistent package cache so an interrupted
  download resumes, and verifies the result by importing every module.
* **Software rendering works.** Mesa lavapipe is present, so MuJoCo's
  offscreen renderer runs headless on this GPU-less box.

---

## What Phase 0 does NOT establish

* Nothing about the real arm. The robot is **not assembled**. Every number
  here is about the robot *specified* in `config/robot.yaml`.
* Nothing about perception. C02 is the next gate, and until it passes, no
  claim about detection or localisation is supported.
* Nothing about the LLM. C03 is blocked on a key.
* Nothing about the closed loop. C01 drives the arm with a scripted policy on
  ground-truth poses. The agent loop does not exist yet.

## Next

1. **C02** - render the same 50 poses, run YOLO-World, measure the detection
   rate and the XY error of the front-bottom-edge homography, and verify the
   D/2 correction. This is the second gate.
2. **Phase 1** - the pure-Python core proper: safety supervisor, trajectories,
   skills, with the unit tests the plan lists (T01-T12).
3. **C03** - as soon as the API key is exported.
