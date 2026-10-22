# Results

The research report. Every number here is produced by code into `runs/` and
copied from there by hand only in the sense that a human typed the table -
the source JSON is in the same repository and regenerable.

**Status: Phase 0 only.** The gates that have run are the physics and
kinematics gates. The perception, agent and evaluation phases have not run, so
the research questions are not yet answered. This file is the report
*structure* plus the results that exist, and it will be filled in as the
phases land. It is deliberately **not** a summary of hopes.

## What is being asked

| ID | Question | Why it is worth asking |
|---|---|---|
| RQ0 | How much does the LLM add over a hand-written skill script, per unit of latency and cost, and how often does it loop or mis-select an identity? | The honest headline. The LLM's per-turn decision space is about 7 skills x 4 objects = 28 options, and for a pick-and-place task the correct sequence is largely determined by the scene. A null result is still a result. |
| RQ1 | Does a skill-based action encoding beat raw-coordinate tool calling, under calibration error? | The legacy design let the model emit `move_arm(x, y, z)`. The new design lets it name a skill and an object. **The comparison is only fair if both arms get the same Z policy, the same perception, the same supervisor, the same turn budget and the same scenes** - otherwise it measures who gets to pick Z, not the encoding. |
| RQ2 | How much does monocular height uncertainty cost, and does verified probing recover it? | Height is the dominant error mode and it is *unobservable* from a top-down camera. The estimator's error is measurable in simulation because the ground truth is known. |
| RQ3 | What are the failure modes of a closed-loop LLM manipulator, and which are recoverable? | More useful than a single success number. |
| RQ4 | Does a policy validated in one simulator transfer to another physics engine? | A robustness check that comes nearly free once two backends exist. |

### Statistical honesty, stated before the data is collected

With 30 paired scenes, McNemar's test resolves only about 25-30 percentage
points. A 10-15 point effect will be reported as **inconclusive**, not as a
trend. This is written down now so that it cannot be rationalised afterwards.

Every metric is tagged `privileged` (computed from simulator ground truth) or
`perceived` (computed from what the system can actually see). The two are
never mixed.

---

## Phase 0 - the physics gate

Full detail and method: [`../spike/REPORT.md`](../spike/REPORT.md).

### Grasp reliability (C01)

A 4 cm cube, 40 g, grasped with a parallel jaw, driven by a scripted policy on
**ground-truth** poses. The object is never attached to the gripper: the grasp
holds because friction holds it.

| effective mu | grasp rate | mean slip | n |
|---|---|---|---|
| 0.10 | 0.0% | 1.7 mm | 50 |
| 0.20 | 100.0% | 1.5 mm | 50 |
| 0.35 | 100.0% | 1.9 mm | 50 |
| 0.50 | 100.0% | 1.7 mm | 50 |
| 0.80 | 100.0% | 1.6 mm | 50 |

**Reading.** The physical part of pick-and-place is solved in simulation, and
the mean slip is under 2 mm. That is the correct starting point: the physics
is a stable substrate, and the variance in this artifact has to come from
perception and decision making, which is where the research questions live.
The ablations inject calibration noise and perception error for exactly that
reason.

There is also a **friction cliff between mu=0.1 and mu=0.2**: at mu=0.1 the
grasp rate is 0%, not 50%. That is a real property of the physics and it sets
the edge of the operating envelope.

*Method note, because it nearly went unrecorded:* the first version of this
sweep set the cube's friction and left the pads at 1.4. MuJoCo combines two
contacting geoms' sliding coefficients by taking the maximum, so the pads
masked the sweep and every mu reported 100%. A sweep that cannot fail is not
a measurement. Both geoms are now set.

### Kinematics agreement (C01c)

The analytic solver and the MuJoCo model are independent implementations of
the same chain. Over 300 random joint poses:

* max tip position error: **0.0000 um**
* max jaw-closing-direction error: **0.0000 um**

Required by the plan: 1e-6 m. Getting there required fixing five silent
kinematics bugs, listed in the [gate report](../spike/REPORT.md#the-five-bugs-this-gate-caught).

### Memory budget (C04)

Peak RSS of the full stack - simulator, physics, torch, YOLO-World - is
**1815 MB** against a 2500 MB budget, with the machine's agent stack left
running. Steady state after loading the detector is 1075 MB.

---

## Not yet measured

| Result | Status |
|---|---|
| Detection rate and XY error on rendered frames (C02) | not run - the next gate |
| Detection and XY error on **real** ESP32-CAM frames | not run - needs the camera capture protocol in [`hardware/CAMERA_CAPTURE.md`](hardware/CAMERA_CAPTURE.md) |
| Height estimator error against ground truth | not run |
| Closed-loop task success | not run - the agent loop does not exist yet |
| RQ0 - RQ4 | not run |

---

## Not verified without the physical arm

This list is binding. None of these may be claimed.

* That the arm can grasp. The robot is **not assembled**; every grasp claim
  here is a claim about the robot *specified* in `config/robot.yaml`.
* That the e-stop is safe. The firmware's e-stop will **hold** the last pose
  rather than dropping PWM and going limp, but that behaviour is only testable
  against a simulator with no servo inertia.
* Any real-time or throughput claim about the hardware.
* Any torque or grasp-force statement. The simulated finger pads are driven by
  a bounded-force position actuator; a real SG90 or MG996R has deadband,
  backlash and a torque limit that the simulation does not have.
* The camera's own optics: the ESP32-CAM's fixed lens has barrel distortion
  and its auto-exposure shifts with the light, both of which invalidate any
  fixed pixel-to-cm constant.
* Anything about the arm's servos' actual travel, zero offsets or horn
  geometry. The CAD's joint frames are not recoverable from the assembly
  file.

---

## Licensing

Ultralytics (YOLO-World) is **AGPL-3.0**. This is disclosed rather than
avoided: it is a real constraint on a public artifact, and a permissive
alternative would have cost more than it was worth at this stage.
