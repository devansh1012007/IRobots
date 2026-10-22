# IRobots v2

A robotic arm you talk to instead of program - rebuilt as a **reproducible
research artifact**.

The deliverable is evidence, not a demo. Four research questions are answered
with numbers, and every headline result must reproduce on a machine with
**no arm, no camera, and no GPU**.

> **Status: Phase 0 complete.** The physics and kinematics gates pass. The
> perception, agent and evaluation phases have not run, so the research
> questions are not yet answered. See [`docs/RESULTS.md`](docs/RESULTS.md) for
> what exists and what does not.

## What it is

An LLM is told a task in plain language. It picks the next atomic action from
a fixed skill library. Deterministic code turns that into motion. The camera
observes the outcome, and the loop continues.

```
operator ─▶ planner (LLM) ─▶ worker (LLM) ─▶ one skill call
                                                 │
                        perception ──▶ safety supervisor ──▶ arm
                            ▲                                  │
                            └──── verification ◀── new frame ◀─┘
```

The one rule that shapes everything: **the LLM never emits a coordinate.** It
selects a skill and names an object; the host derives every position from a
measurement. That is what makes a miss attributable to calibration or
kinematics instead of to a model that guessed.

## Where it actually stands

| Piece | Status | What that means concretely |
|---|---|---|
| Arm model + analytic kinematics | **Working** | 5 DOF + parallel jaw, generated from `config/robot.yaml`. Analytic and simulated kinematics agree to 0.0000 um over 300 random poses. |
| Grasp physics | **Working** | 100% grasp rate on a 4 cm cube at mu >= 0.2 (250 trials), mean slip < 2 mm, with a measured friction cliff below that. |
| Perception | Not yet | YOLO-World is installed and the homography design is fixed; the gate that measures it has not run. |
| Agent loop | Not yet | The skills, the safety supervisor and the planner/worker split are designed; they are not written. |
| ROS 2 skin | Not yet | The interface is fixed. ROS 2 is deliberately *not* in the algorithms. |
| ESP32 arm firmware | Not yet | The protocol is designed; the rewrite is a later phase and nothing depends on it. |
| ESP32-CAM firmware | **Working** | The camera node runs today. It is the only physical hardware the project uses. |
| Mechanical design | **Working** | The SolidWorks assembly and the printable STLs are in `robotic arm schematics/`. |

## Running it

```bash
tools/install_deps.sh        # retries, resumable downloads, verifies the result
python spike/grasp_bench.py  # the physics gate: 250 trials in ~17 seconds
python spike/mem_bench.py    # the memory budget
```

`spike/REPORT.md` is the gate report, with the numbers and the eight bugs
these gates caught.

## Layout

```
PLAN.md                  the plan: design, phases, decisions, risks
AGENTS.md                working rules for humans and agents
config/                  robot.yaml, scene.yaml, perception.yaml  (single sources of truth)
ros2_ws/src/irobots_core/    pure Python: kinematics, safety, skills, perception, agent
ros2_ws/src/irobots_nodes/   thin rclpy wrappers
spike/                   the Phase 0 feasibility gates
tools/                   install_deps.sh, fetch_weights.sh  (resilient installers)
firmware/esp32_cam/      the working camera firmware
legacy/                  the original hackathon code, frozen
docs/                    ARCHITECTURE.md, RESULTS.md, hardware/
robotic arm schematics/  the SolidWorks assembly and the STLs
```

## Why the robot is "specified" rather than measured

The arm is **not physically assembled**. The SolidWorks assembly is an opaque
binary with no recoverable joint frames, so the joint axes, offsets and limits
are *our choices*, recorded in `config/robot.yaml` with a `provenance` tag on
every constant: `measured` (read off the STLs), `design` (chosen, with the
reason), or `assumed` (a guess that needs a real arm).

Every grasp claim in this repository is a claim about that specified robot.
[`docs/RESULTS.md`](docs/RESULTS.md) carries the explicit
"not verified without the physical arm" list, and it is binding.

## Honest notes

* The simulator is an instrument, not the subject. The research questions are
  about the LLM, perception and control architecture.
* Simulation renders are optimistic: no exposure drift, no JPEG artifacts, no
  motion blur. A real-frame set is planned precisely to measure that gap.
* Ultralytics (YOLO-World) is **AGPL-3.0**. Disclosed, not avoided.
* The planner is called when the task changes or a step fails; the worker
  runs every turn. The LLM's contribution is measured, not assumed - a null
  result is a publishable outcome.

## License

MIT, except the camera firmware under `firmware/esp32_cam/`, which derives
from Espressif's CameraWebServer example and stays under Apache 2.0.
Attribution is in [`NOTICE`](NOTICE).
