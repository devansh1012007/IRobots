# Architecture

How the system is put together, and why each boundary is where it is. The
authoritative plan is [`../PLAN.md`](../PLAN.md); this document is the
contract the code follows.

## The shape of it

```
                    operator types a task
                              |
                              v
                    +-------------------+
                    |  planner model    |  task -> ordered list of skills
                    +-------------------+  (no coordinates, ever)
                              |
   camera frame  ------------v------------------->  +------------+
      |                                            |  worker    |
      v                                            |  model     |  scene + plan
  +----------+                                     +------------+  -> ONE skill call
  |perception|  detection, identity, XY, height          |          (no coordinates)
  +----------+                                           v
      |                                          +----------------+
      |                                          | skill executor |  deterministic
      |                                          +----------------+  -> joint trajectory
      |                                                   |
      |                                          +----------------+
      |                                          | safety         |  bounds, joint limits,
      |                                          | supervisor     |  speed, table plane
      |                                          +----------------+
      |                                                   |
      |                                          +----------------+
      |                                          | arm backend    |  mujoco | emulator | esp32
      |                                          +----------------+
      |                                                   |
      +---------- new frame <---------------------------  v
                 (one observation per skill)

              +---------------------------------------+
              | verification: what actually happened?  |  -> back to the planner
              +---------------------------------------+
```

Every box below the planner is deterministic code. The two model boxes are
the only non-deterministic parts, and neither of them is allowed to produce a
coordinate.

## The six invariants

These are enforced in code, not merely documented. CI checks the ones that can
be checked mechanically.

| ID | Invariant | Enforced by |
|---|---|---|
| I1 | The LLM never emits a Cartesian coordinate. | The worker's tool schema has no coordinate parameters; a skill takes a tracked object id and the host derives the pose. |
| I2 | Every commanded pose passes the safety supervisor before execution. | The supervisor sits between the skill executor and the arm backend in the graph. A rejection is a first-class outcome. |
| I3 | One skill per observation. | The worker returns exactly one skill call per turn; the loop asserts it. |
| I4 | Every episode terminates. | Per-skill deadline in the driver, per-episode deadline in the loop, per-trial hard kill in the eval runner (a subprocess with a timeout). |
| I5 | No metric mixes privileged and perceived quantities. | Every metric carries a `privileged` / `perceived` tag and the report prints both. |
| I6 | The sim is stepped deterministically. | Fixed `dt`, explicit step counts, and the sim is **paused while the LLM thinks**. |

**I6 deserves the emphasis it gets.** `joint_trajectory_controller`
interpolates on the *controller's* clock. If the nodes do not all use
`use_sim_time:=true` and the sim is not paused while the LLM is thinking, a
3-second trajectory completes within about 5 ms of simulated time: the arm
teleports the whole path and sweeps the object off the table. The symptom
looks like "the LLM is noisy", but it is trajectory timing error, and it
correlates with the LLM's latency.

## Layer boundaries

### `irobots_core` - pure Python, no ROS

All the algorithms: kinematics, trajectories, the safety supervisor, the skill
library, perception, the LLM client, the planner and worker, the episode
recorder, the arm protocol.

**This package must never import `rclpy` or any other ROS package.** That
separation is what makes the science testable without a ROS graph, and it
keeps ROS 2 deletable if it never earns its place. CI enforces it with a grep.

The consequence is that every algorithm is unit-testable in milliseconds, with
no simulator, no DDS discovery, and no `colcon build`. That is what makes the
evaluation sweeps affordable: 250 grasp trials in 17 seconds is only possible
because the physics harness is a library, not a node.

### `irobots_nodes` - thin `rclpy` wrappers

Each node does three things: subscribe, call into `irobots_core`, publish. No
algorithm lives here. A node is roughly 100 lines.

This is what lets ROS 2 be an *integration skin* - topics, parameters,
rosbag2 for episode recording, launch files for reproducible runs - rather
than a framework the science is written in.

### `irobots_eval` - the experiment harness

Task definitions, the runner, the metrics, the statistics, and the
experiment scripts for RQ0-RQ3. It uses `irobots_core` directly and does not
need a ROS graph for the physics-only conditions.

## The robot

`config/robot.yaml` is the single source of truth. The MuJoCo model and the
firmware's generated header are both built from it, so the analytic
kinematics, the simulator and the (future) firmware cannot drift apart
unnoticed.

**The robot is not physically assembled.** Its joint axes, offsets and limits
are *specified* by us, because the SolidWorks assembly is an opaque binary
with no recoverable joint frames. Every physical constant therefore carries a
`provenance` tag:

* `measured` - read off the STL meshes in `robotic arm schematics/`
* `design` - chosen by us, with the reason recorded
* `assumed` - a guess that must be replaced by measurement on the real arm

The chain is `base_yaw -> shoulder_pitch -> elbow_pitch -> wrist_pitch ->
wrist_yaw`, with `wrist_yaw` at the **tool tip** rotating about the approach
axis. That placement is what makes the jaw's closing direction independently
controllable: for a downward-pointing tool, the wrist yaw moves the tip not
at all and purely rotates the jaws. A yaw-pitch-roll wrist cannot do this -
its roll axis *is* the approach axis, so the jaw direction would be locked to
the reach direction and every grasp would be a corner grasp.

## Perception

Three defects in the legacy pipeline are being fixed, and each fix is
measurable.

1. **Identity.** YOLO-World gives a *label* plus a box plus a confidence. The
   legacy SAM pipeline gave anonymous masks, so "the red block" could not be
   bound to an object at all. On top of that, ids are **tracked** across
   frames: a per-frame detection index changes between frames, and the arm
   would grasp the wrong object.
2. **The coordinate frame.** A table-plane homography maps pixels to
   centimetres in the **arm base frame**. The legacy transform was
   `cm = px / 30.0` with the origin at the image corner.
3. **Height.** There is no depth sensor, so height is either *measured* or
   *assumed*, never guessed silently. The priority order is:
   * **probe** (measure): descend the gripper until the jaw occludes the
     object's top edge, and read the height from that. It reuses the descent
     the grasp needs anyway, and it does not require knowing the footprint
     depth.
   * **class prior** (assume): a per-class height table, used when probing is
     refused or ambiguous.
   * **vanishing point** (estimate): implemented and **disabled by default**.
     The silhouette equation `h = f(H cos t + D sin t)/Z` has two unknowns; on
     a 4 cm cube at 35 degrees it returns +72% error, and the vertical
     vanishing point is off-frame, so it could only come from extrinsics we
     would then have to trust absolutely. It stays in the tree so the
     ablation can report *why* it was rejected.

**The D/2 bias is explicit.** The bottom edge of an axis-aligned box resting
on a table is its *front* bottom edge, so back-projecting it lands D/2 toward
the camera - 2.0 cm for a 4 cm cube. The claim is "the contact point is on the
table plane, offset toward the camera by D/2", not "exactly correct".

**The camera is tilted 35 degrees, never straight down.** A strictly overhead
camera cannot observe object height at all: a vertical object's height
projects to almost no pixels. The legacy design specified straight-down, which
is why its Z was guessed.

## The arm backends

One interface, three implementations:

| Backend | What it is | When it is used |
|---|---|---|
| `mujoco` | The simulator, stepped by a fixed `dt` | The default. Every research number. |
| `emulator` | A Python reimplementation of the ESP32 firmware's state machine, speaking the same wire protocol | Protocol tests, and the hardware-free path to the firmware |
| `esp32` | The real board over TCP | Optional, and **unverified** until the arm is assembled |

The firmware receives **joint angles**, not Cartesian coordinates. The host
solves IK - one source of truth, exactly testable - and the firmware
executes trajectories and owns the limits as defence in depth.

## Determinism: what is and is not reproducible

| Reproducible | Not reproducible |
|---|---|
| The physics, given the same binary, machine and step sequence | The LLM (the provider is nondeterministic and the model may change under you) |
| The policy, given a logged transcript | Pixels across machines and drivers |
| The metrics, given a transcript | The number of steps (perception-driven control flow changes it) |

So the artifact promises an **action-log hash**: a SHA-256 over the ordered
skill calls, arguments and verdicts. Replaying an episode must reproduce the
hash. Cross-machine reproduction is *statistical* (confidence-interval
overlap), and is never claimed to be bit-identical.

## The safety supervisor

A pure function, and the only thing between a confused model and a broken arm.
It checks workspace bounds, joint limits, speed limits, the table plane, and
self-collision, and it returns a verdict rather than raising. A rejection is
a first-class outcome the planner sees.

A pre-flight feasibility check of the whole nominal plan runs before the
episode starts, and a `rejected_infeasible` counter is a first-class metric.
Without it, a configuration error (a wrong yaw convention, say) looks exactly
like an agent that cannot do the task, and the eval reports 0% success for a
reason that has nothing to do with the research question.
