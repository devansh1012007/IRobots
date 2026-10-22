# AGENTS.md — working rules for this repository

Read this before touching anything. It applies to humans and to agents alike.

## What this project is

IRobots v2 is a **reproducible LLM-manipulation research artifact**. A robotic
arm is told a task in plain language; an LLM picks the next atomic action from
a fixed skill library; deterministic code turns that into motion; the camera
observes the outcome and the loop continues.

The deliverable is **evidence**, not a demo. Four research questions are
answered with numbers, and every headline result must be reproducible on a
machine with **no arm, no camera, and no GPU**.

Read `PLAN.md` for the full design, the phase plan, and the decision log.

## The six invariants

These are enforced in code, not merely documented. Do not weaken them.

| ID | Invariant |
|---|---|
| I1 | The LLM never emits a Cartesian coordinate. Coordinates are derived by deterministic code from a scene, at execution time. |
| I2 | Every commanded pose passes the safety supervisor before execution. A rejection is a first-class outcome the planner sees. |
| I3 | One skill per observation. The worker never chains skills in one turn. |
| I4 | Every episode terminates: per-skill deadline, per-episode deadline, per-trial hard kill. |
| I5 | No metric mixes privileged (simulator ground truth) and perceived quantities. |
| I6 | The sim is stepped deterministically. The LLM's wall-clock latency never changes how much physics time elapses. |

## Architecture rules

- **`irobots_core` is pure Python and must never import `rclpy` or any ROS
  package.** That separation is what makes the science testable without a ROS
  graph, and it keeps ROS 2 deletable. CI enforces this.
- ROS 2 is the *integration skin*: topics, parameters, rosbag2, launch. The
  algorithms do not live in nodes.
- One source of truth per concern. `config/robot.yaml` is the only place the
  arm's geometry, limits and servo mapping exist. The MuJoCo model and the
  firmware header are both generated from it.

## Data rules

- Every physical constant carries a `provenance` tag: `measured` (read off the
  STL meshes), `design` (we chose it because the SolidWorks assembly is an
  opaque binary with no recoverable joint frames), or `assumed` (a guess that
  must be replaced by measurement on the real arm).
- No hardcoded IP, port, path, model id, or API key in source. They come from
  `config/*.yaml` or the environment.
- Every random draw goes through a seeded `numpy.random.Generator` threaded
  through the call chain. No global `random`, no bare `np.random.rand`.
- Results are produced by code into `runs/`. Nothing is transcribed by hand.

## Testing rules

- A test that cannot fail is not a test. Prove it: revert the fix and watch
  it fail.
- Tests assert behaviour, not implementation details, and are deterministic.
- Never weaken, skip, or delete an existing test to get green.
- Run the tests you wrote and paste the **actual** output. Never report a pass
  you did not observe.
- CI is logic + physics only. Rendering tests are local/nightly: a software
  rasteriser in CI is slow and flaky.

## Traceability

- Code comments and commit messages carry the plan change ID:
  `PLAN:irobots-v2:C##`.
- Claim types are tagged in design docs: `[FACT path:line]`, `[ASSUMPTION]`,
  `[OPINION]`, `[DECISION D#]`.

## Install rules

Never run a bare `pip install` or `uv pip install` from an agent session. Use:

```bash
tools/install_deps.sh          # retries + resumable downloads + verification
tools/fetch_weights.sh         # resumable model-weight fetch
```

Both scripts retry with exponential backoff and resume partial downloads.
Installing torch from PyPI pulls ~2.5 GB of CUDA wheels on Linux; this project
is CPU-only, so the script installs torch from the CPU-only index first.

## Git rules

- Work on a branch. One phase per commit, with the phase's change IDs in the
  message.
- Never commit secrets, model weights, `runs/` output, or `.venv/`.
- Never force-push, rewrite shared history, or bypass hooks.
- Do not commit to `main` until the phase's tests pass.

## The legacy tree

`legacy/` holds the original hackathon code, frozen. It is not imported by
anything and must not be "fixed" — it is kept as a record of what was tried.
The legacy defects it documents (no class labels, `cm = px/30` with the origin
at the image corner, guessed Z, a `validate_and_execute_tool` that reported
success without sending anything) are the ones this project exists to fix.

## Honesty rules

- The arm is **not physically assembled**. Every grasp claim is a claim about
  the robot described in `config/robot.yaml`. The "not verified without the
  physical arm" list in `docs/RESULTS.md` is binding: do not imply the
  hardware was tested.
- The ESP32-CAM firmware works and may be flashed; the arm firmware does not
  yet speak the protocol.
- If a measurement is optimistic (simulation renders have no exposure drift,
  JPEG artifacts or motion blur), say so and report the real-frame
  comparison when one exists.
- Ultralytics (YOLO-World) is AGPL-3.0. Disclose it.
