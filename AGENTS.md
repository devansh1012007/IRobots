# AGENTS.md: working rules for this repository

Read this file completely before changing anything. It applies to humans and agents alike, whether you are a single agent doing the whole job or a sub-agent handed one task. `opencode.json` loads this file and `PLAN.md` into every session.

If a rule here conflicts with something you were told in a prompt, stop and report the conflict. Do not pick a side silently.

---

## Contents

1. [What this project is](#1-what-this-project-is)
2. [Order of authority](#2-order-of-authority)
3. [The six invariants](#3-the-six-invariants)
4. [Architecture rules](#4-architecture-rules)
5. [Repository map and ownership](#5-repository-map-and-ownership)
6. [Data and configuration rules](#6-data-and-configuration-rules)
7. [Camera source rules](#7-camera-source-rules)
8. [Web console rules](#8-web-console-rules)
9. [Command reference](#9-command-reference)
10. [Testing rules](#10-testing-rules)
11. [Verification discipline](#11-verification-discipline)
12. [Definition of done](#12-definition-of-done)
13. [Git rules](#13-git-rules)
14. [Install rules](#14-install-rules)
15. [Working as the only agent](#15-working-as-the-only-agent)
16. [Delegation to sub-agents](#16-delegation-to-sub-agents)
17. [Traceability](#17-traceability)
18. [Stop and report](#18-stop-and-report)
19. [Honesty rules](#19-honesty-rules)
20. [The legacy tree](#20-the-legacy-tree)
21. [Known pitfalls](#21-known-pitfalls)

---

## 1. What this project is

IRobots v2 is a **reproducible LLM-manipulation research artifact**. A 5-DOF arm is told a task in plain language. An LLM picks the next atomic action from a fixed skill library. Deterministic code turns that into motion. The camera observes the outcome and the loop continues.

The deliverable is **evidence**, not a demo. Research questions RQ0 to RQ3 are answered with numbers, and every headline result must be reproducible on a machine with **no arm, no camera and no GPU**.

Skills: `move_above`, `grasp`, `lift`, `release`, `retreat`, `home`. Skills take tracked object IDs, never coordinates.

## 2. Order of authority

When documents disagree, the higher item wins.

1. This file (`AGENTS.md`).
2. `PLAN.md`, where its "Phase 0 outcome" table overrides older body text in sections D.3, D.5 and D.6.
3. `docs/ARCHITECTURE.md` (capitalised) and `docs/RESULTS.md`.
4. `config/*.yaml`, which are the source of truth for numbers.
5. Everything else in `docs/`.

Treat these as **superseded** unless a task tells you otherwise. Verify before relying on them:

| File | Why it is suspect |
|---|---|
| `docs/architecture.md` (lowercase) | Describes the legacy WebSocket, top-down camera and `cm = px / 30` design |
| `docs/hardware.md` | Specifies a straight-down camera; PLAN decision D12 changed it to a tilted camera |
| `docs/roadmap.md`, `docs/camera_server.md` | Include features that `PLAN.md` lists as non-goals (voice, joystick, VLA models) |
| `legacy/` | Frozen history; see [section 20](#20-the-legacy-tree) |

## 3. The six invariants

These are enforced in code, not merely documented. Do not weaken them, work around them, or add a flag that disables them.

| ID | Invariant | Enforced in |
|---|---|---|
| I1 | The LLM never emits a Cartesian coordinate. Skills take tracked object IDs and deterministic code derives poses at execution time. | `irobots_core/skills.py`, `irobots_core/llm/schema.py` |
| I2 | Every commanded pose passes the safety supervisor before execution. A rejection is a first-class outcome the planner sees. | `irobots_core/safety.py`, `irobots_nodes/safety_node.py` |
| I3 | One skill per observation. The worker never chains skills in one turn. | `irobots_core/agent/worker.py`, `irobots_core/loop.py` |
| I4 | Every episode terminates: per-skill deadline, per-episode deadline, per-trial hard kill. | `irobots_nodes/control_node.py`, `irobots_eval/runner.py` |
| I5 | No metric mixes privileged (simulator ground truth) and perceived quantities. | `irobots_eval/metrics.py` |
| I6 | The simulator is stepped with a fixed `dt`. LLM wall-clock latency never changes how much physics time elapses, and the simulator is paused while the LLM thinks. | `irobots_core/sim/mujoco_sim.py`, `irobots_nodes/sim_node.py` |

Consequences worth stating:

- A new code path that moves the arm must go through the supervisor. If you find one that does not, that is a bug to fix, not a shortcut to copy.
- A new endpoint, button or script that can run more than one skill per observation violates I3.
- A new loop, retry or wait must have a deadline (I4). A dead camera or dead network must produce a typed outcome, never a hang.
- Never add a metric that averages privileged and perceived numbers, even "for convenience" (I5).
- There is no "attach object on contact" shortcut in the simulator. Ever.

## 4. Architecture rules

- **`irobots_core` is pure Python and must never import `rclpy` or any ROS package.** This keeps the science testable without a ROS graph and keeps ROS 2 deletable. A test greps for it, and CI runs that test.
- ROS 2 is the integration skin: topics, parameters, rosbag2, launch. Algorithms do not live in nodes. Nodes in `irobots_nodes` wrap `irobots_core`.
- **One source of truth per concern.** `config/robot.yaml` is the only place the arm's geometry, joint limits and servo mapping exist. The MuJoCo model and `firmware/esp32_arm/generated/robot_config.h` are generated from it. If you edit the YAML, regenerate the header and commit both.
- The web console is a **thin client**. Python is the sole authority for kinematics, safety, projection, scene sampling and hashing. The page draws what the server sends.
- Do not duplicate logic across languages. If the browser needs a number, add it to `/api/config` or `/api/state`.

## 5. Repository map and ownership

| Path | Contents | Primary specialist |
|---|---|---|
| `config/robot.yaml` | Arm geometry, limits, servo map, safety envelope | physics |
| `config/scene.yaml`, `config/tasks/` | Table, camera pose, objects, task definitions | physics, perception |
| `config/perception.yaml` | Detector, camera source, calibration, web console settings | perception |
| `ros2_ws/src/irobots_core/irobots_core/` | `kinematics.py`, `robot_model.py`, `safety.py`, `skills.py`, `trajectory.py`, `estop.py`, `loop.py`, `replay.py`, plus `agent/`, `llm/`, `perception/`, `sim/`, `arm/` | by subpackage |
| `ros2_ws/src/irobots_core/tests/` | The whole test suite | whoever owns the code under test |
| `ros2_ws/src/irobots_nodes/` | ROS 2 nodes and `web_node.py` (console server) | implementer |
| `ros2_ws/src/irobots_eval/` | Experiments RQ0 to RQ3 and metrics | implementer |
| `ros2_ws/src/irobots_msgs/`, `irobots_bringup/` | Messages and launch files | implementer |
| `firmware/esp32_arm/`, `firmware/esp32_cam/` | Arm and camera firmware | firmware |
| `web/irobots_control_console.html` | Operator console page | implementer |
| `spike/` | Phase 0 feasibility gates | physics, perception |
| `tools/` | Installers, firmware config generator, real-frame capture, results generator | implementer |
| `runs/` | Machine-written JSON from gates and experiments | nobody edits by hand |
| `docs/RESULTS.md` | Generated from `runs/` | nobody edits by hand |
| `legacy/` | Frozen version 1 | nobody |

## 6. Data and configuration rules

- Every physical constant carries a `provenance` tag: `measured` (read off the STL meshes), `design` (we chose it because the SolidWorks assembly has no recoverable joint frames), or `assumed` (a guess that must be replaced by measurement on the real arm).
- **No hardcoded IP, port, path, model id or API key in source.** They come from `config/*.yaml` or the environment. A test (T12) greps for this and it applies to `web_node.py` too.
- Every random draw goes through a seeded `numpy.random.Generator` threaded through the call chain. No global `random`, no bare `np.random.rand`.
- Results are produced by code into `runs/`. Nothing in `docs/RESULTS.md` is typed by hand.
- **Never change a value in `config/robot.yaml` or `config/scene.yaml` to make a gate pass.** A failing gate is a finding. Report it.
- Do not spend hosted LLM budget beyond what a named change requires.

Environment variables the code reads:

| Variable | Purpose |
|---|---|
| `IROBOTS_CAMERA_SOURCE` | `sim` (default) or `esp32_cam` |
| `BOARD_IP` | ESP32-CAM host; the variable name itself is set by `endpoint_env` in `config/perception.yaml` |
| `IROBOTS_CALIBRATION_FILE` | Marker calibration for `esp32_cam` |
| `IROBOTS_WEB_HOST`, `IROBOTS_WEB_PORT` | Console bind address |
| `IROBOTS_LLM_MODEL`, `IROBOTS_LLM_BASE_URL`, `OPENROUTER_API_KEY` | LLM client; the model defaults to `scripted-reference` |
| `IROBOTS_VENV` | Virtual environment used by `reproduce.sh` |

Never print, log or commit a key.

## 7. Camera source rules

The system reads frames through one `CameraFrameSource` in `irobots_core/perception/camera_source.py`.

- `sim` is the default and the only mode that needs no hardware.
- `esp32_cam` **requires a marker calibration**. Without one it must refuse with a clear error. Never fall back silently to the simulated camera's analytic calibration, because that is only valid if the real camera sits exactly at the specified pose.
- Real camera plus simulated arm is **hybrid mode**. It requires `allow_real_camera_with_sim_arm: true`. When active, every episode record, `metrics.json` and the console badge are labelled hybrid.
- Real frames are never graded with privileged simulator ground truth. Privileged success is reported as not applicable.
- A capture failure returns a typed outcome (`camera_unavailable`) that ends the episode within its deadline.
- Every episode record stores the camera source. For `esp32_cam` it also stores the host, never credentials.
- The camera frame must be right-side-up: camera +X to the right, +Y up, pixel row increasing downward. This is covered by `test_camera_frame_is_right_side_up`. If you touch `CameraModel` or `look_at_frame`, change both identically and rerun the perception gate.

## 8. Web console rules

Files: `ros2_ws/src/irobots_nodes/irobots_nodes/web_node.py` (server) and `web/irobots_control_console.html` (page). Tests: `tests/test_web_console.py`.

### Server

- Standard library only (`http.server`). Do not add a web framework.
- Bind to `127.0.0.1` by default. Binding elsewhere needs `allow_remote_bind: true` in config and logs a warning.
- Reject request bodies over `max_body_bytes` (64 KB) with `413`. Invalid JSON returns `400`. Unknown paths return `404`. Errors are JSON with no stack traces. CORS stays off.
- Never expose environment variables, credentials or the full config file.
- E-stop must work during a run, so runs execute off the request thread with a thread-safe e-stop controller.
- `/api/step` executes exactly one skill (I3). `/api/run` obeys the deadlines (I4). `/api/task` validates the plan against the scene so that naming an absent object is rejected.
- The `action_log_sha256` it returns must equal the hash `replay.py` computes from the same `episode.jsonl`.

### Page

The page must satisfy the test, which checks the HTML text:

- No `http://` or `https://` anywhere. That includes comments and SVG namespaces, so do not use inline SVG with `xmlns`.
- No `linear-gradient` or `radial-gradient`, including `repeating-` variants and comments.
- None of the strings `forwardKinematics`, `inverseKinematics`, `checkSafety`.
- The literal `IROBOTS` appears in the served page.
- These element IDs exist: `connectionBanner`, `modeBadge`, `chatStream`, `chatForm`, `simCanvas`, `camCanvas`, `camSimBtn`, `camEspBtn`, `trackerTableBody`, `jointsContainer`, `estopBtn`, `approvePlanBtn`, `stepSkillBtn`, `runLoopBtn`, `ndjsonLog`.

Design rules:

- Offline-first. No CDN scripts, styles or web fonts. System font stacks only.
- **Black, gray and white only.** No hue in the interface. Status is carried by shape, weight, fill and text, never by colour alone. The camera frame is the one place colour appears, because it is image data.
- Follows `prefers-color-scheme`. Both themes use the same neutral tokens.
- Write dynamic text with `textContent` or text nodes. Never interpolate server data into `innerHTML`.
- E-stop is always enabled and is the most visually dominant control. A latched e-stop is unmistakable and disables Step and Run.
- Disabled controls explain why in their tooltip.
- Show a connected or reconnecting state and retry automatically.
- Show the mode badge (simulation, hybrid, real camera) at all times.
- Keyboard focus is visible. Layout works down to a phone width.
- Do not reimplement kinematics, safety, projection, scene sampling or hashing in JavaScript.

## 9. Command reference

Set `PYTHONPATH` once per shell. The packages are not installed as a workspace, so imports fail without it.

```bash
export PYTHONPATH=$PWD/ros2_ws/src/irobots_core:$PWD/ros2_ws/src/irobots_nodes:$PWD/ros2_ws/src/irobots_msgs:$PWD/ros2_ws/src/irobots_eval:$PWD/ros2_ws/src/irobots_bringup
```

| Task | Command |
|---|---|
| Install everything | `tools/install_deps.sh` |
| Fetch model weights | `tools/fetch_weights.sh` |
| Full suite | `pytest ros2_ws/src/irobots_core/tests -q` |
| Console tests only | `pytest ros2_ws/src/irobots_core/tests/test_web_console.py -q` |
| Lint | `ruff check ros2_ws tools spike` |
| Types | `mypy --strict ros2_ws/src/irobots_core/irobots_core ros2_ws/src/irobots_nodes/irobots_nodes` |
| Firmware header check | `python tools/gen_firmware_config.py && git diff --exit-code firmware/esp32_arm/generated/robot_config.h` |
| Perception gate (C02) | `python spike/perception_bench.py --trials 50 --no-semantic` |
| Experiments | `python -m irobots_eval.exp_rq0` (also `exp_rq1`, `exp_rq2`, `exp_rq3`) |
| Regenerate results | `python tools/generate_results_md.py` |
| Everything | `./reproduce.sh` |
| Run the console | `python -m irobots_nodes.web_node [--host H] [--port N]` |
| Capture real frames | `python tools/capture_real_frames.py` (needs a camera) |

Use the versions of ruff and mypy pinned in `tools/requirements.lock.txt`. A newer ruff can report errors the pinned one does not.

## 10. Testing rules

- A test that cannot fail is not a test. Prove it: revert the fix and watch it fail.
- Write the failing test first, see it fail, then implement.
- Tests assert behaviour, not implementation details, and are deterministic.
- **Never weaken, skip, `xfail` or delete a test to get green.** If a test is wrong, say so in your report and fix it in a separate, explained commit.
- Run the tests you wrote and paste the **actual** output. Never report a pass you did not observe.
- CI covers logic and physics only. Rendering tests run locally or nightly.
- Optional dependencies (`mujoco`, `ultralytics`, `rclpy`) may be absent. Tests that need them skip cleanly, and the report says which results are fallback or not run.
- Web console tests start the real server on port 0 in a thread and use `urllib`. They need no browser and no network.

## 11. Verification discipline

These rules exist because earlier work reported results that did not hold up.

1. **Run before you report.** Counts of passed, failed and skipped tests, ruff errors and mypy errors come from output you just produced. Do not copy numbers from `PROGRESS.md`, from a previous session or from a commit message.
2. **Commit every source file you changed.** A fix that exists only in your working tree is lost when the work is exported as a patch. Before reporting, run `git status` and confirm nothing relevant is uncommitted. Run `git diff --stat` against the base and confirm the files you claim to have changed appear in it.
3. **A commit message is a claim.** If it says a camera orientation was fixed, the diff must touch the code that does so.
4. **State what you did not run.** Anything needing MuJoCo, YOLO-World, ROS 2, a browser, a real camera or the arm is marked "not run" unless you ran it.
5. **Label fallbacks.** Results from `DeterministicKinematicSim` rather than MuJoCo are labelled `fallback sim` in the JSON and in the report.
6. **Numbers in `docs/RESULTS.md` come only from `runs/`.** Regenerate it. Never edit it.
7. **Gates are findings.** If C01, C02, SC1 (at least 80 percent scripted success on L1), SC2 (at least 95 percent detection, p95 XY error at most 8 mm), SC3 or SC4 (zero hangs) fails, stop and report. Do not retune geometry, object size or model to rescue it.

## 12. Definition of done

A change is done only when every item holds.

- [ ] Failing tests were written first and then pass.
- [ ] The full suite passes, with output pasted.
- [ ] `ruff` and `mypy --strict` are clean on the files you touched, using the pinned versions.
- [ ] No invariant, provenance, hardcoded-value or ROS-import rule is violated.
- [ ] Every file changed is committed, with the change ID in the message.
- [ ] `PROGRESS.md` records status (`done`, `blocked` or `deviated`), test counts, `runs/` paths and deviations.
- [ ] Any gate the change touches was run, with its JSON written to `runs/`.
- [ ] Docs that describe the change were updated (`README.md`, `docs/USER_GUIDE.md`, `docs/ARCHITECTURE.md`).
- [ ] Claims are limited to what was verified. Hardware, e-stop safety, torque, servo behaviour and camera optics are not claimed.

## 13. Git rules

- Work on a branch. One change ID per commit where practical, and one phase per merge.
- Commit message format: `PLAN:irobots-v2:C## <summary>`. Fix-ups use `C##-F#`.
- Never commit secrets, model weights, `runs/` output you were not asked to commit, or `.venv/`.
- Never force-push, rewrite shared history or bypass hooks.
- Do not commit to `main` until the phase's tests pass. If your instructions say to use a branch, use it.
- Do not push unless the operator asks. `opencode.json` sets `git push*` to ask.
- `rm -rf *` is denied. `sudo`, `git reset --hard` and bare `pip install` ask first.

## 14. Install rules

Never run a bare `pip install` or `uv pip install` from an agent session. Use:

```bash
tools/install_deps.sh          # retries, resumable downloads, verification
tools/fetch_weights.sh         # resumable model-weight fetch
```

Both scripts retry with exponential backoff and resume partial downloads. Installing torch from PyPI pulls about 2.5 GB of CUDA wheels on Linux. This project is CPU-only, so the script installs torch from the CPU index first.

If a long command times out, break it into smaller steps, write intermediate output to a file, and add an explicit `timeout`.

## 15. Working as the only agent

When you are the only agent, the delegation framework below is a set of checklists, not a staffing plan. Do not wait for another agent that does not exist.

1. Read this file, `PLAN.md`, `docs/ARCHITECTURE.md`, `docs/RESULTS.md`, `PROGRESS.md`, and the code you will touch.
2. Establish a **verified baseline** first: run the suite, ruff and mypy, and record the real counts.
3. For each change ID, restate its WHAT, HOW and TEST from `PLAN.md`; write the tests and watch them fail; implement; run the suite; run any gate; commit; update `PROGRESS.md`.
4. Switch roles deliberately. After implementing, review your own change as an adversary: try to break it, run the tests yourself and check every claim against the diff.
5. Keep independent work moving when blocked. Record the blocker under "Needs operator" in `PROGRESS.md`.
6. Finish with a report of the changes done, blocked and deviated; each gate with its numbers and `runs/` path; real test and lint counts; deviations and why; and what the operator must do next.

## 16. Delegation to sub-agents

Work may be delegated under the protocol in [`docs/agent-ops/README.md`](docs/agent-ops/README.md). Briefs use [`docs/agent-ops/brief-template.md`](docs/agent-ops/brief-template.md). Definitions live in `.opencode/agent/`.

| Agent | Mode | Use it for | Not for |
|---|---|---|---|
| `irobots-explorer` | read-only | Answering questions about code, configs, git history and CAD or STL assets, citing `path:line` | Changing files, proposing designs |
| `irobots-implementer` | writes | Implementing a change against a written spec, with tests | Deciding what the design should be |
| `irobots-reviewer` | read-only | Trying to break a change; runs the tests itself | Writing the fix |
| `irobots-perception` | writes | Detector, calibration, coordinate transform, height, tracking, grasp candidates | Kinematics, control, the agent loop |
| `irobots-physics` | writes | Kinematics, trajectories, supervisor, MuJoCo model, determinism | Perception, agent loop, firmware protocol |
| `irobots-firmware` | writes | ESP32 sketch, wire protocol, emulator, conformance suite | The host-side control stack |

Rules:

- Every brief names the agent's **branch and working directory**. Parallel agents get their own git worktree.
- There is no orchestrator agent. Deciding what to delegate and in what order is the operator's judgement.
- The implementer writes the tests. The reviewer tries to break them. No separate test-writer agent exists.
- No agent writes the research report. `docs/RESULTS.md` is generated, and its interpretation belongs to the operator.
- If you are the only agent, `docs/agent-ops` conflicts with the single-branch workflow above. Use the branch the operator named and treat roles as checklists.

## 17. Traceability

- Code comments and commit messages carry the plan change ID: `PLAN:irobots-v2:C##`.
- Claim types in design docs are tagged `[FACT path:line]`, `[ASSUMPTION]`, `[OPINION]` or `[DECISION D#]`.
- Every skipped or blocked item is recorded in `PROGRESS.md` with a reason.

## 18. Stop and report

Stop work on the affected item, write the question under "Needs operator" in `PROGRESS.md`, and continue only with independent work, when:

- A gate fails.
- A value in `config/robot.yaml` or `config/scene.yaml` would have to change.
- An existing test would have to be weakened.
- Two documents conflict and this file does not resolve it.
- A key, hardware, MuJoCo, YOLO-World or ROS 2 is missing and blocks a claim.
- You find a fix already present uncommitted on disk. Say so, diff it against yours, and keep the version that passes both the test and the gate.
- Any request would break an invariant in [section 3](#3-the-six-invariants).

## 19. Honesty rules

- The arm is **not physically assembled**. Every grasp claim is a claim about the robot described in `config/robot.yaml`. The "not verified without the physical arm" list in `docs/RESULTS.md` is binding: do not imply hardware was tested. That list covers grasping on hardware, e-stop safety, torque, servo behaviour and camera optics.
- The arm firmware and its host-side emulator implement the NDJSON protocol and pass the conformance suite. That is a software claim. Nothing here has been flashed to and exercised on a real arm.
- The ESP32-CAM firmware works and may be flashed. Real-frame evaluation is pending operator capture until `data/real/` exists.
- If a measurement is optimistic (simulation renders have no exposure drift, JPEG artifacts or motion blur), say so and report the real-frame comparison when one exists.
- Do not fabricate frames, measurements or results.
- Ultralytics (YOLO-World) is AGPL-3.0. Disclose it wherever the project is described or distributed.

## 20. The legacy tree

`legacy/` holds the original hackathon code, frozen. Nothing imports it and it must not be "fixed". It records what was tried. The defects it documents are the ones this project exists to fix: no class labels on masks, `cm = px / 30` with the origin at the image corner, a guessed Z, and a `validate_and_execute_tool` that reported success without sending anything.

## 21. Known pitfalls

| Pitfall | What to do |
|---|---|
| Imports fail with `ModuleNotFoundError` | Export `PYTHONPATH` as in [section 9](#9-command-reference) |
| Results disagree with a teammate's | Check the seed, the camera source and whether MuJoCo or the fallback ran |
| Ruff or mypy report more errors than CI | Use the pinned versions; missing `mujoco` and `ultralytics` stubs need targeted overrides, not `# type: ignore` in core logic |
| Camera image looks mirrored or upside-down | Check the camera frame convention in [section 7](#7-camera-source-rules) |
| `esp32_cam` refuses to start | A marker calibration is required; see [section 7](#7-camera-source-rules) |
| Console test fails after a page edit | Look for an external URL, a gradient string, a forbidden function name, or a removed element ID |
| `reproduce.sh` fails at step 1 | The firmware header is stale; run `tools/gen_firmware_config.py` and commit it |
| A change "works" but a gate regressed | Rerun the gate and report the numbers; do not adjust geometry |
| Long command times out | Split it, write intermediate output to a file, add `timeout` |
