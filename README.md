# IRobots v2 — Reproducible LLM-Manipulation Research Artifact

IRobots v2 (`PLAN:irobots-v2:C64`) is a hardware-free, reproducible LLM-manipulation research artifact. A 5-DOF robotic arm is given a manipulation task in natural language; an LLM planner and worker select atomic skills from a fixed library (`move_above`, `grasp`, `lift`, `release`, `retreat`, `home`); deterministic code resolves object coordinates from perception, checks every pose through a pure safety supervisor, executes trapezoidal joint trajectories, and verifies the physical outcome on the next camera frame.

Every headline result is reproducible on a CPU-only machine with **no physical arm, no camera, and no GPU**. Full quantitative results and statistical tables live in [`docs/RESULTS.md`](docs/RESULTS.md), and the architectural design contract lives in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).

## Quick Start (`reproduce.sh`)

```bash
tools/install_deps.sh      # CPU-only PyTorch + MuJoCo + YOLO-World + test tools
./reproduce.sh             # Run unit tests, Phase 0 gates, RQ0-RQ3 evals, and regenerate docs/RESULTS.md
```

## The Six Invariants

| ID | Invariant | Enforced in |
|---|---|---|
| **I1** | The LLM never emits a Cartesian coordinate; skills take tracked object IDs and the host derives poses. | `irobots_core/skills.py`, `irobots_core/llm/schema.py` |
| **I2** | Every commanded pose passes the safety supervisor before execution; rejection is a first-class outcome. | `irobots_core/safety.py`, `irobots_nodes/safety_node.py` |
| **I3** | Strictly one skill per observation. | `irobots_core/agent/worker.py`, `irobots_core/loop.py` |
| **I4** | Every episode terminates via per-skill deadlines, per-episode deadlines, and per-trial subprocess hard kill. | `irobots_nodes/control_node.py`, `irobots_eval/runner.py` |
| **I5** | No metric mixes privileged (simulator ground truth) and perceived quantities. | `irobots_eval/metrics.py`, `docs/RESULTS.md` |
| **I6** | Fixed `dt` simulation; the simulator is paused while the LLM and perception pipeline run. | `irobots_core/sim/mujoco_sim.py`, `irobots_nodes/sim_node.py` |

## Honest Status Table

| Component | Status | Verification |
|---|---|---|
| Pure-Python core (`irobots_core`) | **Complete** | 80 unit & integration tests passing; zero ROS imports (`T12`) |
| Phase 0 physics & perception gates (`spike/`) | **Complete** | C01: 100% grasp @ $\mu=0.5$; C01c: 0.0000 µm FK/sim agreement; C02: 100% det, 7.0 mm p95 XY; C04: 1815 MB peak RSS |
| ROS 2 thin wrapper nodes & messages (`irobots_nodes`, `irobots_msgs`, `irobots_bringup`) | **Complete** | Unit-tested in pure Python; live ROS 2 Jazzy graph integration not run when `ros2` is absent |
| Evaluation harness & RQ0–RQ3 (`irobots_eval`) | **Complete** | SC1: 100% scripted L1; SC2: 100% det / 7.83 mm p95 XY on 200 frames; SC4: 150/150 unattended episodes, 0 hangs |
| ESP32 arm firmware & emulator (`firmware/esp32_arm`, `irobots_core/arm`) | **Complete** | Both C++ firmware core and Python emulator pass the 6-scenario NDJSON protocol conformance suite (`T55–T59`) |
| Real ESP32-CAM frame evaluation (`tools/capture_real_frames.py`) | **Pending operator capture** | Tooling ready (`T27`); requires physical ESP32-CAM capture into `data/real/` per [`docs/hardware/CAMERA_CAPTURE.md`](docs/hardware/CAMERA_CAPTURE.md) |
| Physical 5-DOF arm hardware | **Not assembled** | Every grasp claim is scoped to the robot specified in `config/robot.yaml`; see the binding unverified list in [`docs/RESULTS.md`](docs/RESULTS.md) |

## Legacy Code (`legacy/`)

The original hackathon scripts and Arduino sketches are frozen in [`legacy/`](legacy/) as a historical record of the three defects this rebuild fixes (anonymous SAM masks without class labels, `cm = px / 30.0` at the image corner, and invented Z). Nothing in the active packages imports from `legacy/`.

## Licensing Disclosure (D24)

This project's code is released under the repository license (`LICENSE`), **except** for the optional open-vocabulary detector dependency Ultralytics (`ultralytics` / YOLO-World), which is licensed under **AGPL-3.0**. Any distribution or hosted deployment that bundles Ultralytics is subject to AGPL-3.0 terms.
