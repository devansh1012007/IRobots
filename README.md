# IRobots v2

**A reproducible research artifact for LLM-driven robot manipulation, built to run without hardware.**

A 5-DOF desk arm receives a task in plain language. An LLM planner and worker choose from a small fixed library of skills. Deterministic code turns tracked object IDs into poses, checks every pose with a safety supervisor, executes the motion, and verifies the result on the next camera frame.

Every headline result can be reproduced on a CPU-only machine with no arm, no camera and no GPU. Results and statistics are in [`docs/RESULTS.md`](docs/RESULTS.md). The design contract is in [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md). To operate the system from a browser, see the [User Guide](docs/USER_GUIDE.md).

---

## Contents

1. [Why this project exists](#why-this-project-exists)
2. [How it works](#how-it-works)
3. [Quick start](#quick-start)
4. [Operator web console](#operator-web-console)
5. [Camera sources](#camera-sources)
6. [The six invariants](#the-six-invariants)
7. [Repository layout](#repository-layout)
8. [Configuration and environment](#configuration-and-environment)
9. [Testing and quality gates](#testing-and-quality-gates)
10. [Status and limits](#status-and-limits)
11. [Legacy code](#legacy-code)
12. [Licensing](#licensing)

---

## Why this project exists

The first version of this project (see `legacy/`) had three defects that made its claims unreliable: anonymous segmentation masks with no class labels, a pixel-to-centimetre conversion that was only correct at the image corner, and an invented Z height. IRobots v2 rebuilds the stack so that each claim is testable:

- The LLM selects *what to do* and never emits a coordinate.
- Geometry comes from calibrated perception, not from the model.
- Every constant carries a provenance tag: `measured`, `design` or `assumed`.
- Privileged (simulator) and perceived (camera) metrics are never mixed.

## How it works

```
 natural-language task
          |
          v
 +-------------------+      skill calls with tracked object IDs
 |  LLM planner and  | ----------------------------------------+
 |  worker (I1, I3)  |                                         |
 +-------------------+                                         v
          ^                                          +-------------------+
          | next observation                         |  Host resolves    |
          |                                          |  poses from       |
 +-------------------+     detections + tracks       |  tracked objects  |
 |  Perception       | <---------------------+       +---------+---------+
 |  YOLO-World,      |                       |                 |
 |  homography,      |              +--------+--------+        v
 |  tracker          |              |  Camera source  |  +-------------+
 +-------------------+              |  sim | esp32    |  |  Safety     |
                                    +-----------------+  |  supervisor |
                                                         |  (I2)       |
                                                         +------+------+
                                                                |
                                                                v
                                            trapezoidal trajectory, fixed dt (I6)
                                                                |
                                    +---------------------------+------------+
                                    |                                        |
                              MuJoCo simulation                 ESP32 arm (NDJSON protocol)
```

The skill library is `move_above`, `grasp`, `lift`, `release`, `retreat` and `home`. Skills take tracked object IDs. The host derives the pose, so a wrong guess from the model cannot become an unsafe coordinate.

## Quick start

Requirements: Linux or macOS, Python 3.12, no GPU.

```bash
git clone https://github.com/devansh1012007/IRobots-Devansh_Gaur.git
cd IRobots-Devansh_Gaur

tools/install_deps.sh     # CPU-only PyTorch, MuJoCo, YOLO-World and test tools
./reproduce.sh            # tests, Phase 0 gates, RQ0-RQ3 evaluations, regenerates docs/RESULTS.md
```

`reproduce.sh` uses the virtual environment named by `IROBOTS_VENV`, then `./.venv`, then your system Python. It runs five stages:

| Stage | What it does |
|---|---|
| 1 | Checks that the generated firmware header matches `config/robot.yaml` |
| 2 | Runs the unit, integration and headless web console tests |
| 3 | Runs the Phase 0 perception gate (C02) |
| 4 | Runs the RQ0 to RQ3 evaluations and the SC1, SC2 and SC4 gates |
| 5 | Regenerates `docs/RESULTS.md` from the JSON files in `runs/` |

Numbers in `docs/RESULTS.md` come only from `runs/` JSON. Do not edit that file by hand.

## Operator web console

The console is a single offline HTML page served by a standard-library Python server. It is a thin client: the server computes everything, and the page only draws what it receives.

```bash
export PYTHONPATH=$PWD/ros2_ws/src/irobots_core:$PWD/ros2_ws/src/irobots_nodes:$PWD/ros2_ws/src/irobots_msgs:$PWD/ros2_ws/src/irobots_eval:$PWD/ros2_ws/src/irobots_bringup
python -m irobots_nodes.web_node            # http://127.0.0.1:8080
python -m irobots_nodes.web_node --port 9000
```

The interface uses only black, gray and white, follows your operating system's light or dark setting, loads no external fonts, scripts or styles, and works with the network disabled. Status is carried by shape, weight and text rather than colour.

| Area | Purpose |
|---|---|
| Header | Connection state, mode (simulation, hybrid or real camera), safety verdict, camera source, simulation clock, e-stop |
| Left rail | Scene reset, instruction entry, plan review, approve, step and run |
| Stage | Arm drawing (iso, side, top), camera frame, tracked objects, conversation, action log and arm protocol logs |
| Right rail | Joint positions against their limits, safety verdict, failure mode, privileged and perceived outcomes, predicates |

Full instructions are in the [User Guide](docs/USER_GUIDE.md).

### HTTP API

| Method and path | Purpose |
|---|---|
| `GET /` | The console page |
| `GET /api/config` | Robot, scene and camera specification read from `config/*.yaml` |
| `GET /api/state` | Pose, tool tip, tracks, plan, safety verdict, e-stop, camera source, episode summary, replay hash |
| `GET /api/frame.jpg` | Latest camera frame |
| `POST /api/reset` | New seeded episode: `{"seed": 0, "level": "L1"}` |
| `POST /api/task` | `{"text": "..."}`; returns a plan for approval, validated against the scene |
| `POST /api/approve` | Approve the current plan |
| `POST /api/step` | Execute exactly one skill |
| `POST /api/run` | Run the approved plan to completion under per-skill and per-episode deadlines |
| `POST /api/estop` | `{"estop": true}` halts; `{"estop": false}` clears |
| `POST /api/camera_source` | `{"source": "sim"}` or `{"source": "esp32_cam", ...}` |

The server binds to `127.0.0.1` by default, rejects request bodies over 64 KB, returns JSON errors rather than stack traces, and keeps CORS off. Binding to another address requires `allow_remote_bind: true` in `config/perception.yaml`.

## Camera sources

Perception and the closed loop read frames through one `CameraFrameSource` (`irobots_core/perception/camera_source.py`), selected in `config/perception.yaml`:

```yaml
perception:
  camera_source:
    source: sim                   # "sim" or "esp32_cam"
    endpoint_env: BOARD_IP
    capture_path: /capture
    timeout_s: 5.0
    retries: 3
    calibration_file: ""          # required for esp32_cam
    allow_real_camera_with_sim_arm: false
```

```bash
export IROBOTS_CAMERA_SOURCE=sim                 # default, hardware-free

export IROBOTS_CAMERA_SOURCE=esp32_cam           # physical ESP32-CAM over Wi-Fi
export BOARD_IP=192.168.1.50
export IROBOTS_CALIBRATION_FILE=path/to/markers.yaml
```

Safety rules for the real camera:

- `esp32_cam` refuses to start without a marker calibration. It never falls back silently to the simulated camera's geometry.
- Running a real camera with the simulated arm is a **hybrid** mode. It needs `allow_real_camera_with_sim_arm: true`, and every episode, metrics file and the console badge are labelled hybrid.
- Real frames are never graded against simulator ground truth, so privileged success reads as not applicable.
- A dead camera produces a `camera_unavailable` outcome and the episode still ends within its deadline.

## The six invariants

| ID | Invariant | Enforced in |
|---|---|---|
| I1 | The LLM never emits a coordinate. Skills take tracked object IDs and the host derives poses. | `irobots_core/skills.py`, `irobots_core/llm/schema.py` |
| I2 | Every commanded pose passes the safety supervisor. Rejection is a first-class outcome. | `irobots_core/safety.py`, `irobots_nodes/safety_node.py` |
| I3 | Strictly one skill per observation. | `irobots_core/agent/worker.py`, `irobots_core/loop.py` |
| I4 | Every episode terminates through per-skill and per-episode deadlines and a per-trial hard kill. | `irobots_nodes/control_node.py`, `irobots_eval/runner.py` |
| I5 | No metric mixes privileged (simulator) and perceived quantities. | `irobots_eval/metrics.py`, `docs/RESULTS.md` |
| I6 | Fixed `dt`. The simulator is paused while the LLM and perception run. | `irobots_core/sim/mujoco_sim.py`, `irobots_nodes/sim_node.py` |

## Repository layout

```
config/                     robot.yaml (single source of truth), scene.yaml, perception.yaml, tasks/
docs/                       ARCHITECTURE.md, RESULTS.md, USER_GUIDE.md, hardware/, agent-ops/
firmware/esp32_arm/         arm firmware and generated robot_config.h
firmware/esp32_cam/         camera firmware
ros2_ws/src/
  irobots_core/             pure Python core: kinematics, safety, skills, perception, sim, agent, loop
  irobots_nodes/            thin ROS 2 nodes and the web console server
  irobots_msgs/             message definitions
  irobots_eval/             evaluation harness and RQ0-RQ3 experiments
  irobots_bringup/          launch files
spike/                      Phase 0 feasibility gates
tools/                      install, firmware config generator, real-frame capture, results generator
web/                        irobots_control_console.html
runs/                       JSON output of every gate and experiment
legacy/                     frozen version 1 code
```

`irobots_core` never imports `rclpy` or any ROS package. A test enforces this, so the core runs and tests without ROS installed.

## Configuration and environment

| Variable | Purpose | Default |
|---|---|---|
| `IROBOTS_CAMERA_SOURCE` | `sim` or `esp32_cam` | `sim` |
| `BOARD_IP` | ESP32-CAM host (name set by `endpoint_env`) | none |
| `IROBOTS_CALIBRATION_FILE` | Marker calibration for `esp32_cam` | none |
| `IROBOTS_WEB_HOST`, `IROBOTS_WEB_PORT` | Console bind address | `127.0.0.1`, `8080` |
| `IROBOTS_LLM_MODEL` | Model identifier | `scripted-reference` |
| `IROBOTS_LLM_BASE_URL` | Hosted model endpoint | empty |
| `OPENROUTER_API_KEY` | Hosted model key | empty |
| `IROBOTS_VENV` | Virtual environment used by `reproduce.sh` | `./.venv` |

No IP address, port, path or model identifier is hardcoded in source. `config/robot.yaml` is the single source of truth for joint limits and geometry. Do not change it to make a gate pass; a failing gate is a finding to report.

## Testing and quality gates

```bash
pytest ros2_ws/src/irobots_core/tests -q
ruff check ros2_ws tools spike
mypy --strict ros2_ws/src/irobots_core/irobots_core ros2_ws/src/irobots_nodes/irobots_nodes
pytest ros2_ws/src/irobots_core/tests/test_web_console.py -q      # console smoke test, no browser
```

Continuous integration (`.github/workflows/`) runs the firmware header check, ruff, mypy, the console smoke test and the full suite on every push and pull request.

## Status and limits

| Component | Status |
|---|---|
| Pure Python core (`irobots_core`) | Complete, with unit and integration tests |
| Phase 0 physics and perception gates (`spike/`) | Complete; figures are in `docs/RESULTS.md` |
| ROS 2 nodes, messages and launch files | Written and unit-tested in pure Python. A live ROS 2 graph has not been run where `ros2` is absent |
| Evaluation harness and RQ0 to RQ3 | Complete |
| ESP32 arm firmware and emulator | Complete; both pass the NDJSON protocol conformance suite |
| Web console and camera switching | Complete; the console redesign has not yet been checked in a browser by the author of this document |
| Real ESP32-CAM frame evaluation | **Pending operator capture** (see `docs/hardware/CAMERA_CAPTURE.md`) |
| Physical arm | **Not assembled** |

Every grasp claim is a claim about the robot described in `config/robot.yaml`, not about hardware. Runs that used the deterministic kinematic fallback instead of MuJoCo are labelled as fallback in their JSON and in the report. The list of things that cannot be verified without the physical arm (hardware grasping, e-stop safety, torque, servo behaviour, camera optics) is in `docs/RESULTS.md`.

## Legacy code

The original hackathon scripts and Arduino sketches are frozen in [`legacy/`](legacy/) as a record of the three defects described above. Nothing in the active packages imports from it.

## Licensing

Project code is released under the repository license in [`LICENSE`](LICENSE). The optional open-vocabulary detector dependency Ultralytics (YOLO-World) is licensed under **AGPL-3.0**. Any distribution or hosted deployment that bundles it is subject to those terms.
