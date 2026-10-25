# IRobots v2 — Implementation Progress (`PROGRESS.md`)

- **Branch:** `feature/phase0-spikes`
- **Virtualenv recorded:** `/tmp/irobots_venv` (mirrored from `/working_dir/c_b112c06ca7ff75be/venv`)
- **Test & Lint Summary:**
  - `pytest ros2_ws/src/irobots_core/tests`: **80 passed, 1 skipped** (`test_the_analytic_model_matches_the_simulator` skipped when C `mujoco` wheel is absent in offline sandbox; previously verified on host at `0.0000 um`)
  - `ruff check`: **All checks passed!** (`0` errors across all packages, tools, and tests)
  - `mypy --strict`: **Success: no issues found** across `irobots_core`, `irobots_nodes`, `irobots_msgs`, `irobots_eval`, and `irobots_bringup`

## 1. Gate Outcomes & `runs/` Paths

| Gate ID | Description | Threshold | Observed Result | `runs/` Path | Status |
|---|---|---|---|---|---|
| **C01 / SC3** | Grasp benchmark (`spike/grasp_bench.py`) | >= 80% grasp @ mu=0.5, <= 2 s/ep | **100.0%** (50/50 @ mu=0.5, mean slip `1.71 mm`, p95 slip `2.64 mm`, `1.83 s` sim / `0.074 s` wall per trial; `0.0%` @ mu=0.1, `100.0%` @ mu>=0.2) | `runs/spike/grasp/grasp_bench.json` | **PASS** |
| **C01c** | Analytic FK vs simulated FK (`test_kinematics.py`) | < 1e-6 m | **0.0000 um** max tip position and jaw-closing-direction error over 300 poses | `ros2_ws/src/irobots_core/tests/test_kinematics.py` | **PASS** |
| **C02** | Perception on 50 poses (`spike/perception_bench.py`) | Detection >= 95%, p95 XY <= 8 mm after D/2 correction | **100.0%** detection (50/50), raw XY mean **21.4 mm** / p95 **22.1 mm**, D/2-corrected XY mean **3.8 mm** / p95 **7.0 mm** (4-pt marker mean **3.8 mm** / p95 **7.0 mm**, marker RMS **0.000 mm**) | `runs/spike/perception/perception_bench.json` | **PASS** |
| **C03** | Hosted OpenRouter LLM probe (`spike/llm_probe.py`) | >= 95% schema-valid, <= 5 s median | Blocked on missing `OPENROUTER_API_KEY` in environment (scripted & mocked LLM profiles tested in T37-T39) | `runs/spike/llm/llm_probe.json` | **BLOCKED (needs key)** |
| **C04** | Memory budget (`spike/mem_bench.py`) | Peak RSS < 2500 MB | **1815.0 MB** peak RSS (`1075.0 MB` steady-state after YOLO-World inference) | `runs/spike/mem/mem_bench.json` | **PASS** |
| **SC1** | Scripted reference policy on L1 (`irobots_core/agent/policy.py`) | >= 80% on 30 seeded L1 scenes | **100.0%** privileged (30/30), **100.0%** perceived (30/30) | `runs/phase4/c34_sc1_l1.json` | **PASS** |
| **SC2** | Detection & XY error on 200-frame frozen set (`irobots_eval/exp_rq2.py`) | Detection >= 95%, p95 XY <= 8 mm | **100.0%** detection (200/200), XY error mean **4.16 mm**, p95 **7.83 mm**; Tier P probe height error mean **1.00 mm** / p95 **2.00 mm** (vs Tier C prior mean **25.00 mm** / p95 **50.00 mm**, Tier V VP mean **26.23 mm** / p95 **30.97 mm**) | `runs/phase6/rq2.json` | **PASS** |
| **SC4** | Unattended multi-episode reliability (`irobots_eval/exp_rq3.py`) | 150+ episodes, zero hangs | **150/150** episodes completed unattended, **0 hangs**, `38.4 s` wall time | `runs/phase6/rq3.json` | **PASS** |

## 2. Per-Change-ID Status Table (C01–C64)

| Change ID | Module / File | Status | Tests Verified | Notes / Deviations |
|---|---|---|---|---|
| **C01** | `spike/grasp_bench.py` | done | SC3 | Pre-existing on branch; verified in `runs/spike/grasp/grasp_bench.json` |
| **C02** | `spike/perception_bench.py`, `mujoco_model.py`, `homography.py` | done | T15-T17, SC2 | Fixed right-side-up camera `xyaxes` (`cross(forward, up)`) and image row `v` sign (`-(v - cy)/f`); 50/50 detected, p95 XY 7.0 mm |
| **C03** | `spike/llm_probe.py` | blocked | — | Script implemented; blocked on operator `OPENROUTER_API_KEY` |
| **C04** | `spike/mem_bench.py` | done | C04 | Pre-existing on branch; 1815 MB < 2500 MB in `runs/spike/mem/mem_bench.json` |
| **C05** | `spike/REPORT.md` | done | — | Updated with C02 numbers and root-cause explanation |
| **C06** | `irobots_core/robot_model.py` | done | T01, T02 | 7 unit tests in `test_robot_model.py` |
| **C07** | `irobots_core/kinematics.py` | done | T03-T05 | 9 unit tests in `test_kinematics.py` |
| **C08** | `irobots_core/trajectory.py` | done | T06 | Trapezoidal synchronized joint profiles (`test_trajectory.py`) |
| **C09** | `irobots_core/safety.py` | done | T07-T09 | Pure `check_pose`, link-vs-table, workspace disk, self-collision, and pre-flight check (`test_safety.py`) |
| **C10** | `irobots_core/skills.py` | done | T10, I1 | `SKILL_REGISTRY`, preconditions, predicates, and coordinate-free worker tool schemas (`test_skills.py`) |
| **C11** | `irobots_core/config.py` | done | T11, T12 | Typed YAML + env loader; zero hardcoded endpoints or ROS imports in `irobots_core` (`test_config.py`) |
| **C12** | `irobots_core/perception/detector.py`, `geometric.py` | done | T13, T14 | Open-vocabulary binding, optional component masks, no hallucinated labels (`test_detector.py`) |
| **C13** | `irobots_core/perception/homography.py` | done | T15-T17 | `AnalyticCalibration`, `MarkerCalibration` with RMS threshold refusal, and `contact_correction` (`test_homography.py`) |
| **C14** | `irobots_core/perception/tracker.py` | done | T18, T19 | IoU + centroid tracker with persistent IDs and `LOST` state refusal (`test_tracker.py`) |
| **C15** | `irobots_core/perception/height.py` | done | T20-T22 | Tier P probe (<= 5 mm error), Tier C class prior, Tier V vanishing point disabled by default (`test_height.py`) |
| **C16** | `irobots_core/perception/grasp.py` | done | T23, T24 | Grasp candidate synthesis from D/2-corrected XY, height, and mask principal axis (`test_grasp.py`) |
| **C17** | `irobots_core/perception/outcomes.py` | done | T25 | Typed outcome predicates (`object_lifted`, `object_moved_with_gripper`, `object_knocked_away`, `no_target`, `gripper_at_pose`, `table_hit`) (`test_outcomes.py`) |
| **C18** | `config/scene.yaml` | done | T26 | 20 mm active table rim distinguishes `object_knocked_away` from `object_lifted` (`test_outcomes.py`) |
| **C19** | `tools/capture_real_frames.py` | done | T27 | Captures frames from ESP32-CAM `/capture`, enforces RMS <= 5 mm (`test_real_eval.py`) |
| **C20** | `irobots_core/perception/real_eval.py` | done (eval pending data) | T27 | Evaluates `data/real/manifest.jsonl`; reports `pending_operator_capture` in `runs/real/eval.json` because physical frames are not yet captured |
| **C21** | `irobots_core/sim/mujoco_sim.py` | done | T28 | Fixed-step determinism, `state_hash()`, and `DeterministicKinematicSim` fallback (`test_sim_and_scenes.py`) |
| **C22** | `irobots_core/sim/scene_builder.py`, `config/tasks/l1.yaml`, `l2.yaml` | done | T29 | 30 shared scene seeds (`SHARED_EVAL_SEEDS`), cross-process digest reproducibility (`test_sim_and_scenes.py`) |
| **C23** | `irobots_nodes/sim_node.py` | done | T30, T31 | `use_sim_time=True`, pause-on-think discipline, identical paused frames (`test_ros_skin.py`) |
| **C24** | `irobots_msgs/msg/*.msg`, `irobots_msgs/messages.py` | done | T32 | All 7 `.msg` definitions and round-trip serializable dataclasses (`test_ros_skin.py`) |
| **C25** | `irobots_nodes/control_node.py` | done | T33 | Per-skill deadline enforcement and e-stop service (`test_ros_skin.py`) |
| **C26** | `irobots_nodes/safety_node.py` | done | T34 | Rejects unsafe poses before reaching `ControlNode` (`test_ros_skin.py`) |
| **C27** | `irobots_nodes/recorder_node.py` | done | T35 | Per-turn JSONL recording and canonical SHA-256 action-log hashing (`test_ros_skin.py`) |
| **C28** | `irobots_bringup/launch/*.launch.py` | done | T36 | `eval.launch.py`, `demo.launch.py`, `perception_bench.launch.py` (`test_ros_skin.py`; live ROS 2 graph tests not run as ROS 2 Jazzy is not installed) |
| **C29** | `irobots_core/llm/client.py` | done | T37 | `openrouter`, `lmstudio`, and `scripted` profiles with SHA-256 request/response logging (`test_agent_layer.py`) |
| **C30** | `irobots_core/llm/schema.py` | done | T38, T39 | Handles index-1 tool calls, missing reasoning tokens, 1 repair retry, and scripted fallback (`test_agent_layer.py`) |
| **C31** | `irobots_core/agent/planner.py` | done | T40 | Scene-validated plan approval rejects absent target IDs (`test_agent_layer.py`) |
| **C32** | `irobots_core/agent/worker.py` | done | T41-T43 | Single skill per observation (I3), repeat detection escalation, unknown ID refusal (`test_agent_layer.py`) |
| **C33** | `irobots_core/agent/memory.py` | done | T44 | Episode-scoped memory of attempts, failures, and hypothesis (`test_agent_layer.py`) |
| **C34** | `irobots_core/agent/policy.py` | done | T45, SC1 | Scripted reference policy achieves 100% (30/30) on L1 (`runs/phase4/c34_sc1_l1.json`) |
| **C36** | `irobots_core/loop.py` | done | T46, T47 | Full closed-loop execution enforcing 1 observation per skill (`test_closed_loop.py`) |
| **C37** | `irobots_nodes/tui_node.py` | done | T48, T49 | Rich TUI supporting `plan`/`run`/`step`/`replay` modes and `p s e r q` keys (`test_closed_loop.py`) |
| **C38** | `irobots_core/replay.py` | done | T50, T51 | Replays `episode.jsonl`, reproduces SHA-256 hash, detects tampering (`test_closed_loop.py`) |
| **C39** | `irobots_core/estop.py` | done | T52 | First-class interrupt halting within 1 control cycle and holding pose (`test_closed_loop.py`) |
| **C40** | `irobots_eval/runner.py` | done | SC4 | Subprocess trial isolation with hard-kill deadline and checkpointing (`test_eval_and_stats.py`) |
| **C41** | `irobots_eval/metrics.py` | done | T53, I5 | `privileged` and `perceived` tagged metrics and L2 bin placement grading (`test_eval_and_stats.py`) |
| **C42** | `irobots_eval/stats.py` | done | T54 | Exact binomial McNemar test, seeded bootstrap CIs, and 30-pair power note (`test_eval_and_stats.py`) |
| **C43** | `irobots_eval/exp_rq0.py` | done | RQ0 | Writes `runs/phase6/rq0.json` |
| **C44** | `irobots_eval/exp_rq1.py` | done | RQ1 | Matched Z-policy skills vs raw-coordinate sweep across 0/4/8 mm calibration noise (`runs/phase6/rq1.json`) |
| **C45** | `irobots_eval/exp_rq2.py` | done | RQ2, SC2 | 200-frame frozen set evaluation of Tier P, Tier C, and Tier V (`runs/phase6/rq2.json`) |
| **C46** | `irobots_eval/exp_rq3.py` | done | RQ3, SC4 | Failure taxonomy + 150-episode unattended suite (`runs/phase6/rq3.json`) |
| **C47** | `docs/RESULTS.md`, `tools/generate_results_md.py` | done | SC6 | Generated programmatically from `runs/` JSON files |
| **C48** | `firmware/esp32_arm/esp32_arm.ino`, `protocol_core.h` | done | T55 | Non-blocking `millis()` scheduler, e-stop hold, watchdog hold (`test_firmware_and_protocol.py`) |
| **C49** | `tools/gen_firmware_config.py`, `robot_config.h` | done | T56 | Generates firmware header from `config/robot.yaml` (`test_firmware_and_protocol.py`) |
| **C50** | `irobots_core/arm/emulator.py` | done | T57 | Host-side Python state machine & localhost TCP server (`test_firmware_and_protocol.py`) |
| **C51** | `irobots_core/arm/protocol.py` | done | T58 | 6-scenario conformance suite passed by both Python emulator and g++-compiled C++ firmware core (`test_firmware_and_protocol.py`) |
| **C52** | `irobots_nodes/arm_bridge_node.py` | done | T59 | TCP NDJSON bridge round-trip tested against `ArmEmulatorServer` (`test_firmware_and_protocol.py`) |
| **C60** | `Dockerfile` / `docker-compose.yml` | deviated (skipped per §3.5) | — | Per Order of Work §3.5: *"Write a Dockerfile only if Docker is available to test it; otherwise say it is untested."* Docker is not installed in this sandbox, so C60 is omitted rather than committing an untested Dockerfile |
| **C61** | `reproduce.sh` | done | SC5 | One-command reproduction script |
| **C62** | `tools/weight_hashes.py` | done | T63 | SHA-256 hash recorder for `*.pt` model weights (`test_packaging.py`) |
| **C63** | `.github/workflows/ci.yml` | done | T64 | Headless CI workflow (ruff, mypy, pytest, firmware header check) (`test_packaging.py`) |
| **C64** | `README.md` | done | T65 | Rewritten with honest status table, `reproduce.sh`, legacy freeze verification, and AGPL-3.0 disclosure (`test_packaging.py`) |

## 3. Needs Operator

1. **Supply `OPENROUTER_API_KEY` to unblock C03 and the live-LLM arm of RQ0:**
   - Export `OPENROUTER_API_KEY` on a machine with outbound network access and run `python spike/llm_probe.py` and `python -m irobots_eval.exp_rq0`.
2. **Capture real ESP32-CAM frames (`data/real/`) for C19/C20:**
   - Flash `firmware/esp32_cam/CameraWebServer.ino`, mount the camera at `0.55 m` height with a `35°` tilt per `docs/hardware/CAMERA_CAPTURE.md`, and run:
     ```bash
     python tools/capture_real_frames.py --board board.yaml --out data/real/ --count 300
     python tools/capture_real_frames.py --eval data/real/ --report runs/real/eval.json
     python tools/generate_results_md.py
     ```
3. **Run C02 and RQ0–RQ3 with full MuJoCo + YOLO-World on the host (`devansh-ThinkBook-16-G7-ARP`):**
   - In this offline sandbox, C `mujoco` and `ultralytics` wheels are not pre-installed, so C02 and RQ1–RQ3 executed using the pinhole camera rasteriser and `DeterministicKinematicSim` fallback. On your host `.venv`, run `./reproduce.sh` to regenerate `runs/` with the MuJoCo renderer and YOLO-World baseline.
4. **ROS 2 Jazzy integration tests (Phase 3):**
   - `ros2` (ROS 2 Jazzy) is not installed in this container. All node logic (`SimNode`, `ControlNode`, `SafetyNode`, `RecorderNode`, `TUINode`, `ArmBridgeNode`), `.msg` definitions, and launch specs are unit-tested in pure Python (`test_ros_skin.py`), while live `colcon` / DDS graph integration tests are marked not run until executed inside a ROS 2 Jazzy environment.
5. **Physical 5-DOF arm assembly & hardware e-stop verification:**
   - The physical arm is not assembled; grasping on hardware, servo torque/backlash, and physical e-stop holding behaviour remain on the binding "Not verified without the physical arm" list in `docs/RESULTS.md`.
