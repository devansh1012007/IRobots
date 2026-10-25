# IRobots v2 — Implementation Progress (`PROGRESS.md`)

- **Branch:** `review/patch` (reconstructed on top of base commit `d14f9a1` + 21 patch commits + `C67-F1..F7`)
- **Virtualenv:** `/tmp/irobots_venv` (override via `IROBOTS_VENV`, defaults to `./.venv` in `reproduce.sh`)

## 1. Verified Baseline (Before `C67-F1..F7`)

Executed against the 21-commit `review/patch` tree on top of `d14f9a1` prior to any `C67` changes:

- **`pytest ros2_ws/src/irobots_core/tests -q`**: **1 failed, 85 passed, 1 skipped** in `18.90s`
  ```text
  =================================== FAILURES ===================================
  ___ ros2_ws/src/irobots_core/tests/test_homography.py::test_camera_frame_is_right_side_up ___
      assert rot[0, 0] > 0.99, f"camera X axis is inverted: {rot[:, 0]}"
  AssertionError: camera X axis is inverted: [-1.  0.  0.]
  1 failed, 85 passed, 1 skipped in 18.90s
  ```
- **`ruff check ros2_ws tools spike`**: **14 errors**
  ```text
  ros2_ws/src/irobots_core/irobots_core/agent/policy.py:12:1: F401 `Path` imported but unused
  ros2_ws/src/irobots_core/irobots_core/perception/detector.py:13:1: I001 Import block is un-sorted or un-formatted
  ros2_ws/src/irobots_core/irobots_core/perception/geometric.py:27:1: I001 Import block is un-sorted or un-formatted
  ros2_ws/src/irobots_core/irobots_core/perception/geometric.py:217:1: F841 Local variable `centroids` is assigned to but never used
  ros2_ws/src/irobots_core/irobots_core/perception/geometric.py:231:1: F841 Local variable `cx_px` is assigned to but never used
  ros2_ws/src/irobots_core/irobots_core/perception/homography.py:31:1: UP035 `typing.Sequence` is deprecated, use `collections.abc.Sequence` instead
  ros2_ws/src/irobots_core/irobots_core/perception/homography.py:169:1: UP037 Remove quotes from type annotation
  ros2_ws/src/irobots_core/tests/test_real_eval.py:14:1: F401 `json` imported but unused
  ros2_ws/src/irobots_core/tests/test_real_eval.py:17:1: F401 `cv2` imported but unused
  spike/grasp_bench.py:42:1: E402 Module level import not at top of file
  spike/grasp_bench.py:43:1: E402 Module level import not at top of file
  spike/grasp_bench.py:44:1: E402 Module level import not at top of file
  spike/grasp_bench.py:45:1: E402 Module level import not at top of file
  spike/grasp_bench.py:46:1: E402 Module level import not at top of file
  Found 14 error(s).
  ```
- **`mypy --strict ros2_ws/src/irobots_core/irobots_core ros2_ws/src/irobots_nodes/irobots_nodes`**: **9 errors in 43 source files**
  ```text
  ros2_ws/src/irobots_core/irobots_core/perception/camera_source.py:27: error: Cannot find implementation or library stub for module named "cv2"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/perception/detector.py:88: error: Cannot find implementation or library stub for module named "ultralytics"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/perception/geometric.py:121: error: Cannot find implementation or library stub for module named "cv2"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/perception/homography.py:103: error: Returning Any from function declared to return "np.ndarray"  [no-any-return]
  ros2_ws/src/irobots_core/irobots_core/perception/real_eval.py:18: error: Cannot find implementation or library stub for module named "cv2"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/sim/mujoco_sim.py:71: error: Cannot find implementation or library stub for module named "mujoco"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/sim/mujoco_sim.py:496: error: Cannot find implementation or library stub for module named "cv2"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/sim/mujoco_sim.py:277: error: Cannot find implementation or library stub for module named "mujoco"  [import-not-found]
  ros2_ws/src/irobots_core/irobots_core/sim/mujoco_sim.py:283: error: Returning Any from function declared to return "np.ndarray"  [no-any-return]
  Found 9 error(s) in 43 source files
  ```

## 2. Gate Outcomes & `runs/` Paths

| Gate ID | Description | Threshold | Observed Result | Engine / Provenance | `runs/` Path | Status |
|---|---|---|---|---|---|---|
| **C01 / SC3** | Grasp benchmark (`spike/grasp_bench.py`) | >= 80% grasp @ mu=0.5, <= 2 s/ep | **100.0%** (50/50 @ mu=0.5, mean slip `1.71 mm`, p95 slip `2.64 mm`, `1.83 s` sim / `0.074 s` wall per trial; `0.0%` @ mu=0.1, `100.0%` @ mu>=0.2) | MuJoCo (pre-existing on `d14f9a1`) | `runs/spike/grasp/grasp_bench.json` | **PASS** |
| **C01c** | Analytic FK vs simulated FK (`test_kinematics.py`) | < 1e-6 m | `not run` in offline container (`mujoco` C wheel absent; skipped by `pytest.importorskip("mujoco")`; recorded on `d14f9a1` host at `0.0000 um` in `PLAN.md`) | `not run` in sandbox | `ros2_ws/src/irobots_core/tests/test_kinematics.py` | **SKIPPED (no MuJoCo wheel)** |
| **C02** | Perception on 50 poses (`spike/perception_bench.py`) | Detection >= 95%, p95 XY <= 8 mm after D/2 correction | **100.0%** detection (50/50), raw XY mean **21.4 mm** / p95 **22.1 mm**, D/2-corrected XY mean **3.8 mm** / p95 **7.0 mm** (4-pt marker p95 **7.0 mm**, RMS **0.000 mm**); YOLO-World semantic baseline `not run` (`ultralytics` absent) | `fallback sim` (`pinhole_rasterizer` + `GeometricDetector`) | `runs/spike/perception/perception_bench.json` | **PASS (fallback sim)** |
| **C03** | Hosted OpenRouter LLM probe (`spike/llm_probe.py`) | >= 95% schema-valid, <= 5 s median | `not run` — blocked on missing `OPENROUTER_API_KEY` and disabled sandbox network | `not run` | `runs/spike/llm/llm_probe.json` | **BLOCKED (needs key)** |
| **C04** | Memory budget (`spike/mem_bench.py`) | Peak RSS < 2500 MB | **1815.0 MB** peak RSS (`1075.0 MB` steady-state) | Host measurement from `d14f9a1` | `runs/spike/mem/mem_bench.json` | **PASS** |
| **SC1** | Scripted reference policy on L1 (`irobots_core/agent/policy.py`) | >= 80% on 30 seeded L1 scenes | **100.0%** privileged (30/30), **100.0%** perceived (30/30) | `fallback sim` (`DeterministicKinematicSim`) | `runs/phase4/c34_sc1_l1.json` | **PASS (fallback sim)** |
| **SC2** | Detection & XY error on 200-frame frozen set (`irobots_eval/exp_rq2.py`) | Detection >= 95%, p95 XY <= 8 mm | **100.0%** detection (200/200), XY error mean **4.16 mm**, p95 **7.83 mm**; Tier P probe `|H_hat - H|` mean **1.00 mm** / p95 **2.00 mm** | `fallback sim` (`DeterministicKinematicSim`) | `runs/phase6/rq2.json` | **PASS (fallback sim)** |
| **SC4** | Unattended multi-episode reliability (`irobots_eval/exp_rq3.py`) | 150+ episodes, zero hangs | **150/150** episodes completed unattended, **0 hangs** | `fallback sim` (`DeterministicKinematicSim`) | `runs/phase6/rq3.json` | **PASS (fallback sim)** |

## 3. Per-Change-ID Status Table (C01–C66 & C67-F1..F7)

| Change ID | Module / File | Status | Tests Verified | Notes / Deviations |
|---|---|---|---|---|
| **C01** | `spike/grasp_bench.py` | done | SC3 | Pre-existing in `runs/spike/grasp/grasp_bench.json` |
| **C02 / C67-F1** | `homography.py`, `mujoco_model.py`, `spike/perception_bench.py` | done | T15-T17, C67-F1 | Fixed `x_axis = np.cross(forward, up)` and `-(v - cy)/f` in both `CameraModel` and `look_at_frame`; added `test_world_plus_x_projects_to_larger_u_and_farther_y_to_smaller_v`; 50/50 detected, p95 XY 7.0 mm (`fallback sim`) |
| **C03** | `spike/llm_probe.py` | not run (blocked) | — | Blocked on `OPENROUTER_API_KEY` |
| **C04** | `spike/mem_bench.py` | done | C04 | Pre-existing in `runs/spike/mem/mem_bench.json` (1815 MB < 2500 MB) |
| **C05** | `spike/REPORT.md` | done | — | Updated with C02 numbers and root-cause explanation |
| **C06** | `irobots_core/robot_model.py` | done | T01, T02 | `test_robot_model.py` (7 passed) |
| **C07** | `irobots_core/kinematics.py` | done | T03-T05 | `test_kinematics.py` (8 passed, 1 skipped without `mujoco`) |
| **C08** | `irobots_core/trajectory.py` | done | T06 | `test_trajectory.py` (3 passed) |
| **C09** | `irobots_core/safety.py` | done | T07-T09 | `test_safety.py` (6 passed) |
| **C10** | `irobots_core/skills.py` | done | T10, I1 | `test_skills.py` (4 passed) |
| **C11** | `irobots_core/config.py` | done | T11, T12 | `test_config.py` (7 passed) |
| **C12** | `irobots_core/perception/detector.py`, `geometric.py` | done | T13, T14 | `test_detector.py` (3 passed; `WorldDetector` YOLO-World path `not run` without `ultralytics`) |
| **C13** | `irobots_core/perception/homography.py` | done | T15-T17 | `test_homography.py` (6 passed including C67-F1 regression test) |
| **C14** | `irobots_core/perception/tracker.py` | done | T18, T19 | `test_tracker.py` (3 passed) |
| **C15** | `irobots_core/perception/height.py` | done | T20-T22 | `test_height.py` (3 passed) |
| **C16** | `irobots_core/perception/grasp.py` | done | T23, T24 | `test_grasp.py` (2 passed) |
| **C17–C18** | `irobots_core/perception/outcomes.py`, `config/scene.yaml` | done | T25, T26 | `test_outcomes.py` (2 passed) |
| **C19–C20** | `tools/capture_real_frames.py`, `real_eval.py` | done (real capture not run) | T27 | Unit tests pass (`test_real_eval.py`); physical frame capture `not run` (`pending_operator_capture` in `runs/real/eval.json`) |
| **C21–C22** | `mujoco_sim.py`, `scene_builder.py`, `config/tasks/*.yaml` | done | T28, T29 | `test_sim_and_scenes.py` (2 passed using `DeterministicKinematicSim` fallback) |
| **C23–C28** | `irobots_nodes/*`, `irobots_msgs/*`, `irobots_bringup/*` | done (ROS 2 graph not run) | T30-T36 | Pure-Python node/message/launch tests pass (`test_ros_skin.py`, 6 passed); live `rclpy` / DDS graph tests `not run` (ROS 2 Jazzy not installed) |
| **C29–C34** | `irobots_core/llm/*`, `irobots_core/agent/*` | done (live LLM not run) | T37-T45, SC1 | `test_agent_layer.py` (8 passed); live OpenRouter/LM Studio calls `not run` |
| **C36–C39** | `loop.py`, `tui_node.py`, `replay.py`, `estop.py` | done | T46-T52 | `test_closed_loop.py` (4 passed) |
| **C40–C47** | `irobots_eval/*`, `docs/RESULTS.md` | done | T53, T54, SC2, SC4, SC6 | `test_eval_and_stats.py` (5 passed); `runs/phase6/*.json` generated via `fallback sim` |
| **C48–C52** | `firmware/esp32_arm/*`, `emulator.py`, `protocol.py`, `arm_bridge_node.py` | done | T55-T59 | `test_firmware_and_protocol.py` (3 passed, including C++ `g++` conformance compilation); physical MCU flash `not run` |
| **C60** | `Dockerfile` / `docker-compose.yml` | not run (skipped) | — | Skipped per rule: Docker is not available in this container to test a Dockerfile |
| **C61–C64** | `reproduce.sh`, `weight_hashes.py`, `ci.yml`, `README.md` | done | T63-T65 | `test_packaging.py` (4 passed) |
| **C65** | `irobots_core/perception/camera_source.py`, `config/perception.yaml` | done | T66-T70 | `SimCameraSource` vs `ESP32CameraSource` switching (`test_camera_source.py`) |
| **C66** | `web/irobots_control_console.html`, `irobots_nodes/web_node.py` | done | `test_web_console.py` | Technical deep-black/grey operator Web Control Console and backend |

## 4. Known Limitations (`F3`–`F7` Backlog & Hardware Boundaries)

- **F3 (Camera source semantics & hybrid safety):** `esp32_cam` currently reuses `AnalyticCalibration` instead of requiring a validated `MarkerCalibration` (`perception.camera_source.calibration_file` with RMS <= `max_reprojection_rms_m`), lacks the `allow_real_camera_with_sim_arm` hybrid guard/labelling (`"hybrid: real camera + simulated arm"`), raises a raw exception on camera timeout instead of returning a typed `camera_unavailable` outcome within episode deadlines (I4), and does not record `camera_source` + capture host in every `episode.jsonl` turn record. *(Scheduled next in `C67-F3`.)*
- **F4 (Web console single source of truth):** `web/irobots_control_console.html` currently duplicates FK, IK, safety, camera projection, scene sampling, and hashing in client-side JS, includes simulated ESP32 pixel jitter, loads external Tailwind/Google Fonts CDNs, uses `innerHTML` for dynamic strings, and is not yet driven by a live `WebConsoleServer` HTTP API (`GET /api/state`, `GET /api/frame.jpg`, `GET /api/config`, `POST /api/camera_source`, `POST /api/task`, `POST /api/approve`, `POST /api/step`, `POST /api/run`, `POST /api/estop`, `POST /api/reset`). *(Scheduled in `C67-F4`.)*
- **F5 (Lint and type debt):** `ruff check ros2_ws tools spike` and `mypy --strict` have remaining import-sorting, unused-variable, `no-any-return`, `# type: ignore`, and missing third-party mypy override items across `ros2_ws`, `tools`, and `spike`. *(Scheduled in `C67-F5`.)*
- **F6 (Fallback simulator labelling in `runs/` and `docs/RESULTS.md`):** Runs produced by `DeterministicKinematicSim` / pinhole rasteriser when `mujoco` is absent must be explicitly labelled `"fallback sim"` in both `runs/*.json` and `docs/RESULTS.md`, while real-frame evaluation remains `"pending_operator_capture"`. *(Scheduled in `C67-F6`.)*
- **F7 (Packaging & `reproduce.sh` portability):** `reproduce.sh` currently hardcodes `/tmp/irobots_venv` instead of honouring `${IROBOTS_VENV:-./.venv}`, and headless CI needs the web console smoke test. *(Scheduled in `C67-F7`.)*

## 5. Needs Operator

1. **Uncommitted F1 fix found on disk in pre-`git init` snapshot (Section 5 disclosure):**
   - When the repository was first initialized in the sandbox (`/tmp/irobots_repo_pre_c67_backup`), `homography.py` and `mujoco_model.py` had already been edited before the initial base commit (`473a524`) was created. Consequently, `git diff 473a524..HEAD` did not include those two files in `irobots-v2-all-changes.patch` / `irobots-v2-commits.patch`. We diffed `/tmp/orig_d14f9a1` (`d14f9a1`) against that pre-init copy and against our `C67-F1` commit (`cac9f6d`): the mathematical fix (`x_axis = np.cross(forward, up)`, `-(v - cy) / f`, `cy - f * cam[1] / -cam[2]`) is identical and is now properly committed on top of `d14f9a1` in `PLAN:irobots-v2:C67-F1`.
2. **`mujoco`, `ultralytics`, and ROS 2 Jazzy unavailable in this container:**
   - All simulation and perception benchmarks executed here (`C02`, `SC1`, `SC2`, `SC4`, `RQ0`–`RQ3`) used the `DeterministicKinematicSim` and pinhole rasteriser (`fallback sim`), `test_the_analytic_model_matches_the_simulator` is skipped, YOLO-World (`WorldDetector`) is `not run`, and live ROS 2 DDS graph launch tests are `not run`. Install `mujoco` and `ultralytics` on the host and rerun `./reproduce.sh`.
3. **Supply `OPENROUTER_API_KEY`:**
   - Required to run `spike/llm_probe.py` (C03) and the live-LLM condition in `irobots_eval/exp_rq0.py`.
4. **Capture real ESP32-CAM frames (`data/real/`) with measured ChArUco/marker calibration:**
   - Required to replace `pending_operator_capture` in `runs/real/eval.json` and to run `esp32_cam` mode with a validated `MarkerCalibration` file.
