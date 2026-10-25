# IRobots v2 — Implementation Progress (`PROGRESS.md`)

- **Branch:** `review/patch` (reconstructed on top of base commit `d14f9a1` + 21 patch commits + `C67-F1..F7`)
- **Virtualenv:** `${IROBOTS_VENV:-./.venv}` (tested with `IROBOTS_VENV=/tmp/irobots_venv` in sandbox)

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

## 2. Post-`C67-F1..F7` Verification Summary

- **`pytest ros2_ws/src/irobots_core/tests -q`**: **92 passed, 0 failed, 1 skipped** (`test_the_analytic_model_matches_the_simulator` skipped because the C `mujoco` wheel is absent in this container)
- **`ruff check ros2_ws tools spike`**: **All checks passed!** (`0` errors)
- **`mypy --strict ros2_ws/src/irobots_core/irobots_core ros2_ws/src/irobots_nodes/irobots_nodes`**: **Success: no issues found in 43 source files** (`0` errors, `0` `# type: ignore` comments in `irobots_core` and `irobots_nodes`)

## 3. Gate Outcomes & `runs/` Paths

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

## 4. Per-Fix Report (`C67-F1` through `C67-F7`)

| Fix ID | Commit Tag | Status | Files Changed | Summary & Verification |
|---|---|---|---|---|
| **F1** | `PLAN:irobots-v2:C67-F1` | done | `homography.py`, `mujoco_model.py`, `test_homography.py`, `runs/spike/perception/perception_bench.json` | Committed right-side-up camera basis (`x_axis = np.cross(forward, up)`, `y_axis = np.cross(z_axis, x_axis)`) and downward row `v` (`-(v - cy)/f`); added `test_world_plus_x_projects_to_larger_u_and_farther_y_to_smaller_v`; reran `spike/perception_bench.py --trials 50` (`100%` detection, `7.0 mm` p95 XY). |
| **F2** | `PLAN:irobots-v2:C67-F2` | done | `PROGRESS.md` | Recorded verified baseline counts, added `C65` & `C66`, marked unmeasured items `not run`, and documented `F3`–`F7` known limitations. |
| **F3** | `PLAN:irobots-v2:C67-F3` | done | `config/perception.yaml`, `camera_source.py`, `outcomes.py`, `loop.py`, `estop.py`, `policy.py`, `sim_node.py`, `web_node.py`, `test_camera_source.py`, `test_web_console.py` | Required validated `MarkerCalibration` (`perception.camera_source.calibration_file`, 4+ points, RMS <= threshold) for `esp32_cam`; added `allow_real_camera_with_sim_arm` hybrid guard & `"hybrid: real camera + simulated arm"` labelling with `task_success_privileged=None`; turned capture failures/timeouts into typed `CameraCaptureError` -> `camera_unavailable` outcome within I4 deadlines; logged `camera_source` and credential-stripped `capture_host` in `episode.jsonl`. |
| **F4** | `PLAN:irobots-v2:C67-F4` | done | `config/perception.yaml`, `web_node.py`, `web/irobots_control_console.html`, `test_web_console.py`, `test_config.py` | Implemented stdlib `WebConsoleServer` (`GET /api/state`, `GET /api/frame.jpg`, `GET /api/config`, `POST /api/camera_source`, `POST /api/task`, `POST /api/approve`, `POST /api/step`, `POST /api/run`, `POST /api/estop`, `POST /api/reset`) + `python -m irobots_nodes.web_node --port N`; removed all external URLs, CSS gradients, simulated ESP32 jitter, and JS copies of `forwardKinematics`, `inverseKinematics`, and `checkSafety`; escaped DOM updates with `textContent`. |
| **F5** | `PLAN:irobots-v2:C67-F5` | done | `pyproject.toml`, `ros2_ws/src/irobots_core/pyproject.toml`, `detector.py`, `geometric.py`, `real_eval.py`, `mujoco_sim.py`, `tui_node.py`, `grasp_bench.py`, `perception_bench.py`, `generate_results_md.py`, `tests/*` | Added targeted mypy overrides for `mujoco`, `ultralytics`, and `cv2`; eliminated all `# type: ignore` comments from `irobots_core` and `irobots_nodes`; resolved all `ruff check ros2_ws tools spike` (`0` errors) and `mypy --strict` (`0` errors) issues. |
| **F6** | `PLAN:irobots-v2:C67-F6` | done | `mujoco_sim.py`, `policy.py`, `exp_rq0.py`, `exp_rq1.py`, `exp_rq2.py`, `exp_rq3.py`, `perception_bench.py`, `generate_results_md.py`, `runs/**`, `docs/RESULTS.md`, `test_eval_and_stats.py` | Tagged all `DeterministicKinematicSim` / pinhole rasteriser outputs with `sim_backend="fallback sim"` in `runs/*.json` and regenerated `docs/RESULTS.md`; kept `runs/real/eval.json` as `"pending_operator_capture"`. |
| **F7** | `PLAN:irobots-v2:C67-F7` | done | `reproduce.sh`, `.github/workflows/ci.yml`, `test_packaging.py`, `PROGRESS.md` | Removed hardcoded `/tmp/irobots_venv` from `reproduce.sh` in favour of `${IROBOTS_VENV:-$SCRIPT_DIR/.venv}`; added headless web console smoke test (`test_web_console.py`) to `.github/workflows/ci.yml`. |

## 5. Known Limitations & What Is Not Run in This Container

- **MuJoCo C physics & renderer (`mujoco`) and YOLO-World (`ultralytics`):** Not installed in this offline container. All simulation and perception numbers regenerated here (`C02`, `SC1`, `SC2`, `SC4`, `RQ0`–`RQ3`) come from `DeterministicKinematicSim` and the pinhole rasteriser and are labelled `"fallback sim"` in `runs/` and `docs/RESULTS.md`. `test_the_analytic_model_matches_the_simulator` is skipped (`1 skipped`), and `WorldDetector` is `not run`.
- **Hosted OpenRouter LLM (`C03` and live-LLM `RQ0`):** `not run` because `OPENROUTER_API_KEY` is not set and outbound network access is disabled in the sandbox.
- **Real ESP32-CAM frame dataset (`C19`/`C20`, `data/real/`):** `not run` (`pending_operator_capture` in `runs/real/eval.json`) until physical ChArUco + object frames are captured.
- **ROS 2 Jazzy DDS graph launch tests (`C23`–`C28`):** `rclpy` / ROS 2 Jazzy is not installed; pure-Python node, message, and launch unit tests run in `test_ros_skin.py`, while live ROS 2 DDS graph integration is `not run`.
- **Docker image (`C60`):** `not run` / omitted per rule because Docker is not available in this container to test a `Dockerfile`.
- **Physical 5-DOF arm hardware:** Not assembled; all items on the "Not verified without the physical arm" list in `docs/RESULTS.md` remain unverified on hardware.

## 6. Needs Operator (Next Steps)

1. **Install `mujoco` and `ultralytics` in your host `.venv` and rerun `./reproduce.sh`:**
   ```bash
   tools/install_deps.sh
   tools/fetch_weights.sh
   ./reproduce.sh
   ```
   This will replace the `"fallback sim"` labels in `runs/` and `docs/RESULTS.md` with full MuJoCo physics/rendering and run `test_the_analytic_model_matches_the_simulator` (`0.0000 um`).
2. **Provide `OPENROUTER_API_KEY` and run C03 + live-LLM RQ0:**
   ```bash
   export OPENROUTER_API_KEY="sk-or-v1-..."
   .venv/bin/python spike/llm_probe.py
   .venv/bin/python -m irobots_eval.exp_rq0
   .venv/bin/python tools/generate_results_md.py
   ```
3. **Capture real ESP32-CAM frames (`data/real/`) with a measured ChArUco/marker calibration:**
   ```bash
   export BOARD_IP="<esp32-cam-ip>"
   .venv/bin/python tools/capture_real_frames.py --board board.yaml --out data/real/ --count 300
   .venv/bin/python tools/capture_real_frames.py --eval data/real/ --report runs/real/eval.json
   .venv/bin/python tools/generate_results_md.py
   ```
4. **Launch the Web Control Console against the live Python server:**
   ```bash
   PYTHONPATH="ros2_ws/src/irobots_core:ros2_ws/src/irobots_nodes:ros2_ws/src/irobots_msgs:ros2_ws/src/irobots_eval" \
     .venv/bin/python -m irobots_nodes.web_node --port 8080
   ```
   Then open `http://127.0.0.1:8080` in your browser.
