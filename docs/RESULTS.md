# Results

The research report (`PLAN:irobots-v2:C47`, `PLAN:irobots-v2:C67-F6`). Every number in this file is generated programmatically by `tools/generate_results_md.py` from the JSON artifacts under `runs/` — nothing is transcribed by hand. Every result produced by `DeterministicKinematicSim` rather than C MuJoCo is explicitly labelled `fallback sim`.

## What is being asked

| ID | Question | Why it is worth asking |
|---|---|---|
| RQ0 | How much does the LLM add over a hand-written skill script, per unit of latency and cost, and how often does it loop or mis-select an identity? | The honest headline. The LLM's per-turn decision space is about 7 skills x 4 objects = 28 options, and for a pick-and-place task the correct sequence is largely determined by the scene. A null result is still a result. |
| RQ1 | Does a skill-based action encoding beat raw-coordinate tool calling, under calibration error? | The legacy design let the model emit `move_arm(x, y, z)`. The new design lets it name a skill and an object. **The comparison is only fair if both arms get the same Z policy, the same perception, the same supervisor, the same turn budget and the same scenes** - otherwise it measures who gets to pick Z, not the encoding. |
| RQ2 | How much does monocular height uncertainty cost, and does verified probing recover it? | Height is the dominant error mode and it is *unobservable* from a top-down camera. The estimator's error is measurable in simulation because the ground truth is known. |
| RQ3 | What are the failure modes of a closed-loop LLM manipulator, and which are recoverable? | More useful than a single success number. |
| RQ4 | Does a policy validated in one simulator transfer to another physics engine? | A robustness check that comes nearly free once two backends exist. |

### Statistical honesty, stated before the data is collected

With 30 paired scenes, McNemar's test resolves only ~25-30 percentage-point differences (alpha=0.05, 80% power). Any 10-15 pp difference is reported as inconclusive, not as a trend.

Every metric is tagged `privileged` (computed from simulator ground truth) or `perceived` (computed from what the system can actually see). The two are never mixed (Invariant I5).

---

## Phase 0 - Feasibility Gates (`runs/spike/`)

Full detail and method: [`../spike/REPORT.md`](../spike/REPORT.md).

### Grasp reliability (C01 / SC3) — `runs/spike/grasp/grasp_bench.json`

| effective mu | grasp rate (`privileged`) | mean slip (`privileged`) | p95 slip (`privileged`) | n |
|---|---|---|---|---|
| 0.10 | 0.0% | 1.7 mm | 3.9 mm | 50 |
| 0.20 | 100.0% | 1.5 mm | 2.6 mm | 50 |
| 0.50 | 100.0% | 1.7 mm | 2.6 mm | 50 |

### Perception on rendered frames (C02, `fallback sim`) — `runs/spike/perception/perception_bench.json`

| Metric | Raw bottom-edge | D/2-corrected (analytic) | D/2-corrected (4-pt marker) | Backend | Gate |
|---|---|---|---|---|---|
| Detection rate (`perceived`) | 100.0% | **100.0%** | **100.0%** | `fallback sim` | >= 95% |
| Mean XY error (`privileged`) | 21.4 mm | **3.8 mm** | **3.8 mm** | `fallback sim` | — |
| p95 XY error (`privileged`) | 22.1 mm | **7.0 mm** | **7.0 mm** | `fallback sim` | <= 8.0 mm |

### Memory budget (C04) — `runs/spike/mem/mem_bench.json`

* Peak RSS: **1815.0 MB** against a **2500.0 MB** budget (`within_budget=True`).
* Steady-state after YOLO-World inference: **1075.0 MB**.

---

## RQ0 - Scripted Reference Policy vs LLM (`fallback sim`, `runs/phase4/c34_sc1_l1.json`, `runs/phase6/rq0.json`)

| Condition | Simulator Backend | Task success (`privileged`) | Task success (`perceived`) | Loop rate | Mis-select rate | Cost / ep | Status |
|---|---|---|---|---|---|---|---|
| Scripted reference policy (SC1) | `fallback sim` | **100.0%** (30/30) | **100.0%** | 0.0% | 0.0% | $0.00 | **PASS** (`fallback sim`, gate >= 80%) |
| Hosted OpenRouter LLM | `not run` | — | — | — | — | — | `blocked_no_openrouter_api_key (scripted profile evaluated; export OPENROUTER_API_KEY to run hosted LLM)` |

---

## RQ1 - Skills vs Raw-Coordinate Encoding Under Calibration Noise (`fallback sim`, `runs/phase6/rq1.json`)

Both arms use the exact same Z policy, perception, safety supervisor, turn budget, and 30 paired scenes (`sim_backend=fallback sim`):

| Calibration noise sigma | Backend | Skills (`privileged`, 95% CI) | Raw coords (`privileged`, 95% CI) | Delta (pp) | McNemar (b / c) | Two-sided p-value | Conclusive? |
|---|---|---|---|---|---|---|---|
| 0.0 mm | `fallback sim` | **100.0%** [100.0%, 100.0%] | 70.0% [53.3%, 86.7%] | +30.0 pp | 9 / 0 | 0.0039 | `yes` |
| 4.0 mm | `fallback sim` | **100.0%** [100.0%, 100.0%] | 60.0% [43.3%, 76.7%] | +40.0 pp | 12 / 0 | 0.0005 | `yes` |
| 8.0 mm | `fallback sim` | **83.3%** [66.7%, 96.7%] | 50.0% [33.3%, 66.7%] | +33.3 pp | 12 / 2 | 0.0129 | `yes` |

---

## RQ2 - Monocular Height Estimation Tiers & SC2 200-Frame Set (`fallback sim`, `runs/phase6/rq2.json`)

* **SC2 200-frame frozen set (`fallback sim`)**: detection rate **100.0%** (gate >= 95%), XY error mean **4.16 mm**, p95 **7.83 mm** (gate <= 8.0 mm) -> **PASS** (`fallback sim`).

| Height Tier | Backend | Mean `|H_hat - H|` (`privileged`) | p95 `|H_hat - H|` (`privileged`) | Step padding cost | Status |
|---|---|---|---|---|---|
| Tier P — Jaw occlusion probe | `fallback sim` | **1.00 mm** | **2.00 mm** | +1 step | Primary (enabled) |
| Tier C — Class height prior | `fallback sim` | 25.00 mm | 50.00 mm | +0 steps | Fallback (enabled) |
| Tier V — Vanishing point silhouette | `fallback sim` | 26.23 mm | 30.97 mm | +0 steps | Disabled by default (Confounded by footprint depth D*sin(theta); disabled by default) |

---

## RQ3 - Failure Taxonomy & SC4 Unattended Suite (`fallback sim`, `runs/phase6/rq3.json`)

* **SC4 unattended reliability (`fallback sim`)**: **150/150** episodes completed unattended with **0 hangs** in `48.9 s` -> **PASS** (`fallback sim`).

| Failure mode | Count (150 scripted/skill episodes) | Recoverable? | Recovery / Guard Mechanism |
|---|---|---|---|
| `wrong_object` | 0 | `yes` | persistent tracker ID + plan approval rejection |
| `wrong_target` | 0 | `yes` | unknown_target refusal in worker |
| `grasp_miss` | 0 | `yes` | verification predicate object_lifted=False triggers replan |
| `collision` | 0 | `yes` | SafetySupervisor rejects table_plane / self_collision pre-execution |
| `invalid_tool_call` | 0 | `yes` | 1 repair retry then scripted fallback |
| `planner_loop` | 0 | `yes` | RepeatActionEscalation after 2 identical (skill, target) attempts |
| `timeout` | 0 | `no (terminates cleanly)` | per-skill and per-episode deadlines terminate trial cleanly |
| `knocked_away` | 0 | `no (terminates cleanly)` | table rim predicate detects object outside workspace |

---

## Real ESP32-CAM Frame Evaluation (`runs/real/eval.json`)

* **Status**: `pending_operator_capture` (`frames_evaluated=0`).
* **Note**: data/real/manifest.jsonl is absent; awaiting operator ESP32-CAM capture per docs/hardware/CAMERA_CAPTURE.md.

---

## Not verified without the physical arm

This list is binding. None of these are claimed:

* That the physical arm can grasp. The robot is **not assembled**; every grasp claim here is a claim about the robot *specified* in `config/robot.yaml`.
* That the e-stop is safe on physical hardware. The firmware's e-stop holds the last pose rather than dropping PWM and going limp, which is only verified against the host-side emulator and simulator with no servo inertia.
* Any real-time or throughput claim about the physical hardware.
* Any torque or grasp-force statement on real SG90 / MG996R servos (deadband, backlash, stall torque).
* The ESP32-CAM's physical lens distortion and auto-exposure drift until the operator runs `tools/capture_real_frames.py` per `docs/hardware/CAMERA_CAPTURE.md`.
* Anything about the physical servos' actual travel, zero offsets or horn geometry.

---

## Licensing

Ultralytics (YOLO-World) is **AGPL-3.0**. This is disclosed explicitly in both this report and `README.md` (PLAN:irobots-v2:D24).
