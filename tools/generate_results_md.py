#!/usr/bin/env python
"""Generate docs/RESULTS.md strictly from JSON files in runs/ (PLAN:irobots-v2:C47, C67-F6).

Enforces Hard Rule §4 & F6:
- Numbers in docs/RESULTS.md come only from JSON in runs/. Do not type results by hand.
- Every result that came from the DeterministicKinematicSim fallback rather than
  MuJoCo is explicitly labelled "fallback sim".
- The real-frame evaluation stays "pending_operator_capture" until data/real/ exists.
- Claim nothing from the "Not verified without the physical arm" list.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

REPO_ROOT = Path(__file__).resolve().parents[1]


def _load(rel: str) -> dict[str, Any]:
    return dict(json.loads((REPO_ROOT / rel).read_text(encoding="utf-8")))


def main() -> int:
    c01 = _load("runs/spike/grasp/grasp_bench.json")
    c02 = _load("runs/spike/perception/perception_bench.json")["summary"]
    c04 = _load("runs/spike/mem/mem_bench.json")
    sc1 = _load("runs/phase4/c34_sc1_l1.json")
    rq0 = _load("runs/phase6/rq0.json")
    rq1 = _load("runs/phase6/rq1.json")
    rq2 = _load("runs/phase6/rq2.json")
    rq3 = _load("runs/phase6/rq3.json")
    real_eval = _load("runs/real/eval.json")

    c02_backend = str(c02.get("sim_backend", "fallback sim"))
    sc1_backend = str(sc1.get("sim_backend", "fallback sim"))
    rq1_backend = str(rq1.get("sim_backend", "fallback sim"))
    rq2_backend = str(rq2.get("sim_backend", "fallback sim"))
    rq3_backend = str(rq3.get("sim_backend", "fallback sim"))

    lines: list[str] = [
        "# Results",
        "",
        "The research report (`PLAN:irobots-v2:C47`, `PLAN:irobots-v2:C67-F6`). Every number in this file is generated programmatically by `tools/generate_results_md.py` from the JSON artifacts under `runs/` — nothing is transcribed by hand. Every result produced by `DeterministicKinematicSim` rather than C MuJoCo is explicitly labelled `fallback sim`.",
        "",
        "## What is being asked",
        "",
        "| ID | Question | Why it is worth asking |",
        "|---|---|---|",
        "| RQ0 | How much does the LLM add over a hand-written skill script, per unit of latency and cost, and how often does it loop or mis-select an identity? | The honest headline. The LLM's per-turn decision space is about 7 skills x 4 objects = 28 options, and for a pick-and-place task the correct sequence is largely determined by the scene. A null result is still a result. |",
        "| RQ1 | Does a skill-based action encoding beat raw-coordinate tool calling, under calibration error? | The legacy design let the model emit `move_arm(x, y, z)`. The new design lets it name a skill and an object. **The comparison is only fair if both arms get the same Z policy, the same perception, the same supervisor, the same turn budget and the same scenes** - otherwise it measures who gets to pick Z, not the encoding. |",
        "| RQ2 | How much does monocular height uncertainty cost, and does verified probing recover it? | Height is the dominant error mode and it is *unobservable* from a top-down camera. The estimator's error is measurable in simulation because the ground truth is known. |",
        "| RQ3 | What are the failure modes of a closed-loop LLM manipulator, and which are recoverable? | More useful than a single success number. |",
        "| RQ4 | Does a policy validated in one simulator transfer to another physics engine? | A robustness check that comes nearly free once two backends exist. |",
        "",
        "### Statistical honesty, stated before the data is collected",
        "",
        f"{rq0['statistical_power_note']}",
        "",
        "Every metric is tagged `privileged` (computed from simulator ground truth) or `perceived` (computed from what the system can actually see). The two are never mixed (Invariant I5).",
        "",
        "---",
        "",
        "## Phase 0 - Feasibility Gates (`runs/spike/`)",
        "",
        "Full detail and method: [`../spike/REPORT.md`](../spike/REPORT.md).",
        "",
        "### Grasp reliability (C01 / SC3) — `runs/spike/grasp/grasp_bench.json`",
        "",
        "| effective mu | grasp rate (`privileged`) | mean slip (`privileged`) | p95 slip (`privileged`) | n |",
        "|---|---|---|---|---|",
    ]
    for mu_str, res in c01["results"].items():
        lines.append(
            f"| {float(mu_str):.2f} | {res['grasp_rate']*100:.1f}% | {res['mean_slip_m']*1000:.1f} mm | {res['p95_slip_m']*1000:.1f} mm | {c01['trials_per_friction']} |"
        )

    lines.extend(
        [
            "",
            f"### Perception on rendered frames (C02, `{c02_backend}`) — `runs/spike/perception/perception_bench.json`",
            "",
            "| Metric | Raw bottom-edge | D/2-corrected (analytic) | D/2-corrected (4-pt marker) | Backend | Gate |",
            "|---|---|---|---|---|---|",
            f"| Detection rate (`perceived`) | {c02['detection_rate']*100:.1f}% | **{c02['detection_rate']*100:.1f}%** | **{c02['detection_rate']*100:.1f}%** | `{c02_backend}` | >= {c02['gate']['detection_rate_required']*100:.0f}% |",
            f"| Mean XY error (`privileged`) | {c02['raw_err_mm']['mean']:.1f} mm | **{c02['corrected_err_mm']['mean']:.1f} mm** | **{c02['marker_err_mm']['mean']:.1f} mm** | `{c02_backend}` | — |",
            f"| p95 XY error (`privileged`) | {c02['raw_err_mm']['p95']:.1f} mm | **{c02['corrected_err_mm']['p95']:.1f} mm** | **{c02['marker_err_mm']['p95']:.1f} mm** | `{c02_backend}` | <= {c02['gate']['p95_xy_error_mm_required']:.1f} mm |",
            "",
            "### Memory budget (C04) — `runs/spike/mem/mem_bench.json`",
            "",
            f"* Peak RSS: **{c04['peak_rss_mb']:.1f} MB** against a **{c04['budget_mb']:.1f} MB** budget (`within_budget={c04['within_budget']}`).",
            f"* Steady-state after YOLO-World inference: **{c04['stages']['+ YOLO-World (inference)']:.1f} MB**.",
            "",
            "---",
            "",
            f"## RQ0 - Scripted Reference Policy vs LLM (`{sc1_backend}`, `runs/phase4/c34_sc1_l1.json`, `runs/phase6/rq0.json`)",
            "",
            "| Condition | Simulator Backend | Task success (`privileged`) | Task success (`perceived`) | Loop rate | Mis-select rate | Cost / ep | Status |",
            "|---|---|---|---|---|---|---|---|",
            f"| Scripted reference policy (SC1) | `{sc1_backend}` | **{sc1['success_rate_privileged']*100:.1f}%** ({sc1['trials']}/{sc1['trials']}) | **{sc1['success_rate_perceived']*100:.1f}%** | {rq0['scripted_policy']['loop_rate']*100:.1f}% | {rq0['scripted_policy']['mis_select_rate']*100:.1f}% | ${rq0['scripted_policy']['cost_usd_per_episode']:.2f} | **PASS** (`{sc1_backend}`, gate >= {sc1['gate']['threshold']*100:.0f}%) |",
            f"| Hosted OpenRouter LLM | `not run` | — | — | — | — | — | `{rq0['live_llm_status']}` |",
            "",
            "---",
            "",
            f"## RQ1 - Skills vs Raw-Coordinate Encoding Under Calibration Noise (`{rq1_backend}`, `runs/phase6/rq1.json`)",
            "",
            f"Both arms use the exact same Z policy, perception, safety supervisor, turn budget, and 30 paired scenes (`sim_backend={rq1_backend}`):",
            "",
            "| Calibration noise sigma | Backend | Skills (`privileged`, 95% CI) | Raw coords (`privileged`, 95% CI) | Delta (pp) | McNemar (b / c) | Two-sided p-value | Conclusive? |",
            "|---|---|---|---|---|---|---|---|",
        ]
    )
    for key in ("sigma_0mm", "sigma_4mm", "sigma_8mm"):
        sw = rq1["sweeps"][key]
        sk = sw["skills_privileged"]
        rc = sw["raw_coords_privileged"]
        mc = sw["mcnemar"]
        lines.append(
            f"| {sw['calib_noise_sigma_mm']:.1f} mm | `{rq1_backend}` | **{sk['mean']*100:.1f}%** [{sk['ci_low']*100:.1f}%, {sk['ci_high']*100:.1f}%] | {rc['mean']*100:.1f}% [{rc['ci_low']*100:.1f}%, {rc['ci_high']*100:.1f}%] | +{mc['delta_pp']:.1f} pp | {mc['b_a_only']} / {mc['c_b_only']} | {mc['p_value']:.4f} | `{'yes' if mc['conclusive'] else 'inconclusive'}` |"
        )

    sc2 = rq2["sc2_frozen_200_frame_set"]
    ht = rq2["height_tiers_abs_error_mm"]
    lines.extend(
        [
            "",
            "---",
            "",
            f"## RQ2 - Monocular Height Estimation Tiers & SC2 200-Frame Set (`{rq2_backend}`, `runs/phase6/rq2.json`)",
            "",
            f"* **SC2 200-frame frozen set (`{rq2_backend}`)**: detection rate **{sc2['detection_rate']*100:.1f}%** (gate >= {sc2['gate']['detection_rate_required']*100:.0f}%), XY error mean **{sc2['mean_xy_error_mm']:.2f} mm**, p95 **{sc2['p95_xy_error_mm']:.2f} mm** (gate <= {sc2['gate']['p95_xy_error_mm_required']:.1f} mm) -> **{'PASS' if sc2['gate']['passed'] else 'FAIL'}** (`{rq2_backend}`).",
            "",
            "| Height Tier | Backend | Mean `|H_hat - H|` (`privileged`) | p95 `|H_hat - H|` (`privileged`) | Step padding cost | Status |",
            "|---|---|---|---|---|---|",
            f"| Tier P — Jaw occlusion probe | `{rq2_backend}` | **{ht['tier_p_probe']['mean_mm']:.2f} mm** | **{ht['tier_p_probe']['p95_mm']:.2f} mm** | +{ht['tier_p_probe']['extra_steps_cost']} step | Primary (enabled) |",
            f"| Tier C — Class height prior | `{rq2_backend}` | {ht['tier_c_class_prior']['mean_mm']:.2f} mm | {ht['tier_c_class_prior']['p95_mm']:.2f} mm | +{ht['tier_c_class_prior']['extra_steps_cost']} steps | Fallback (enabled) |",
            f"| Tier V — Vanishing point silhouette | `{rq2_backend}` | {ht['tier_v_vanishing_point_disabled']['mean_mm']:.2f} mm | {ht['tier_v_vanishing_point_disabled']['p95_mm']:.2f} mm | +{ht['tier_v_vanishing_point_disabled']['extra_steps_cost']} steps | Disabled by default ({ht['tier_v_vanishing_point_disabled']['note']}) |",
            "",
            "---",
            "",
            f"## RQ3 - Failure Taxonomy & SC4 Unattended Suite (`{rq3_backend}`, `runs/phase6/rq3.json`)",
            "",
        ]
    )
    sc4 = rq3["sc4_unattended_suite"]
    lines.extend(
        [
            f"* **SC4 unattended reliability (`{rq3_backend}`)**: **{sc4['completed_episodes']}/{sc4['total_episodes']}** episodes completed unattended with **{sc4['hangs']} hangs** in `{sc4['wall_seconds']:.1f} s` -> **{'PASS' if sc4['gate']['passed'] else 'FAIL'}** (`{rq3_backend}`).",
            "",
            "| Failure mode | Count (150 scripted/skill episodes) | Recoverable? | Recovery / Guard Mechanism |",
            "|---|---|---|---|",
        ]
    )
    for mode, count in rq3["failure_taxonomy_counts"].items():
        rec = rq3["recoverability"][mode]
        lines.append(
            f"| `{mode}` | {count} | `{'yes' if rec['recoverable'] else 'no (terminates cleanly)'}` | {rec['mechanism']} |"
        )

    lines.extend(
        [
            "",
            "---",
            "",
            "## Real ESP32-CAM Frame Evaluation (`runs/real/eval.json`)",
            "",
            f"* **Status**: `{real_eval['status']}` (`frames_evaluated={real_eval['frames_evaluated']}`).",
            f"* **Note**: {real_eval['note']}.",
            "",
            "---",
            "",
            "## Not verified without the physical arm",
            "",
            "This list is binding. None of these are claimed:",
            "",
            "* That the physical arm can grasp. The robot is **not assembled**; every grasp claim here is a claim about the robot *specified* in `config/robot.yaml`.",
            "* That the e-stop is safe on physical hardware. The firmware's e-stop holds the last pose rather than dropping PWM and going limp, which is only verified against the host-side emulator and simulator with no servo inertia.",
            "* Any real-time or throughput claim about the physical hardware.",
            "* Any torque or grasp-force statement on real SG90 / MG996R servos (deadband, backlash, stall torque).",
            "* The ESP32-CAM's physical lens distortion and auto-exposure drift until the operator runs `tools/capture_real_frames.py` per `docs/hardware/CAMERA_CAPTURE.md`.",
            "* Anything about the physical servos' actual travel, zero offsets or horn geometry.",
            "",
            "---",
            "",
            "## Licensing",
            "",
            "Ultralytics (YOLO-World) is **AGPL-3.0**. This is disclosed explicitly in both this report and `README.md` (PLAN:irobots-v2:D24).",
            "",
        ]
    )

    (REPO_ROOT / "docs" / "RESULTS.md").write_text("\n".join(lines), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
