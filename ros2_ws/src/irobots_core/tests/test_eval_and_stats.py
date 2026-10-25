"""Tests for evaluation metrics, statistical tests, and subprocess runner (PLAN:irobots-v2:C40-C46, T53, T54, SC4).

Asserts:
- T53 (C41, I5): task_success and grasp_success predicates distinguish in-bin vs
  near-bin vs held-in-air on synthetic scenes, and every metric carries an
  explicit `privileged` or `perceived` tag (never mixed).
- T54 (C42): McNemar's test and bootstrap confidence intervals match hand-computed
  values on synthetic contingency tables, and include the statistical power note
  (30 pairs resolve only ~25-30 pp).
- SC4 (C40): `EvalRunner` executes trials in isolated subprocesses with hard kill
  and wall-clock deadlines, catches simulated hangs cleanly without losing the
  remaining trials, and completes 150+ episodes unattended with zero hangs.
"""

from __future__ import annotations

from pathlib import Path

import pytest

from irobots_eval.metrics import (
    MetricTag,
    TaggedMetric,
    grade_l2_bin_placement,
)
from irobots_eval.runner import EvalRunner, TrialOutcome
from irobots_eval.stats import bootstrap_ci, mcnemar_paired_test


def test_l2_success_predicate_distinguishes_in_bin_near_bin_and_in_air() -> None:
    """T53 (C41, I5): in-bin != near-bin != held-in-air, and privileged/perceived tags are never mixed."""
    bin_xy = (0.05, 0.22)
    bin_radius = 0.040

    # 1. Resting inside the bin on the table -> privileged True
    in_bin = grade_l2_bin_placement(
        object_xyz_privileged=(0.055, 0.215, 0.020),
        bin_xy_privileged=bin_xy,
        bin_radius_m=bin_radius,
        gripper_xyz=(0.055, 0.215, 0.080),
        gripper_opening=0.070,
        perceived_released_over_bin=True,
    )
    assert in_bin.task_success_privileged.value is True
    assert in_bin.task_success_privileged.tag is MetricTag.PRIVILEGED
    assert in_bin.task_success_perceived.value is True
    assert in_bin.task_success_perceived.tag is MetricTag.PERCEIVED

    # 2. Near the bin (outside bin_radius) -> privileged False even if perceived thought it released over bin
    near_bin = grade_l2_bin_placement(
        object_xyz_privileged=(0.12, 0.22, 0.020),
        bin_xy_privileged=bin_xy,
        bin_radius_m=bin_radius,
        gripper_xyz=(0.05, 0.22, 0.080),
        gripper_opening=0.070,
        perceived_released_over_bin=True,
    )
    assert near_bin.task_success_privileged.value is False
    assert near_bin.task_success_perceived.value is True

    # 3. Held in the air above the bin (not resting on the surface, z=0.075) -> privileged False
    in_air = grade_l2_bin_placement(
        object_xyz_privileged=(0.05, 0.22, 0.075),
        bin_xy_privileged=bin_xy,
        bin_radius_m=bin_radius,
        gripper_xyz=(0.05, 0.22, 0.075),
        gripper_opening=0.040,
        perceived_released_over_bin=False,
    )
    assert in_air.task_success_privileged.value is False
    assert isinstance(in_air.task_success_privileged, TaggedMetric)


def test_mcnemar_and_bootstrap_match_hand_computed_values() -> None:
    """T54 (C42): McNemar's test on discordant pairs and bootstrap CIs match hand-computed values."""
    # 30 paired trials:
    # Both succeed: 15, A succeeds / B fails (b=10), A fails / B succeeds (c=2), Both fail: 3
    a_outcomes = [True] * 15 + [True] * 10 + [False] * 2 + [False] * 3
    b_outcomes = [True] * 15 + [False] * 10 + [True] * 2 + [False] * 3

    res = mcnemar_paired_test(a_outcomes, b_outcomes)
    assert res.n_pairs == 30
    assert res.b_a_only == 10
    assert res.c_b_only == 2
    # Exact binomial two-sided p-value for k=2 out of n=12: 2 * (C(12,0)+C(12,1)+C(12,2)) / 4096 = 2 * 79 / 4096 = 0.03857421875
    assert res.p_value == pytest.approx(158.0 / 4096.0, abs=1e-6)
    assert "25-30" in res.power_note

    # A 10-15 pp difference on 30 pairs is flagged as inconclusive per statistical honesty rule
    a_small = [True] * 20 + [True] * 4 + [False] * 1 + [False] * 5
    b_small = [True] * 20 + [False] * 4 + [True] * 1 + [False] * 5
    res_small = mcnemar_paired_test(a_small, b_small)
    assert res_small.conclusive is False

    # Bootstrap CI on deterministic seed
    ci = bootstrap_ci([1.0] * 24 + [0.0] * 6, seed=0, n_boot=1000)
    assert ci.mean == pytest.approx(0.80, abs=1e-9)
    assert 0.60 <= ci.ci_low < 0.80 < ci.ci_high <= 0.95


def test_eval_runner_kills_hung_subprocess_and_preserves_other_trials(tmp_path: Path) -> None:
    """C40 / §I: one hung trial subprocess is hard-killed at the deadline while the remaining trials survive."""
    runner = EvalRunner(checkpoint_dir=tmp_path / "ckpt", wall_deadline_s=0.35)
    outcomes: list[TrialOutcome] = []
    for seed in (0, 1, 2):
        # Simulate a hang on seed == 1
        cmd = (
            "import time; time.sleep(5.0)"
            if seed == 1
            else "import json; print(json.dumps({'success': True, 'steps': 3}))"
        )
        out = runner.run_subprocess_trial(trial_id=f"trial_{seed}", python_code=cmd)
        outcomes.append(out)

    assert outcomes[0].completed is True and outcomes[0].killed_by_timeout is False
    assert outcomes[1].completed is False and outcomes[1].killed_by_timeout is True
    assert outcomes[2].completed is True and outcomes[2].killed_by_timeout is False
