"""Paired statistical comparison (McNemar's test and bootstrap CIs) (PLAN:irobots-v2:C42, T54).

Includes the mandatory statistical honesty note (PLAN.md §H): with 30 paired
scenes, McNemar's test resolves only ~25-30 percentage-point differences; a
10-15 pp difference is reported as inconclusive, not as a trend.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Final

import numpy as np

STATISTICAL_POWER_NOTE: Final[str] = (
    "With 30 paired scenes, McNemar's test resolves only ~25-30 percentage-point "
    "differences (alpha=0.05, 80% power). Any 10-15 pp difference is reported as "
    "inconclusive, not as a trend."
)


@dataclass(frozen=True, slots=True)
class McNemarResult:
    n_pairs: int
    both_pass: int
    b_a_only: int
    c_b_only: int
    both_fail: int
    rate_a: float
    rate_b: float
    delta_pp: float
    p_value: float
    conclusive: bool
    power_note: str = STATISTICAL_POWER_NOTE


@dataclass(frozen=True, slots=True)
class BootstrapCI:
    mean: float
    ci_low: float
    ci_high: float
    n: int


def mcnemar_paired_test(
    outcomes_a: Sequence[bool],
    outcomes_b: Sequence[bool],
    *,
    alpha: float = 0.05,
) -> McNemarResult:
    """Exact two-sided binomial McNemar test on paired binary outcomes (T54)."""
    if len(outcomes_a) != len(outcomes_b):
        raise ValueError("outcomes_a and outcomes_b must have the same length")
    n = len(outcomes_a)
    if n == 0:
        raise ValueError("cannot run McNemar test on 0 pairs")

    both_pass = sum(1 for a, b in zip(outcomes_a, outcomes_b, strict=True) if a and b)
    b_a_only = sum(1 for a, b in zip(outcomes_a, outcomes_b, strict=True) if a and not b)
    c_b_only = sum(1 for a, b in zip(outcomes_a, outcomes_b, strict=True) if not a and b)
    both_fail = sum(1 for a, b in zip(outcomes_a, outcomes_b, strict=True) if not a and not b)

    discordant = b_a_only + c_b_only
    if discordant == 0:
        p_val = 1.0
    else:
        k = min(b_a_only, c_b_only)
        tail = sum(math.comb(discordant, i) for i in range(k + 1)) / (2.0**discordant)
        p_val = min(1.0, 2.0 * tail)

    rate_a = sum(1 for a in outcomes_a if a) / n
    rate_b = sum(1 for b in outcomes_b if b) / n
    delta_pp = (rate_a - rate_b) * 100.0
    conclusive = bool(p_val < alpha and abs(delta_pp) >= 25.0)
    return McNemarResult(
        n_pairs=n,
        both_pass=both_pass,
        b_a_only=b_a_only,
        c_b_only=c_b_only,
        both_fail=both_fail,
        rate_a=rate_a,
        rate_b=rate_b,
        delta_pp=delta_pp,
        p_value=p_val,
        conclusive=conclusive,
    )


def bootstrap_ci(
    values: Sequence[float],
    *,
    seed: int = 0,
    n_boot: int = 2000,
    confidence: float = 0.95,
) -> BootstrapCI:
    """Seeded non-parametric percentile bootstrap confidence interval (T54)."""
    arr = np.asarray(values, dtype=float)
    if arr.size == 0:
        raise ValueError("values must not be empty")
    rng = np.random.default_rng(seed)
    idx = rng.integers(0, arr.size, size=(n_boot, arr.size))
    means = arr[idx].mean(axis=1)
    alpha = (1.0 - confidence) / 2.0
    low = float(np.percentile(means, alpha * 100.0))
    high = float(np.percentile(means, (1.0 - alpha) * 100.0))
    return BootstrapCI(mean=float(arr.mean()), ci_low=low, ci_high=high, n=int(arr.size))
