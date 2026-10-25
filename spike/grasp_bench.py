#!/usr/bin/env python
"""C01 - the grasp benchmark. THE GATE FOR THE WHOLE PROJECT.

The question this answers: **can this arm pick a 4cm cube up, in simulation,
reliably?** Everything else in the project is downstream of that answer. If
grasping does not work, the artifact has no result, so this runs first, on
day one, before any ROS 2, before any LLM, before any perception.

Design notes:

* The policy uses GROUND-TRUTH object poses. That is deliberate: it isolates
  *physics* from *perception*. If the grasp rate is low, we know the problem
  is contact and grasping, not the detector. The perception half is measured
  separately by ``perception_bench.py``.
* The object is NEVER attached to the gripper. A grasp succeeds because
  friction holds it. Attaching the object would make "the object moved with
  the gripper" a tautology and would measure the attach trigger instead of
  grasping, so it is not an option.
* The grasp rate and the slip distance are OUTPUTS, not assumptions. A low
  grasp rate is a finding to report, not a bug to hide.

Usage::

    python spike/grasp_bench.py --trials 50 --frictions 0.3,0.5,0.8,1.2
    python spike/grasp_bench.py --trials 20 --quick
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "irobots_core"))

from irobots_core.config import load_yaml  # noqa: E402
from irobots_core.kinematics import IKStatus, inverse_kinematics  # noqa: E402
from irobots_core.robot_model import RobotModel, load_robot  # noqa: E402
from irobots_core.sim.mujoco_sim import MuJoCoSim  # noqa: E402

CUBE_NAME = "cube"
CUBE_SIZE = 0.04
#: How far the BOTTOM of the object must clear the table to count as a grasp.
#: This is the physically meaningful criterion: an object is "lifted" when it
#: is off the table, not when it has moved by some arbitrary number of
#: millimetres. Measuring the height gain instead makes the threshold depend
#: on how high the lift waypoint happens to be.
LIFT_CLEARANCE_M = 0.015
#: How far the object may drift from the tool tip and still count as held.
HOLD_TOLERANCE_M = 0.035


@dataclass
class Trial:
    seed: int
    friction: float
    success: bool
    lifted: bool
    slipped: bool
    knocked: bool
    height_gain_m: float
    bottom_clearance_m: float
    slip_m: float
    sim_seconds: float
    wall_seconds: float
    final_opening_m: float
    note: str = ""


def _yaw_quat(yaw: float) -> list[float]:
    return [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)]


def _sample_pose(rng: np.random.Generator, centre: tuple[float, float], radius: float) -> tuple[float, float, float]:
    r = radius * math.sqrt(float(rng.random()))
    theta = float(rng.random()) * 2.0 * math.pi
    x = centre[0] + r * math.cos(theta)
    y = centre[1] + r * math.sin(theta)
    yaw = float(rng.random()) * math.pi  # 0..180 deg: a cube has 4-fold symmetry
    return x, y, yaw


def run_trial(
    sim: MuJoCoSim,
    robot,
    rng: np.random.Generator,
    seed: int,
    friction: float,
) -> Trial:
    """One scripted grasp attempt, using ground-truth object poses."""
    wall_start = time.perf_counter()
    centre = robot.safety.workspace_centre
    radius = robot.safety.workspace_radius * 0.85
    x, y, cube_yaw = _sample_pose(rng, centre, radius)

    home = dict(robot.home_pose)
    sim.reset(home, robot.gripper.max_opening)
    # Place the cube on the table and let it settle.
    sim.set_object_pose(CUBE_NAME, (x, y, CUBE_SIZE / 2.0), _yaw_quat(cube_yaw))
    sim.step(120)
    obj_start = sim.object_position(CUBE_NAME)

    # The jaws close across the cube's local X axis, so the closing azimuth is
    # the cube's yaw.
    closing_azimuth = cube_yaw
    grasp_z = CUBE_SIZE / 2.0
    above_z = 0.06

    note = ""
    reached = True
    for target_z, opening in (
        (above_z, robot.gripper.max_opening),
        (grasp_z, robot.gripper.max_opening),
        (grasp_z, robot.gripper.min_opening),
        (above_z, robot.gripper.min_opening),
    ):
        res = inverse_kinematics(
            robot,
            (obj_start[0], obj_start[1], target_z),
            closing_azimuth=closing_azimuth,
        )
        if res.status is not IKStatus.REACHABLE:
            note = f"ik:{res.status.value} z={target_z:.3f}"
            reached = False
            break
        sim.set_joint_targets(res.pose)
        if opening >= robot.gripper.max_opening - 1e-9:
            sim.set_opening(opening)
        else:
            # Close the jaws: the position actuators push until contact, and
            # the force limit stops them driving through the object.
            sim.set_opening(robot.gripper.min_opening)
        sim.step_until_settled(max_steps=2500, tol=5e-4, settle_steps=40)

    obj_end = sim.object_position(CUBE_NAME)
    tool_end = sim.tool_position()
    height_gain = float(obj_end[2] - obj_start[2])
    # How far the object's underside is above the table surface.
    bottom_clearance = float(obj_end[2] - CUBE_SIZE / 2.0)
    slip = float(np.linalg.norm(obj_end[:2] - tool_end[:2]))
    lifted = bottom_clearance > LIFT_CLEARANCE_M
    held = slip < HOLD_TOLERANCE_M
    # Knocked away = it left the working area, not merely the table.
    _wc = robot.safety.workspace_centre
    _wr = robot.safety.workspace_radius
    knocked = bool(
        math.hypot(obj_end[0] - _wc[0], obj_end[1] - _wc[1]) > _wr + 0.04
    )
    success = bool(lifted and held and reached)
    if not reached:
        note = note or "unreachable"
    elif not lifted:
        note = note or "not lifted"
    elif not held:
        note = note or "slipped"

    return Trial(
        seed=seed,
        friction=friction,
        success=success,
        lifted=lifted,
        slipped=not held,
        knocked=knocked,
        height_gain_m=height_gain,
        bottom_clearance_m=bottom_clearance,
        slip_m=slip,
        sim_seconds=sim.total_steps * sim.dt,
        wall_seconds=time.perf_counter() - wall_start,
        final_opening_m=sim.opening(),
        note=note,
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trials", type=int, default=50, help="trials per friction value")
    ap.add_argument("--frictions", type=str, default="0.3,0.5,0.8,1.2")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true", help="10 trials, one friction")
    ap.add_argument("--out", type=str, default=str(REPO_ROOT / "runs" / "spike" / "grasp"))
    args = ap.parse_args()

    frictions = [0.3, 0.5, 0.8, 1.2] if args.quick is False else [0.5]
    trials = 10 if args.quick else args.trials
    if args.frictions and not args.quick:
        frictions = [float(f) for f in args.frictions.split(",")]

    robot = load_robot()
    scene = load_yaml("config/scene.yaml")["scene"]
    cube_spec = scene["objects"]["cube"]
    objects = [
        {
            "name": CUBE_NAME,
            "size": [CUBE_SIZE, CUBE_SIZE, CUBE_SIZE],
            "pos": [0.0, 0.0, CUBE_SIZE],
            "mass": float(cube_spec.get("mass", 0.04)),
            "friction": float(cube_spec.get("friction", 0.6)),
            "rgba": "0.85 0.25 0.25 1",
        }
    ]

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {"trials_per_friction": trials, "results": {}}
    all_trials: list[Trial] = []

    for mu in frictions:
        sim = MuJoCoSim(
            robot,
            scene,
            objects,
            options={},
        )
        # Sweep the EFFECTIVE friction. MuJoCo combines the sliding
        # coefficients of two contacting geoms by taking the MAX, so setting
        # only the cube's friction would be masked by the pads' 1.4 and the
        # sweep would measure nothing. Both are set to the swept value.
        for i in range(sim.mj.ngeom):
            name = sim.mj.geom(i).name
            if name == f"{CUBE_NAME}_geom" or name in ("finger_pos_pad", "finger_neg_pad"):
                sim.mj.geom_friction[i][0] = mu
                sim.mj.geom_friction[i][1] = 0.01
                sim.mj.geom_friction[i][2] = 0.0001
        rng = np.random.default_rng(args.seed + int(mu * 1000))
        mu_trials: list[Trial] = []
        t0 = time.perf_counter()
        for i in range(trials):
            trial = run_trial(sim, robot, rng, args.seed + i, mu)
            mu_trials.append(trial)
            all_trials.append(trial)
        elapsed = time.perf_counter() - t0
        sim.close()

        n = len(mu_trials)
        n_succ = sum(1 for t in mu_trials if t.success)
        n_lift = sum(1 for t in mu_trials if t.lifted)
        n_hold = sum(1 for t in mu_trials if not t.slipped)
        slips = [t.slip_m for t in mu_trials]
        heights = [t.height_gain_m for t in mu_trials]
        notes: dict[str, int] = {}
        for t in mu_trials:
            notes[t.note or "ok"] = notes.get(t.note or "ok", 0) + 1
        rate = n_succ / n if n else 0.0
        summary["results"][f"{mu}"] = {
            "grasp_rate": rate,
            "lifted_rate": n_lift / n if n else 0.0,
            "held_rate": n_hold / n if n else 0.0,
            "mean_slip_m": float(np.mean(slips)) if slips else 0.0,
            "p95_slip_m": float(np.percentile(slips, 95)) if slips else 0.0,
            "mean_height_gain_m": float(np.mean(heights)) if heights else 0.0,
            "failure_notes": notes,
            "wall_seconds": elapsed,
            "mean_sim_seconds_per_trial": float(
                np.mean([t.sim_seconds for t in mu_trials])
            ),
        }
        print(
            f"mu={mu:<5} grasp={rate:6.1%}  lifted={n_lift/n:6.1%}  "
            f"held={n_hold/n:6.1%}  mean_slip={np.mean(slips) * 1000:6.1f}mm  "
            f"notes={notes}  [{elapsed:.1f}s]"
        )

    gate = 0.8
    best_mu = max(summary["results"], key=lambda k: summary["results"][k]["grasp_rate"])
    best_rate = summary["results"][best_mu]["grasp_rate"]
    summary["gate"] = {
        "threshold": gate,
        "best_friction": float(best_mu),
        "best_grasp_rate": best_rate,
        "passed": best_rate >= gate,
    }
    summary["trials"] = [asdict(t) for t in all_trials]

    out_file = out_dir / "grasp_bench.json"
    out_file.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print()
    print(f"best: mu={best_mu} rate={best_rate:.1%}  (gate: {gate:.0%})")
    print(f"GATE {'PASS' if best_rate >= gate else 'FAIL'}  -> {out_file}")
    return 0 if best_rate >= gate else 1


if __name__ == "__main__":
    raise SystemExit(main())
