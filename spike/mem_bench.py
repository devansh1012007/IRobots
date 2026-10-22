#!/usr/bin/env python
"""C04 - the memory budget.

The binding constraint on this machine is RAM, not disk: 12 GiB total with
about 5.9 GiB available, and the Docker agent stack must keep running. The
simulator, the physics, and the detector all have to fit in that.

This measures the steady-state RSS of the real stack rather than assuming it.

Usage::

    python spike/mem_bench.py
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "ros2_ws" / "src" / "irobots_core"))


def rss_mb() -> float:
    """Resident set size of this process, in MiB."""
    with open("/proc/self/statm", "r", encoding="utf-8") as fh:
        pages = int(fh.read().split()[1])
    return pages * os.sysconf("SC_PAGE_SIZE") / (1024 * 1024)


def peak_rss_mb() -> float:
    try:
        import resource

        return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    except Exception:  # noqa: BLE001
        return 0.0


def main() -> int:
    import json

    budget_mb = 2500.0
    report: dict[str, object] = {"budget_mb": budget_mb, "stages": {}}

    def mark(stage: str) -> None:
        report["stages"][stage] = round(rss_mb(), 1)  # type: ignore[index]
        print(f"  {stage:<28s} RSS = {rss_mb():8.1f} MB")

    print("memory budget:")
    mark("python baseline")
    import numpy as np

    mark("+ numpy")

    import mujoco  # noqa: F401

    mark("+ mujoco")

    from irobots_core.config import load_yaml
    from irobots_core.robot_model import load_robot
    from irobots_core.sim.mujoco_sim import MuJoCoSim

    robot = load_robot()
    scene = load_yaml("config/scene.yaml")["scene"]
    mark("+ core (robot, scene)")

    objs = [
        {
            "name": "cube",
            "size": [0.04, 0.04, 0.04],
            "pos": [0.0, 0.2, 0.04],
            "mass": 0.04,
            "friction": 0.6,
            "rgba": "0.85 0.25 0.25 1",
        }
    ]
    sim = MuJoCoSim(robot, scene, objs)
    sim.reset(dict(robot.home_pose), robot.gripper.max_opening)
    sim.step(200)
    mark("+ mujoco scene + physics")

    try:
        import torch

        torch.set_num_threads(max(1, (os.cpu_count() or 2) // 2))
        mark("+ torch")
    except Exception as exc:  # noqa: BLE001
        print(f"  torch unavailable: {exc}")

    try:
        from ultralytics import YOLOWorld

        model = YOLOWorld(str(REPO_ROOT / "weights" / "yolov8s-world.pt"))
        model.set_classes(["red block", "blue block", "green block", "cube"])
        mark("+ YOLO-World (loaded)")
        import numpy as np

        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[200:240, 300:340] = 200
        for _ in range(3):
            model.predict(frame, imgsz=640, verbose=False, device="cpu")
        mark("+ YOLO-World (inference)")
    except Exception as exc:  # noqa: BLE001
        print(f"  detector unavailable: {type(exc).__name__}: {exc}")

    peak = peak_rss_mb()
    report["peak_rss_mb"] = round(peak, 1)
    report["within_budget"] = peak <= budget_mb
    print(f"\n  peak RSS = {peak:.1f} MB   budget = {budget_mb:.0f} MB")
    print(f"  GATE {'PASS' if peak <= budget_mb else 'FAIL'}")

    out = REPO_ROOT / "runs" / "spike" / "mem"
    out.mkdir(parents=True, exist_ok=True)
    (out / "mem_bench.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    return 0 if peak <= budget_mb else 1


if __name__ == "__main__":
    raise SystemExit(main())
