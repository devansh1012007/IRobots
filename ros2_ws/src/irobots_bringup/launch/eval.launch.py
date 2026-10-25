"""ROS 2 launch file for eval (PLAN:irobots-v2:C28)."""

from __future__ import annotations

from irobots_bringup.launch_specs import build_eval_launch_spec


def generate_launch_description() -> dict[str, object]:
    return build_eval_launch_spec()
