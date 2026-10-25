"""Declarative launch specifications shared by ROS 2 launch files and tests (PLAN:irobots-v2:C28, T36)."""

from __future__ import annotations

from typing import Any


def build_eval_launch_spec() -> dict[str, Any]:
    """Headless fixed-step evaluation launch specification (`eval.launch.py`)."""
    return {
        "name": "eval",
        "headless": True,
        "fixed_step": True,
        "use_sim_time": True,
        "nodes": [
            {"package": "irobots_nodes", "executable": "sim_node", "parameters": {"use_sim_time": True, "headless": True}},
            {"package": "irobots_nodes", "executable": "safety_node", "parameters": {"use_sim_time": True}},
            {"package": "irobots_nodes", "executable": "control_node", "parameters": {"use_sim_time": True}},
            {"package": "irobots_nodes", "executable": "recorder_node", "parameters": {"use_sim_time": True}},
        ],
    }


def build_demo_launch_spec() -> dict[str, Any]:
    """Operator TUI launch specification (`demo.launch.py`, GUI off by default)."""
    return {
        "name": "demo",
        "headless": True,
        "fixed_step": True,
        "use_sim_time": True,
        "nodes": [
            {"package": "irobots_nodes", "executable": "sim_node", "parameters": {"use_sim_time": True, "gui": False}},
            {"package": "irobots_nodes", "executable": "safety_node", "parameters": {"use_sim_time": True}},
            {"package": "irobots_nodes", "executable": "control_node", "parameters": {"use_sim_time": True}},
            {"package": "irobots_nodes", "executable": "tui_node", "parameters": {"use_sim_time": True}},
        ],
    }


def build_perception_bench_launch_spec() -> dict[str, Any]:
    """Perception benchmark launch specification (`perception_bench.launch.py`)."""
    return {
        "name": "perception_bench",
        "headless": True,
        "fixed_step": True,
        "use_sim_time": True,
        "nodes": [
            {"package": "irobots_nodes", "executable": "sim_node", "parameters": {"use_sim_time": True}},
            {"package": "irobots_nodes", "executable": "recorder_node", "parameters": {"use_sim_time": True}},
        ],
    }
