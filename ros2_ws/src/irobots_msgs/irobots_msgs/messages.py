"""Pure-Python message representations matching ``irobots_msgs/msg/*.msg`` (PLAN:irobots-v2:C24)."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class DetectionMsg:
    track_id: str
    label: str
    confidence: float
    bbox_px: tuple[float, float, float, float]
    xy_m: tuple[float, float]
    height_m: float
    height_source: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "track_id": self.track_id,
            "label": self.label,
            "confidence": float(self.confidence),
            "bbox_px": list(self.bbox_px),
            "xy_m": list(self.xy_m),
            "height_m": float(self.height_m),
            "height_source": self.height_source,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> DetectionMsg:
        b = d["bbox_px"]
        xy = d["xy_m"]
        return cls(
            track_id=str(d["track_id"]),
            label=str(d["label"]),
            confidence=float(d["confidence"]),
            bbox_px=(float(b[0]), float(b[1]), float(b[2]), float(b[3])),
            xy_m=(float(xy[0]), float(xy[1])),
            height_m=float(d["height_m"]),
            height_source=str(d["height_source"]),
        )


@dataclass(frozen=True, slots=True)
class SceneMsg:
    frame_id: int
    timestamp_s: float
    detections: tuple[DetectionMsg, ...]

    def to_dict(self) -> dict[str, Any]:
        return {
            "frame_id": int(self.frame_id),
            "timestamp_s": float(self.timestamp_s),
            "detections": [det.to_dict() for det in self.detections],
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> SceneMsg:
        return cls(
            frame_id=int(d["frame_id"]),
            timestamp_s=float(d["timestamp_s"]),
            detections=tuple(DetectionMsg.from_dict(x) for x in d["detections"]),
        )


@dataclass(frozen=True, slots=True)
class SkillCallMsg:
    skill: str
    target_id: str
    turn_index: int

    def to_dict(self) -> dict[str, Any]:
        return {
            "skill": self.skill,
            "target_id": self.target_id,
            "turn_index": int(self.turn_index),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> SkillCallMsg:
        return cls(
            skill=str(d["skill"]),
            target_id=str(d["target_id"]),
            turn_index=int(d["turn_index"]),
        )


@dataclass(frozen=True, slots=True)
class SkillResultMsg:
    skill: str
    target_id: str
    approved: bool
    outcome: str
    elapsed_sim_s: float
    timed_out: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "skill": self.skill,
            "target_id": self.target_id,
            "approved": bool(self.approved),
            "outcome": self.outcome,
            "elapsed_sim_s": float(self.elapsed_sim_s),
            "timed_out": bool(self.timed_out),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> SkillResultMsg:
        return cls(
            skill=str(d["skill"]),
            target_id=str(d["target_id"]),
            approved=bool(d["approved"]),
            outcome=str(d["outcome"]),
            elapsed_sim_s=float(d["elapsed_sim_s"]),
            timed_out=bool(d["timed_out"]),
        )


@dataclass(frozen=True, slots=True)
class PlanMsg:
    task: str
    steps: tuple[SkillCallMsg, ...]
    approved: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "task": self.task,
            "steps": [s.to_dict() for s in self.steps],
            "approved": bool(self.approved),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> PlanMsg:
        return cls(
            task=str(d["task"]),
            steps=tuple(SkillCallMsg.from_dict(s) for s in d["steps"]),
            approved=bool(d["approved"]),
        )


@dataclass(frozen=True, slots=True)
class ArmStateMsg:
    timestamp_s: float
    joints: dict[str, float]
    opening_m: float
    estopped: bool

    def to_dict(self) -> dict[str, Any]:
        return {
            "timestamp_s": float(self.timestamp_s),
            "joints": {k: float(v) for k, v in self.joints.items()},
            "opening_m": float(self.opening_m),
            "estopped": bool(self.estopped),
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> ArmStateMsg:
        return cls(
            timestamp_s=float(d["timestamp_s"]),
            joints={str(k): float(v) for k, v in d["joints"].items()},
            opening_m=float(d["opening_m"]),
            estopped=bool(d["estopped"]),
        )


@dataclass(frozen=True, slots=True)
class TaskResultMsg:
    episode_id: str
    task_success_privileged: bool
    task_success_perceived: bool
    steps: int
    action_log_sha256: str

    def to_dict(self) -> dict[str, Any]:
        return {
            "episode_id": self.episode_id,
            "task_success_privileged": bool(self.task_success_privileged),
            "task_success_perceived": bool(self.task_success_perceived),
            "steps": int(self.steps),
            "action_log_sha256": self.action_log_sha256,
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> TaskResultMsg:
        return cls(
            episode_id=str(d["episode_id"]),
            task_success_privileged=bool(d["task_success_privileged"]),
            task_success_perceived=bool(d["task_success_perceived"]),
            steps=int(d["steps"]),
            action_log_sha256=str(d["action_log_sha256"]),
        )
