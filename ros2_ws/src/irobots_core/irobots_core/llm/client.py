"""Unified OpenAI-compatible client supporting openrouter, lmstudio, and scripted profiles (PLAN:irobots-v2:C29, T37).

No hardcoded IP, port, or model ID in source (Hard Rule §4 / T12): endpoints
and model IDs are read from environment variables or caller configuration.
Every request and response is logged with a SHA-256 hash and provider
fingerprint for reproducibility auditing (PLAN.md §I).
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

from ..config import env_or
from ..skills import SkillCall, tool_schemas_for_worker
from .schema import parse_and_validate_tool_call


class LLMProfile(StrEnum):
    """Supported backend profiles."""

    OPENROUTER = "openrouter"
    LMSTUDIO = "lmstudio"
    SCRIPTED = "scripted"


@dataclass(frozen=True, slots=True)
class LLMCallRecord:
    """Result of one LLM tool-call turn with cryptographic audit hashes."""

    profile: LLMProfile
    model_id: str
    provider_fingerprint: str
    skill_call: SkillCall
    request_sha256: str
    response_sha256: str
    repaired: bool
    used_fallback: bool
    error_log: str = ""


class LLMClient:
    """One client, three profiles (`openrouter`, `lmstudio`, `scripted`)."""

    def __init__(
        self,
        profile: LLMProfile | str = LLMProfile.OPENROUTER,
        *,
        model_id: str | None = None,
        base_url: str | None = None,
        api_key: str | None = None,
        transport: Callable[[LLMProfile, dict[str, Any]], dict[str, Any]] | None = None,
    ) -> None:
        self.profile = LLMProfile(profile)
        self.model_id = model_id or env_or("IROBOTS_LLM_MODEL", "scripted-reference") or "scripted-reference"
        self.base_url = base_url or env_or("IROBOTS_LLM_BASE_URL", "") or ""
        self.api_key = api_key or env_or("OPENROUTER_API_KEY", "") or ""
        self._transport = transport

    def _default_scripted_response(self, available_target_ids: Sequence[str]) -> dict[str, Any]:
        target = available_target_ids[0] if available_target_ids else None
        skill = "move_above" if target else "home"
        args = json.dumps({"target_id": target}) if target else "{}"
        return {
            "id": "scripted_turn",
            "model": self.model_id,
            "system_fingerprint": "scripted_v1",
            "choices": [
                {
                    "message": {
                        "role": "assistant",
                        "content": None,
                        "tool_calls": [
                            {
                                "index": 0,
                                "type": "function",
                                "function": {"name": skill, "arguments": args},
                            }
                        ],
                    }
                }
            ],
        }

    def chat_tool_call(
        self,
        messages: Sequence[Mapping[str, Any]],
        available_target_ids: Sequence[str],
        *,
        fallback_call: SkillCall | None = None,
    ) -> LLMCallRecord:
        payload: dict[str, Any] = {
            "profile": self.profile.value,
            "model": self.model_id,
            "messages": [dict(m) for m in messages],
            "tools": tool_schemas_for_worker(),
            "tool_choice": "required",
        }
        req_bytes = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode("utf-8")
        req_hash = hashlib.sha256(req_bytes).hexdigest()

        if self._transport is not None:
            raw_resp = self._transport(self.profile, payload)
        else:
            raw_resp = self._default_scripted_response(available_target_ids)

        resp_bytes = json.dumps(raw_resp, sort_keys=True, separators=(",", ":")).encode("utf-8")
        resp_hash = hashlib.sha256(resp_bytes).hexdigest()

        default_fb = fallback_call or (
            SkillCall(skill="move_above", target_id=available_target_ids[0])
            if available_target_ids
            else SkillCall(skill="home")
        )
        parsed = parse_and_validate_tool_call(
            raw_resp,
            set(available_target_ids),
            fallback_call=default_fb,
        )
        fingerprint = str(raw_resp.get("system_fingerprint") or "unknown")
        return LLMCallRecord(
            profile=self.profile,
            model_id=str(raw_resp.get("model") or self.model_id),
            provider_fingerprint=fingerprint,
            skill_call=parsed.call,
            request_sha256=req_hash,
            response_sha256=resp_hash,
            repaired=parsed.repaired,
            used_fallback=parsed.used_fallback,
            error_log=parsed.error_log,
        )
