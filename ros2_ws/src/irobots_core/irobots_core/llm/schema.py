"""Strict JSON-schema validation for worker tool calls (PLAN:irobots-v2:C30, T38, T39).

Fixes the two legacy defects in ``legacy/agent_harness.py:120/128``:
1. ``UnboundLocalError`` when a model emits content or tool calls without
   ``reasoning_content`` tokens.
2. Blindly indexing ``tool_calls[0]`` when streaming or multi-chunk responses
   place the valid function call at ``index >= 1``.

Allows at most one repair retry on malformed JSON or invalid arguments; if the
retry still fails, falls back to the scripted policy and records the fallback.
"""

from __future__ import annotations

import json
from collections.abc import Callable, Mapping, Set
from dataclasses import dataclass
from typing import Any

from ..skills import SkillCall, SkillValidationError, validate_skill_call


@dataclass(frozen=True, slots=True)
class ToolCallParseResult:
    """Validated tool call with repair and fallback provenance."""

    call: SkillCall
    repaired: bool
    used_fallback: bool
    error_log: str = ""
    reasoning_text: str = ""


def _extract_first_valid_tool_call(
    raw_response: Mapping[str, Any],
    known_target_ids: Set[str],
) -> tuple[SkillCall, str]:
    choices = raw_response.get("choices")
    if not isinstance(choices, list) or not choices:
        raise SkillValidationError("LLM response has no 'choices'")
    msg = choices[0].get("message")
    if not isinstance(msg, dict):
        raise SkillValidationError("LLM choice[0] has no 'message' dict")

    # Never fail when reasoning tokens are absent (fixes legacy agent_harness.py:120)
    reasoning = str(msg.get("reasoning_content") or msg.get("reasoning") or "")
    tool_calls = msg.get("tool_calls")
    if not isinstance(tool_calls, list) or not tool_calls:
        raise SkillValidationError("LLM message contains no tool_calls")

    last_err = "no non-empty tool_call entry found"
    # Inspect all tool_call entries in index order (fixes legacy tool_call_chunks[0] bug)
    sorted_calls = sorted(
        (tc for tc in tool_calls if isinstance(tc, dict)),
        key=lambda tc: int(tc.get("index", 0)),
    )
    for tc in sorted_calls:
        fn = tc.get("function")
        if not isinstance(fn, dict):
            continue
        name = str(fn.get("name") or "").strip()
        raw_args = fn.get("arguments", "{}")
        if not name:
            continue
        try:
            if isinstance(raw_args, str):
                args_dict = json.loads(raw_args) if raw_args.strip() else {}
            elif isinstance(raw_args, dict):
                args_dict = dict(raw_args)
            else:
                raise SkillValidationError("tool_call arguments must be JSON object")
            if not isinstance(args_dict, dict):
                raise SkillValidationError("tool_call arguments did not decode to a dict")
        except Exception as exc:
            last_err = f"malformed JSON arguments for {name!r}: {exc}"
            continue

        target_id = args_dict.get("target_id")
        target_str = str(target_id) if target_id is not None else None
        extra_args = {k: v for k, v in args_dict.items() if k != "target_id"}
        candidate = SkillCall(skill=name, target_id=target_str, arguments=extra_args)
        try:
            validate_skill_call(candidate, known_target_ids)
            return candidate, reasoning
        except SkillValidationError as exc:
            last_err = str(exc)

    raise SkillValidationError(last_err)


def parse_and_validate_tool_call(
    raw_response: Mapping[str, Any],
    known_target_ids: Set[str],
    *,
    repair_fn: Callable[[str], Mapping[str, Any]] | None = None,
    fallback_call: SkillCall | None = None,
) -> ToolCallParseResult:
    """Validate an LLM tool-call response with at most one repair retry, then fallback."""
    try:
        call, reasoning = _extract_first_valid_tool_call(raw_response, known_target_ids)
        return ToolCallParseResult(
            call=call,
            repaired=False,
            used_fallback=False,
            reasoning_text=reasoning,
        )
    except SkillValidationError as first_err:
        err_msg = str(first_err)

    if repair_fn is not None:
        try:
            repaired_raw = repair_fn(err_msg)
            call, reasoning = _extract_first_valid_tool_call(repaired_raw, known_target_ids)
            return ToolCallParseResult(
                call=call,
                repaired=True,
                used_fallback=False,
                error_log=f"repaired after: {err_msg}",
                reasoning_text=reasoning,
            )
        except SkillValidationError as second_err:
            err_msg = f"{err_msg}; retry failed: {second_err}"

    if fallback_call is not None:
        return ToolCallParseResult(
            call=fallback_call,
            repaired=False,
            used_fallback=True,
            error_log=f"fallback engaged: {err_msg}",
        )
    raise SkillValidationError(err_msg)
