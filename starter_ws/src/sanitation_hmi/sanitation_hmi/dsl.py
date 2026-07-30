"""Fail-closed natural-language to allowlisted task DSL conversion."""

from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any


ALLOWLIST = {
    "mission.start_coverage",
    "mission.start_spot_clean",
    "mission.schedule",
    "mission.pause",
    "mission.resume",
    "mission.return_home",
    "mission.status",
    "safety.emergency_stop",
}
DIRECT_ACTUATOR_TERMS = {
    "cmd_vel",
    "/cmd_vel",
    "joint",
    "关节",
    "电机",
    "motor",
    "actuator",
    "刷盘转速",
    "wheel speed",
}
INTENT_PATTERNS = {
    "emergency_stop": (
        r"\b(emergency stop|e[- ]?stop|stop immediately)\b",
        r"(急停|紧急停止|立刻停下)",
    ),
    "pause": (r"\b(pause|hold)\b", r"(暂停|先停一下)"),
    "resume": (r"\b(resume|continue)\b", r"(继续|恢复任务)"),
    "return_home": (
        r"\b(return|go) (home|to base)\b",
        r"(返航|返回充电站|回基地)",
    ),
    "status": (r"\b(status|progress|battery)\b", r"(状态|进度|电量)"),
    "schedule": (r"\b(schedule|at \d{1,2}:\d{2})\b", r"(定时|预约|\d{1,2}点)"),
    "spot_clean": (
        r"\b(spot clean|pick up|clean the (bottle|can|paper|leaves|puddle))\b",
        r"(定点清扫|捡起|清理.*(瓶|罐|纸|落叶|积水))",
    ),
    "start_coverage": (
        r"\b(start|begin|clean) .*(zone|area)\b",
        r"(开始|执行).*(区域|分区).*(清扫)?",
    ),
}
TOOL_BY_INTENT = {
    "start_coverage": "mission.start_coverage",
    "spot_clean": "mission.start_spot_clean",
    "schedule": "mission.schedule",
    "pause": "mission.pause",
    "resume": "mission.resume",
    "return_home": "mission.return_home",
    "status": "mission.status",
    "emergency_stop": "safety.emergency_stop",
}
CLASS_ALIASES = {
    "plastic_bottle": ("plastic bottle", "bottle", "塑料瓶", "瓶子"),
    "metal_can": ("metal can", "can", "易拉罐", "罐子"),
    "paper_litter": ("paper litter", "paper", "废纸", "纸屑"),
    "leaf_pile": ("leaf pile", "leaves", "落叶", "树叶"),
    "puddle": ("puddle", "积水", "水渍"),
}


@dataclass(frozen=True)
class ParseResult:
    status: str
    dsl: dict[str, Any] | None
    reason: str | None


def _normalize(text: str) -> str:
    normalized = text.strip().lower()
    normalized = normalized.replace("，", ",").replace("：", ":")
    return re.sub(r"\s+", " ", normalized)


def _matches(text: str, patterns: tuple[str, ...]) -> bool:
    return any(re.search(pattern, text, flags=re.IGNORECASE) for pattern in patterns)


def _extract_zone(text: str) -> str | None:
    match = re.search(
        r"(?:zone|area|区域|分区)\s*[-_ ]?([a-z0-9\u4e00-\u9fff]{1,16})",
        text,
        flags=re.IGNORECASE,
    )
    return match.group(1).upper() if match else None


def _extract_time(text: str) -> str | None:
    match = re.search(r"\b([01]?\d|2[0-3]):([0-5]\d)\b", text)
    if match:
        return f"{int(match.group(1)):02d}:{match.group(2)}"
    match = re.search(r"([01]?\d|2[0-3])点(?:([0-5]?\d)分?)?", text)
    if match:
        return f"{int(match.group(1)):02d}:{int(match.group(2) or 0):02d}"
    return None


def _extract_class(text: str) -> str | None:
    for class_id, aliases in CLASS_ALIASES.items():
        if any(alias in text for alias in aliases):
            return class_id
    return None


def parse_command(text: str) -> ParseResult:
    normalized = _normalize(text)
    if not normalized:
        return ParseResult("REJECTED", None, "empty_command")
    if any(term in normalized for term in DIRECT_ACTUATOR_TERMS):
        return ParseResult("REJECTED", None, "direct_actuator_access_forbidden")
    matched = [
        intent
        for intent, patterns in INTENT_PATTERNS.items()
        if _matches(normalized, patterns)
    ]
    if "emergency_stop" in matched:
        matched = ["emergency_stop"]
    if len(matched) != 1:
        reason = "unresolved_ambiguity" if matched else "unsupported_intent"
        return ParseResult("REJECTED", None, reason)
    intent = matched[0]
    arguments: dict[str, Any] = {}
    if intent in {"start_coverage", "schedule"}:
        zone = _extract_zone(normalized)
        if not zone:
            return ParseResult("REJECTED", None, "missing_zone")
        arguments["zone_id"] = zone
    if intent == "schedule":
        start_time = _extract_time(normalized)
        if not start_time:
            return ParseResult("REJECTED", None, "missing_schedule_time")
        arguments["start_time_local"] = start_time
    if intent == "spot_clean":
        class_id = _extract_class(normalized)
        if not class_id:
            return ParseResult("REJECTED", None, "missing_target_class")
        arguments["class_id"] = class_id
    tool = TOOL_BY_INTENT[intent]
    if tool not in ALLOWLIST:
        return ParseResult("REJECTED", None, "tool_not_allowlisted")
    dsl = {
        "intent": intent,
        "ordered_subtasks": [
            {"sequence": 1, "task": tool, "arguments": arguments}
        ],
        "tool_calls": [tool],
        "arguments": arguments,
        "constraints": {
            "allowlisted_tools_only": True,
            "direct_actuator_access": False,
            "safety_supervisor_required": True,
        },
        "expected_terminal_state": (
            "SAFE_STOPPED" if intent == "emergency_stop" else "TASK_ACCEPTED"
        ),
    }
    return ParseResult("ACCEPTED", dsl, None)


def validate_dsl(dsl: dict[str, Any]) -> list[str]:
    errors = []
    required = {
        "intent",
        "ordered_subtasks",
        "tool_calls",
        "arguments",
        "constraints",
        "expected_terminal_state",
    }
    missing = sorted(required - set(dsl))
    if missing:
        errors.append("missing_fields:" + ",".join(missing))
    if any(tool not in ALLOWLIST for tool in dsl.get("tool_calls", [])):
        errors.append("tool_not_allowlisted")
    if dsl.get("constraints", {}).get("direct_actuator_access") is not False:
        errors.append("direct_actuator_access_not_false")
    serialized = repr(dsl).lower()
    if "/cmd_vel" in serialized or "joint_command" in serialized:
        errors.append("direct_actuator_topic_present")
    return errors
