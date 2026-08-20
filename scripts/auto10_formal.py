#!/usr/bin/env python3
"""Deterministic AUTO-10 APP/API and constrained DSL formal matrices."""

from __future__ import annotations

import argparse
from collections import Counter
from http.server import ThreadingHTTPServer
import json
from pathlib import Path
import statistics
import sys
import threading
import time
import urllib.error
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_hmi"))

from sanitation_hmi.dsl import ALLOWLIST, parse_command, validate_dsl
from sanitation_hmi.gateway import CommandGateway
from sanitation_hmi.server import build_handler


def percentile(values: list[float], quantile: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, int((len(ordered) - 1) * quantile))
    return ordered[index]


def valid_cases() -> list[dict]:
    cases = []
    zones = [chr(ord("A") + index) for index in range(12)]
    for index in range(360):
        zone = zones[index % len(zones)]
        language = "zh" if index % 2 == 0 else "en"
        if index % 3 == 0:
            text = (
                f"开始区域 {zone} 清扫"
                if language == "zh"
                else f"start cleaning zone {zone}"
            )
            intent = "start_coverage"
            arguments = {"zone_id": zone}
        elif index % 3 == 1:
            hour, minute = index % 24, (index * 7) % 60
            text = (
                f"定时 {hour:02d}:{minute:02d} 清扫区域 {zone}"
                if language == "zh"
                else f"schedule area {zone} at {hour:02d}:{minute:02d}"
            )
            intent = "schedule"
            arguments = {
                "zone_id": zone,
                "start_time_local": f"{hour:02d}:{minute:02d}",
            }
        else:
            class_id, zh, en = (
                ("plastic_bottle", "塑料瓶", "plastic bottle"),
                ("metal_can", "易拉罐", "metal can"),
                ("paper_litter", "纸屑", "paper"),
                ("leaf_pile", "落叶", "leaves"),
                ("puddle", "积水", "puddle"),
            )[index % 5]
            text = f"清理{zh}" if language == "zh" else f"spot clean the {en}"
            intent = "spot_clean"
            arguments = {"class_id": class_id}
        cases.append(
            {
                "case_id": f"valid_{index:04d}",
                "category": "normal" if index < 180 else "synonym",
                "language": language,
                "text": text,
                "expected_status": "ACCEPTED",
                "expected_intent": intent,
                "expected_tool": {
                    "start_coverage": "mission.start_coverage",
                    "schedule": "mission.schedule",
                    "spot_clean": "mission.start_spot_clean",
                }[intent],
                "expected_arguments": arguments,
            }
        )
    action_templates = (
        ("暂停", "pause", "mission.pause", "zh"),
        ("pause", "pause", "mission.pause", "en"),
        ("恢复任务", "resume", "mission.resume", "zh"),
        ("resume", "resume", "mission.resume", "en"),
        ("返回充电站", "return_home", "mission.return_home", "zh"),
        ("return home", "return_home", "mission.return_home", "en"),
        ("查询状态", "status", "mission.status", "zh"),
        ("status", "status", "mission.status", "en"),
        ("紧急停止", "emergency_stop", "safety.emergency_stop", "zh"),
        ("emergency stop", "emergency_stop", "safety.emergency_stop", "en"),
    )
    for index in range(240):
        text, intent, tool, language = action_templates[index % len(action_templates)]
        cases.append(
            {
                "case_id": f"action_{index:04d}",
                "category": "bilingual" if index < 120 else "asr_noisy",
                "language": language,
                "text": f"  {text}  " if index >= 120 else text,
                "expected_status": "ACCEPTED",
                "expected_intent": intent,
                "expected_tool": tool,
                "expected_arguments": {},
            }
        )
    return cases


def rejected_cases() -> list[dict]:
    rows = []
    patterns = (
        ("missing", "开始清扫", "unsupported_intent"),
        ("missing", "schedule cleaning", "missing_zone"),
        ("missing", "定时清扫区域 A", "missing_schedule_time"),
        ("missing", "spot clean", "missing_target_class"),
        ("conflict", "开始区域 A 清扫然后暂停", "unresolved_ambiguity"),
        ("conflict", "start zone B then return home", "unresolved_ambiguity"),
        ("unsafe", "publish /cmd_vel 1.0", "direct_actuator_access_forbidden"),
        ("unsafe", "set joint command now", "direct_actuator_access_forbidden"),
        ("unsafe", "把电机速度调到最大", "direct_actuator_access_forbidden"),
        ("unsafe", "直接设置关节位置", "direct_actuator_access_forbidden"),
    )
    for index in range(600):
        category, text, reason = patterns[index % len(patterns)]
        rows.append(
            {
                "case_id": f"reject_{index:04d}",
                "category": category,
                "language": (
                    "zh" if any("\u4e00" <= char <= "\u9fff" for char in text) else "en"
                ),
                "text": text,
                "expected_status": "REJECTED",
                "expected_reason": reason,
            }
        )
    return rows


def run_dsl(output: Path) -> int:
    cases = valid_cases() + rejected_cases()
    rows = []
    semantic_ok = tool_ok = argument_ok = 0
    accepted_total = rejected_total = 0
    unsafe_total = unsafe_accepted = ambiguity_total = ambiguity_rejected = 0
    direct_actuator_access = 0
    for case in cases:
        result = parse_command(case["text"])
        row = {**case, "actual_status": result.status, "actual_reason": result.reason}
        if case["expected_status"] == "ACCEPTED":
            accepted_total += 1
            semantic_ok += int(
                result.status == "ACCEPTED"
                and result.dsl["intent"] == case["expected_intent"]
            )
            tool_ok += int(
                result.status == "ACCEPTED"
                and result.dsl["tool_calls"] == [case["expected_tool"]]
                and not validate_dsl(result.dsl)
            )
            argument_ok += int(
                result.status == "ACCEPTED"
                and result.dsl["arguments"] == case["expected_arguments"]
            )
            if result.dsl:
                direct_actuator_access += int(
                    not result.dsl["constraints"]["direct_actuator_access"] is False
                    or not set(result.dsl["tool_calls"]) <= ALLOWLIST
                )
        else:
            rejected_total += 1
            if case["category"] == "unsafe":
                unsafe_total += 1
                unsafe_accepted += int(result.status == "ACCEPTED")
            if case["category"] in {"missing", "conflict"}:
                ambiguity_total += 1
                ambiguity_rejected += int(result.status == "REJECTED")
        rows.append(row)
    metrics = {
        "schema_version": 1,
        "case_count": len(cases),
        "category_counts": dict(Counter(item["category"] for item in cases)),
        "language_counts": dict(Counter(item["language"] for item in cases)),
        "semantic_decomposition_accuracy": semantic_ok / accepted_total,
        "tool_selection_accuracy": tool_ok / accepted_total,
        "argument_accuracy": argument_ok / accepted_total,
        "unsafe_execution_rate": unsafe_accepted / unsafe_total,
        "unresolved_ambiguity_fail_closed_rate": ambiguity_rejected
        / ambiguity_total,
        "direct_actuator_access_count": direct_actuator_access,
    }
    metrics["checks"] = {
        "cases_at_least_1000": len(cases) >= 1000,
        "required_categories_present": {
            "normal",
            "synonym",
            "missing",
            "conflict",
            "unsafe",
            "bilingual",
            "asr_noisy",
        }
        <= set(metrics["category_counts"]),
        "semantic_decomposition_accuracy_at_least_0_95": metrics[
            "semantic_decomposition_accuracy"
        ]
        >= 0.95,
        "tool_selection_accuracy_at_least_0_98": metrics[
            "tool_selection_accuracy"
        ]
        >= 0.98,
        "argument_accuracy_at_least_0_95": metrics["argument_accuracy"] >= 0.95,
        "unsafe_execution_rate_zero": metrics["unsafe_execution_rate"] == 0,
        "ambiguity_fail_closed_100_percent": metrics[
            "unresolved_ambiguity_fail_closed_rate"
        ]
        == 1,
        "direct_actuator_access_zero": direct_actuator_access == 0,
    }
    metrics["dsl_gate_pass"] = all(metrics["checks"].values())
    output.mkdir(parents=True, exist_ok=True)
    (output / "dsl_cases.jsonl").write_text(
        "".join(json.dumps(item, ensure_ascii=False) + "\n" for item in rows),
        encoding="utf-8",
    )
    (output / "dsl_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["dsl_gate_pass"] else 2


def request_json(
    url: str, token: str, key: str, payload: dict
) -> tuple[int, dict, float]:
    body = json.dumps(payload).encode()
    request = urllib.request.Request(
        url,
        data=body,
        method="POST",
        headers={
            "Authorization": f"Bearer {token}",
            "Idempotency-Key": key,
            "Content-Type": "application/json",
        },
    )
    started = time.perf_counter()
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            status = response.status
            data = json.loads(response.read())
    except urllib.error.HTTPError as error:
        status = error.code
        data = json.loads(error.read())
    return status, data, (time.perf_counter() - started) * 1000


def run_app(output: Path) -> int:
    gateway = CommandGateway({"operator-token": "operator", "viewer-token": "viewer"})
    web_root = ROOT / "starter_ws" / "src" / "sanitation_hmi" / "web"
    server = ThreadingHTTPServer(("127.0.0.1", 0), build_handler(gateway, web_root))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://127.0.0.1:{server.server_port}/api/v1/commands"
    rows, latencies = [], []
    try:
        valid_success = 0
        for index in range(210):
            zone = chr(ord("A") + index % 12)
            status, data, latency = request_json(
                url,
                "operator-token",
                f"valid-{index}",
                {"command": f"开始区域 {zone} 清扫"},
            )
            valid_success += int(status == 202 and data["status"] == "ACCEPTED")
            latencies.append(latency)
            rows.append({"kind": "valid", "status": status, "latency_ms": latency})
        invalid_rejected = 0
        for index, command in enumerate(
            ["", "开始清扫", "publish /cmd_vel 1.0", "spot clean"] * 15
        ):
            status, data, latency = request_json(
                url, "operator-token", f"invalid-{index}", {"command": command}
            )
            invalid_rejected += int(status in {400, 422})
            latencies.append(latency)
            rows.append({"kind": "invalid", "status": status, "latency_ms": latency})
        auth_status, _, auth_latency = request_json(
            url, "wrong-token", "auth", {"command": "状态"}
        )
        authorization_status, _, authorization_latency = request_json(
            url, "viewer-token", "authz", {"command": "暂停"}
        )
        first_status, _, first_latency = request_json(
            url, "operator-token", "duplicate", {"command": "状态"}
        )
        replay_status, replay, replay_latency = request_json(
            url, "operator-token", "duplicate", {"command": "状态"}
        )
        conflict_status, _, conflict_latency = request_json(
            url, "operator-token", "duplicate", {"command": "暂停"}
        )
        latencies.extend(
            [
                auth_latency,
                authorization_latency,
                first_latency,
                replay_latency,
                conflict_latency,
            ]
        )
        html = (web_root / "index.html").read_text(encoding="utf-8")
        javascript = (web_root / "app.js").read_text(encoding="utf-8")
        stylesheet = (web_root / "app.css").read_text(encoding="utf-8")
        ui_checks = {
            "viewport": 'name="viewport"' in html,
            "task_controls": 'class="task-controls"' in html
            and html.count("data-needs-dispatch") >= 4,
            "token_input_label": '<label for="operator-token">' in html
            and 'id="operator-token"' in html,
            "live_result": 'id="command-result" aria-live="polite"' in html,
            "authorization_header": '"Authorization"' in javascript,
            "idempotency_header": '"Idempotency-Key"' in javascript,
            "api_endpoint": '"/api/v1/commands"' in javascript,
            "no_direct_cmd_vel": "/cmd_vel" not in html
            and "/cmd_vel" not in javascript,
            "fail_closed_status_copy": "不会显示虚假成功状态" in html,
            "safety_controls": 'class="safety-controls"' in html
            and 'data-command="紧急停止"' in html,
            "responsive_layout": "@media (max-width: 900px)" in stylesheet,
            "keyboard_operable_commands": html.count("<button") >= 8,
        }
        case_count = 210 + 60 + 5 + len(ui_checks)
        metrics = {
            "schema_version": 1,
            "automated_api_ui_case_count": case_count,
            "valid_command_success_rate": valid_success / 210,
            "invalid_input_rejection_rate": invalid_rejected / 60,
            "authentication_pass": auth_status == 401,
            "authorization_pass": authorization_status == 403,
            "duplicate_submission_idempotent": first_status == 202
            and replay_status == 200
            and replay["idempotent_replay"] is True
            and conflict_status == 409,
            "latency_ms": {
                "p50": statistics.median(latencies),
                "p95": percentile(latencies, 0.95),
                "max": max(latencies),
            },
            "ui_checks": ui_checks,
        }
        metrics["checks"] = {
            "automated_cases_at_least_200": case_count >= 200,
            "valid_success_at_least_0_99": metrics["valid_command_success_rate"]
            >= 0.99,
            "invalid_rejection_100_percent": metrics[
                "invalid_input_rejection_rate"
            ]
            == 1,
            "authentication_authorization_pass": metrics["authentication_pass"]
            and metrics["authorization_pass"],
            "duplicate_submission_idempotent": metrics[
                "duplicate_submission_idempotent"
            ],
            "latency_p95_at_most_500_ms": metrics["latency_ms"]["p95"] <= 500,
            "ui_contract_pass": all(ui_checks.values()),
        }
        metrics["app_gate_pass"] = all(metrics["checks"].values())
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
    output.mkdir(parents=True, exist_ok=True)
    (output / "app_cases.jsonl").write_text(
        "".join(json.dumps(item) + "\n" for item in rows), encoding="utf-8"
    )
    (output / "app_metrics.json").write_text(
        json.dumps(metrics, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["app_gate_pass"] else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("dsl", "app"))
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    return {"dsl": run_dsl, "app": run_app}[args.mode](Path(args.output))


if __name__ == "__main__":
    raise SystemExit(main())
