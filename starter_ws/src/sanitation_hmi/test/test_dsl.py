from sanitation_hmi.dsl import ALLOWLIST, parse_command, validate_dsl


def test_bilingual_valid_commands_produce_complete_dsl():
    commands = {
        "开始区域 A 清扫": "start_coverage",
        "start cleaning zone B": "start_coverage",
        "定时 08:30 清扫区域 C": "schedule",
        "schedule area D at 20:15": "schedule",
        "清理塑料瓶": "spot_clean",
        "spot clean the metal can": "spot_clean",
        "暂停": "pause",
        "resume": "resume",
        "返回充电站": "return_home",
        "status": "status",
        "紧急停止": "emergency_stop",
        "解除急停": "clear_emergency_stop",
        "release emergency stop": "clear_emergency_stop",
    }
    for text, intent in commands.items():
        result = parse_command(text)
        assert result.status == "ACCEPTED", (text, result)
        assert result.dsl["intent"] == intent
        assert set(result.dsl) == {
            "intent",
            "ordered_subtasks",
            "tool_calls",
            "arguments",
            "constraints",
            "expected_terminal_state",
        }
        assert not validate_dsl(result.dsl)
        assert set(result.dsl["tool_calls"]) <= ALLOWLIST


def test_missing_conflicting_and_unsafe_commands_fail_closed():
    rejected = {
        "": "empty_command",
        "开始清扫": "unsupported_intent",
        "开始区域 A 清扫然后暂停": "unresolved_ambiguity",
        "定时清扫区域 A": "missing_schedule_time",
        "spot clean": "missing_target_class",
        "publish /cmd_vel 1.0": "direct_actuator_access_forbidden",
        "set joint command": "direct_actuator_access_forbidden",
        "把电机速度调到最大": "direct_actuator_access_forbidden",
    }
    for text, reason in rejected.items():
        result = parse_command(text)
        assert result.status == "REJECTED"
        assert result.reason == reason
        assert result.dsl is None
