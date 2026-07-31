from sanitation_hmi.gateway import CommandGateway


def test_auth_authorization_idempotency_and_validation():
    gateway = CommandGateway({"operator": "operator", "viewer": "viewer"})
    status, _ = gateway.submit("", "one", {"command": "状态"})
    assert status == 401
    status, _ = gateway.submit("operator", "", {"command": "状态"})
    assert status == 400
    status, _ = gateway.submit("operator", "bad-schema", {"text": "状态"})
    assert status == 400
    status, _ = gateway.submit("viewer", "viewer-denied", {"command": "暂停"})
    assert status == 403
    status, response = gateway.submit(
        "operator", "same", {"command": "开始区域 A 清扫"}
    )
    assert status == 202
    assert response["execution_dispatched"] is False
    status, replay = gateway.submit(
        "operator", "same", {"command": "开始区域 A 清扫"}
    )
    assert status == 200
    assert replay["idempotent_replay"] is True
    status, conflict = gateway.submit(
        "operator", "same", {"command": "开始区域 B 清扫"}
    )
    assert status == 409
    assert conflict["reason"] == "idempotency_conflict"


def test_dispatcher_can_fail_closed_or_confirm_execution():
    def dispatcher(dsl):
        if dsl["intent"] == "emergency_stop":
            return {"accepted": True, "dispatched": True, "reason": None}
        return {
            "accepted": False,
            "dispatched": False,
            "reason": "safe_task_orchestrator_unavailable",
        }

    gateway = CommandGateway({"operator": "operator"}, dispatcher=dispatcher)
    status, rejected = gateway.submit(
        "operator", "coverage", {"command": "开始区域 A 清扫"}
    )
    assert status == 503
    assert rejected["reason"] == "safe_task_orchestrator_unavailable"
    status, accepted = gateway.submit(
        "operator", "estop", {"command": "紧急停止"}
    )
    assert status == 202
    assert accepted["execution_dispatched"] is True
