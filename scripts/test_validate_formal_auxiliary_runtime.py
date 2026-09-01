from __future__ import annotations

from validate_formal_auxiliary_runtime import FAILED_STATUS, PASSED_STATUS, evaluate


def _phase(**updates):
    phase = {
        "battery_soc": 0.5,
        "battery_voltage_v": 27.6,
        "evidence_authority": "SIMULATION_ENGINEERING_ONLY",
        "interface_class": "product_simulation",
        "emergency_stop_active": False,
        "relay_enabled": True,
        "charge_connected": False,
        "branches": {"safety": True, "low_voltage": True, "high_power": True},
        "lighting": {"work": True, "tail": True, "four_corner_warning": True},
        "applied_lighting": {
            "work": True,
            "tail": True,
            "four_corner_warning": True,
        },
        "main_isolator_closed": True,
        "main_isolator_feedback_fresh": True,
        "main_contactor_closed": True,
        "main_contactor_feedback_fresh": True,
        "active_reasons": [],
        "operator_command_fresh": True,
        "bindings": {
            name: {"link": name, "datum": name}
            for name in (
                "work_lights",
                "tail_lights",
                "four_corner_warning_lights",
                "charge_interface",
                "fused_power_distribution",
                "isolated_low_voltage_power",
                "safety_relay",
                "main_power_isolator",
                "main_power_contactor",
                "emergency_stop",
            )
        },
    }
    phase.update(updates)
    return phase


def _passing_phases():
    return {
        "enabled": _phase(),
        "operator_timeout": _phase(
            emergency_stop_active=True,
            relay_enabled=False,
            branches={"safety": True, "low_voltage": False, "high_power": False},
            lighting={"work": False, "tail": False, "four_corner_warning": True},
            applied_lighting={
                "work": False,
                "tail": False,
                "four_corner_warning": True,
            },
            operator_command_fresh=False,
        ),
    }


def test_all_auxiliary_runtime_effects_pass_together():
    report = evaluate(_passing_phases())
    assert report["passed"] is True
    assert report["status"] == PASSED_STATUS
    assert all(report["checks"].values())


def test_missing_physical_binding_fails_closed():
    phases = _passing_phases()
    phases["operator_timeout"]["bindings"].pop("charge_interface")
    report = evaluate(phases)
    assert report["passed"] is False
    assert report["status"] == FAILED_STATUS
    assert not report["checks"]["runtime_bound_to_all_product_positions"]


def test_missing_applied_lighting_fails_closed():
    phases = _passing_phases()
    phases["enabled"]["applied_lighting"]["work"] = False
    report = evaluate(phases)
    assert not report["checks"]["requested_work_tail_warning_lights_on"]
