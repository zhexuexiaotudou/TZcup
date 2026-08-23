from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).resolve().parents[4] / "scripts" / "j6_hil_network_faults.py"
SPEC = spec_from_file_location("j6_hil_network_faults", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC and SPEC.loader
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_fault_profiles_are_bounded_to_requested_interface():
    plan = MODULE.build_fault_plan("delay", "eth0")
    assert plan.command[:6] == ("tc", "qdisc", "replace", "dev", "eth0", "root")
    assert "120ms" in plan.command
    dry_run = MODULE.execute_plan(plan, apply=False)
    assert dry_run["applied"] is False
    assert dry_run["returncode"] is None


def test_disconnect_and_restore_are_explicit_and_interface_is_validated():
    assert "100%" in MODULE.build_fault_plan("disconnect").command
    assert MODULE.build_fault_plan("normal").command[2] == "del"
    with pytest.raises(ValueError, match="invalid network interface"):
        MODULE.build_fault_plan("loss", "eth0; reboot")
