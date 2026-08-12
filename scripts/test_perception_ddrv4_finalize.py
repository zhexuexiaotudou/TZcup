import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).with_name("perception_ddrv4_finalize.py")


def module():
    spec = importlib.util.spec_from_file_location("ddrv4_finalize", SCRIPT)
    assert spec is not None and spec.loader is not None
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def test_final_status_locks_every_downstream_gate_after_online_failure():
    loaded = module()
    status = loaded.build_status(
        {"DDRV4_D1_PASS": True},
        {"DDRV4_X86_DEV_PASS": False},
        {"pass": False},
        {"PRODUCT_J6_TOOLCHAIN_READY": False},
        {"PRODUCT_J6_BOARD_READY": False},
        {"PRODUCT_FIELD_READY": False},
        {"commit": "a" * 40},
    )
    assert status["DDRV4_D1_PASS"] is True
    assert status["DDRV4_D2_STATE"] == "NOT_EXECUTED_D1_STATIC_PASSED"
    assert status["MODEL_FREEZE_X86_CREATED"] is False
    assert status["G5_V2_PASS"] is False
    assert status["PRODUCT_X86_PERCEPTION_READY"] is False
    assert status["PR_90_READY_ALLOWED"] is False
    assert status["NEAT_FREAK_SYNC_STATUS"] == "NOT_RUN_PRODUCTION_GATE_NOT_REACHED"
