import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("collect_s100p_g0_inventory.py")
SPEC = importlib.util.spec_from_file_location("collect_s100p_g0_inventory", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def test_inventory_is_explicitly_read_only(monkeypatch):
    monkeypatch.setattr(MODULE, "glob", type("G", (), {"glob": staticmethod(lambda _p: [])}))
    monkeypatch.setattr(MODULE, "_read_text", lambda _p: {"status": "PRESENT", "value": "x"})
    monkeypatch.setattr(MODULE, "_run", lambda command: {"status": "OK", "command": command})

    monkeypatch.setattr(MODULE, "_path_hash", lambda path: {"path": path, "status": "ABSENT"})
    monkeypatch.setattr(MODULE.shutil, "which", lambda _tool: None)
    report = MODULE.collect(overlay_paths=["/overlay"], model_paths=["/model"])

    assert report["report_id"] == "tzcup_s100p_g0_read_only_software_bpu_ros_inventory_v2"
    assert report["safety"]["actuator_commands_sent"] is False
    assert report["safety"]["privileged_commands_used"] is False
    assert report["safety"]["ros_publish_or_service_calls_sent"] is False
    assert report["safety"]["can_gpio_or_actuator_access_attempted"] is False
    assert report["ros_graph_read_only"]["ros2cli_daemon_disabled"] is True
    assert report["bpu_runtime_and_tools"]["hb_runtime"]["status"] == "ABSENT"


def test_command_allowlist_has_no_mutating_tools():
    forbidden = {"ros2", "candump", "cansend", "ipmitool", "devmem", "gpio", "sudo"}
    executables = {command[0] for command in MODULE.READ_ONLY_COMMANDS.values()}
    assert forbidden.isdisjoint(executables)
    assert MODULE.ROS_READ_ONLY_ARGUMENTS == {
        "packages": ("pkg", "list"),
        "nodes": ("node", "list", "--no-daemon"),
        "topics_with_types": ("topic", "list", "-t", "--no-daemon"),
    }


def test_ros_cli_read_only_probes_disable_the_daemon(monkeypatch):
    captured = {}

    class Result:
        returncode = 0
        stdout = ""
        stderr = ""

    def fake_run(command, **kwargs):
        captured["command"] = command
        captured["environment"] = kwargs["env"]
        return Result()

    monkeypatch.setattr(MODULE.subprocess, "run", fake_run)
    result = MODULE._run(["/bin/bash", "-lc", "source /opt/tros/humble/setup.bash; exec ros2 topic list -t --no-daemon"])
    assert result["status"] == "OK"
    assert "--no-daemon" in captured["command"][-1]
    assert captured["environment"]["ROS2CLI_DISABLE_DAEMON"] == "1"


def test_sourced_ros_rejects_non_allowlisted_setup_or_ros_subcommand(tmp_path):
    setup = tmp_path / "setup.bash"
    setup.write_text("", encoding="utf-8")
    try:
        MODULE._run_sourced_ros(setup, ("topic", "pub", "/cmd_vel"))
    except RuntimeError as error:
        assert "allowlisted" in str(error) or "read-only" in str(error)
    else:
        raise AssertionError("non-allowlisted setup or ros2 publish was accepted")


def test_model_inventory_only_hashes_explicit_files_or_direct_model_directory_entries(tmp_path):
    model = tmp_path / "dosod.hbm"
    model.write_bytes(b"model-bytes")
    ignored = tmp_path / "nested"
    ignored.mkdir()
    (ignored / "not_scanned.hbm").write_bytes(b"nested")
    rows = MODULE._model_files([str(model), str(tmp_path)])
    present = [row for row in rows if row["status"] == "PRESENT"]
    assert len(present) == 1
    assert present[0]["path"] == str(model.resolve())
    assert len(present[0]["sha256"]) == 64
