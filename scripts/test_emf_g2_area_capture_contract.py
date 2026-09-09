import shutil
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
POWERSHELL = ROOT / "scripts" / "run_emf_g2_area_capture_docker.ps1"
CAPTURE = ROOT / "scripts" / "emf_g2_area_capture.sh"


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def test_docker_wrapper_uses_only_explicit_external_writable_mounts():
    source = _text(POWERSHELL)
    assert (
        '[string]$Image = "tzcup/sanitation-jazzy@sha256:'
        '418550f48916d794bc0aff144c60a3b1353d0bb0bb1dcf086cda0ec8e2a5aadc"'
        in source
    )
    assert "[Parameter(Mandatory = $true)]" in source
    assert "linorobot2_description\\package.xml" in source
    assert '"${repo}:/repo:ro"' in source
    assert '"${data}:/data:rw"' in source
    assert '"${runtime}:/runtime:rw"' in source
    assert '"${upstream}:/upstream/linorobot2:ro"' in source
    assert "Assert-OutsideRepository" in source
    assert "bash\", \"/repo/scripts/emf_g2_area_capture.sh" in source
    assert "stage1_20260714_154523" not in source
    assert "/stage5br3" not in source


def test_shell_builds_explicit_upstream_before_project_overlay():
    source = _text(CAPTURE)
    upstream = source.index("--packages-select linorobot2_description")
    upstream_source = source.index(
        'source "${RUNTIME_WS}/upstream_install/setup.bash"'
    )
    project = source.index(
        "--packages-up-to sanitation_learning sanitation_vehicle_description"
    )
    assert upstream < upstream_source < project
    assert '--base-paths "${UPSTREAM_ROOT}"' in source
    assert '--base-paths "${REPO}/starter_ws/src"' in source
    assert 'findmnt -T "${REPO}"' in source
    assert "stage1_20260714_154523" not in source
    assert "/stage5br3" not in source


def test_capture_matrix_is_exactly_four_fixed_nonsealed_missions():
    source = _text(CAPTURE)
    expected = {
        "world_a_asphalt_campus_scene_0001": ("world_a_asphalt_campus", "1", "positive"),
        "world_a_asphalt_campus_scene_0019": ("world_a_asphalt_campus", "19", "negative"),
        "world_d_mixed_curb_vegetation_scene_0003": (
            "world_d_mixed_curb_vegetation",
            "3",
            "positive",
        ),
        "world_d_mixed_curb_vegetation_scene_0038": (
            "world_d_mixed_curb_vegetation",
            "38",
            "negative",
        ),
    }
    for mission, contract in expected.items():
        assert source.count(mission) == 2
        assert all(value in source for value in contract)
    assert source.count("capture_world \\") == 2
    assert "FORBIDDEN_MARKERS=(G5 G5_V2 G5V2 VAL_NEW DEV_VAL SEALED)" in source
    assert "world_e_tiled_plaza" not in source
    assert "world_f_service_road" not in source


def test_capture_uses_real_g2_entrypoints_sensors_and_fail_closed_checks():
    source = _text(CAPTURE)
    for entrypoint in (
        "stage5br2_generate_g2_worlds",
        "stage5br3_randomize_scene",
        "stage5br3_capture_scene",
    ):
        assert entrypoint in source
    assert "enable_training_gt:=true" in source
    for topic in (
        "/camera/color/camera_info",
        "/camera/color/image_raw",
        "/camera/depth/image_rect_raw",
        "/ground_truth/semantic/image",
        "/ground_truth/instance/image",
    ):
        assert topic in source
    assert 'assert report["capture_pass"] is True' in source
    assert 'assert record["exact_four_sensor_timestamp"] is True' in source
    assert 'assert {4, 5}.issubset(observed)' in source
    assert 'observed.issubset({0})' in source
    assert "build_emf_area_dataset.py" in source
    assert '--split-root "TRAIN=${DATA_ROOT}/TRAIN"' in source
    assert '--split-root "HOLDOUT=${DATA_ROOT}/HOLDOUT"' in source


def test_shell_has_valid_bash_syntax():
    bash = shutil.which("bash")
    if bash is None:
        return
    command = [bash, "-n", str(CAPTURE)]
    if Path(bash).suffix.lower() == ".exe":
        wsl = shutil.which("wsl.exe")
        if wsl is None:
            return
        relative = CAPTURE.relative_to(CAPTURE.anchor).as_posix()
        wsl_path = f"/mnt/{CAPTURE.drive[0].lower()}/{relative}"
        command = [wsl, "--", "bash", "-n", wsl_path]
    result = subprocess.run(
        command,
        check=False,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stderr
