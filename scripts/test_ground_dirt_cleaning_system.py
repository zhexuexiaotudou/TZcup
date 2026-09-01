from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
PLUGIN = ROOT / "starter_ws/src/sanitation_gazebo_control/src/GroundDirtCleaningSystem.cc"
XACRO = ROOT / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
LAUNCH = ROOT / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
RUNNER = ROOT / "scripts/run_formal_ground_dirt_cleaning_runtime.sh"
VALIDATOR = ROOT / "scripts/validate_formal_ground_dirt_cleaning_runtime.py"


def test_plugin_uses_real_joint_state_world_pose_and_only_dirt_visual_cells():
    source = PLUGIN.read_text(encoding="utf-8")
    for token in (
        "left_side_brush_joint",
        "right_side_brush_joint",
        "central_roller_joint",
        "cleaning_lift_joint",
        "left_side_brush_link",
        "right_side_brush_link",
        "central_roller_link",
        "components::JointVelocity",
        "components::JointPosition",
        "gz::sim::worldPose",
        'rfind("leaf_", 0)',
        'rfind("dust_mottle_", 0)',
        'rfind("puddle_lobe_", 0)',
        "components::Transparency",
    ):
        assert token in source
    for forbidden in (
        "requestRemoveEntity",
        "RequestRemoveEntity",
        "removeEntity",
        "EntityCreator",
        '"object_',
        '"material_cube',
    ):
        assert forbidden not in source


def test_plugin_wiring_and_product_ros_truth_isolation():
    cmake = (PLUGIN.parent.parent / "CMakeLists.txt").read_text(encoding="utf-8")
    xacro = XACRO.read_text(encoding="utf-8")
    launch = LAUNCH.read_text(encoding="utf-8")
    assert "add_library(GroundDirtCleaningSystem SHARED" in cmake
    assert "libGroundDirtCleaningSystem.so" in xacro
    assert "<cell_area_m2>0.01</cell_area_m2>" in xacro
    assert "<sweep_sample_spacing_m>0.05</sweep_sample_spacing_m>" in xacro
    assert "<minimum_lift_position_m>0.095</minimum_lift_position_m>" in xacro
    assert "lift >= this->minimumLiftPositionM" in PLUGIN.read_text(encoding="utf-8")
    assert "ground_dirt" not in launch


def test_runtime_gate_requires_negative_partial_conservation_and_litter_retention():
    source = VALIDATOR.read_text(encoding="utf-8")
    assert 'REPORT_ID = "tzcup_formal_ground_dirt_physical_cleaning_v1"' in source
    assert 'result["report_id"] = REPORT_ID' in source
    for check in (
        "disabled_gate_removes_zero_area",
        "raised_gate_removes_zero_area",
        "stopped_gate_removes_zero_area",
        "partial_pass_is_strictly_partial",
        "physical_sweep_reaches_95_percent",
        "area_mass_conservation_exact",
        "all_rigid_litter_models_remain",
        "no_task_set_pose_after_start",
    ):
        assert f'"{check}"' in source
    runner = RUNNER.read_text(encoding="utf-8")
    assert "prepare_formal_ground_dirt_runtime.py" in runner
    assert "ground_dirt/status_json" in runner
    assert "validate_formal_ground_dirt_cleaning_runtime.py" in runner
    assert "FORMAL_ACCEPTANCE_SESSION" in runner
    assert "FORMAL_VEHICLE_SNAPSHOT_MANIFEST" in runner
    assert "FORMAL_FINAL_RUNTIME_CLOSURE_MANIFEST" in runner
    assert 'runtime_binding="${output}.runtime_binding.json"' in runner
    assert "generate_formal_vehicle_snapshot.py" in runner
    assert "--check --output \"${snapshot}\"" in runner
    assert runner.index("generate_formal_vehicle_snapshot.py") < runner.index(
        "formal_runtime_gate_binding.py"
    ) < runner.index("ros2 launch")
    assert "--runtime-binding \"${runtime_binding}\"" in runner
    assert "from formal_runtime_gate_binding import load_binding" in source
    assert "def _bound_runtime_evidence(" in source
    assert "session_manifest_sha256" in source
    assert 'Path(str(bound_session.get("session_manifest", ""))).resolve()' in source
    assert 'result["runtime_gate_binding"] = runtime_binding' in source
    assert 'result["acceptance_session_binding"] = acceptance_session_binding' in source
    assert 'result["runtime_closure_binding"] = runtime_binding["runtime_closure_binding"]' in source


def test_prepared_random_episode_has_one_square_metre_cellized_patch_and_20_litter_bodies(tmp_path):
    episode = tmp_path / "episode"
    subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/prepare_formal_ground_dirt_runtime.py"),
            "--output-dir",
            str(episode),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    setup = json.loads((episode / "evaluator/runtime_setup.json").read_text(encoding="utf-8"))
    world = (episode / "public/world.sdf").read_text(encoding="utf-8")
    assert setup["initial_area_m2"] == 1.0
    assert setup["cell_contract"]["total_cell_count"] == 100
    assert len(setup["rigid_litter_ids"]) == 20
    assert sum(
        world.count(f'<visual name="{prefix}')
        for prefix in ("leaf_", "dust_mottle_", "puddle_lobe_")
    ) == 100
    assert all(f'<model name="{name}">' in world for name in setup["rigid_litter_ids"])
