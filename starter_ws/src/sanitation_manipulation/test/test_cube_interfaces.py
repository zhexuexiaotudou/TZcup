from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
INTERFACES = ROOT / "sanitation_perception_interfaces"


def test_cube_ros_contracts_are_generated_and_truth_free():
    cmake = (INTERFACES / "CMakeLists.txt").read_text(encoding="utf-8")
    expected = (
        "msg/CubeGraspCandidate.msg",
        "msg/CubeTargetState.msg",
        "msg/GraspVerification.msg",
        "action/PickCube.action",
    )
    for relative in expected:
        assert f'"{relative}"' in cmake
        text = (INTERFACES / relative).read_text(encoding="utf-8")
        assert "ground_truth" not in text.lower()
    action = (INTERFACES / "action" / "PickCube.action").read_text(encoding="utf-8")
    assert action.count("\n---\n") == 2
    assert "estimated_cube_pose" in action
    assert "placed_in_bin" in action
