from pathlib import Path

from sanitation_learning.g2_contract import read_production_camera_contract, validate_sim_launch_topics


ROOT = Path(__file__).resolve().parents[2]


def test_extracts_real_production_camera_contract():
    xacro = ROOT / "sanitation_vehicle_description" / "urdf" / "sanitation_vehicle.urdf.xacro"
    launch = ROOT / "sanitation_bringup" / "launch" / "sim.launch.py"
    contract = read_production_camera_contract(xacro)
    assert contract["camera_link"] == "camera_link"
    assert contract["optical_frame"] == "camera_depth_link"
    assert contract["extrinsics"]["xyz_m"] == [0.53, 0.0, 0.22]
    assert contract["extrinsics"]["rpy_rad"] == [0.0, 0.0, 0.0]
    assert contract["native_resolution"] == {"width": 640, "height": 480}
    assert contract["horizontal_fov_rad"] == 1.50098
    assert contract["update_rate_hz"] == 15.0
    validate_sim_launch_topics(launch, contract)
