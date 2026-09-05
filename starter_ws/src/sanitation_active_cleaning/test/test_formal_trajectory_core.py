import math

from sanitation_active_cleaning.formal_trajectory_core import (
    FormalTrajectoryGate,
    PathPose,
)


def _pose(x, y, *, frame="map", quaternion=(0.0, 0.0, 0.0, 1.0)):
    return PathPose(x=x, y=y, quaternion=quaternion, frame_id=frame)


def _gate():
    return FormalTrajectoryGate(
        frame_id="map",
        outer_polygon=((0, 0), (4, 0), (4, 4), (0, 4)),
        keepout_polygons=(((2, 2), (3, 2), (3, 3), (2, 3)),),
        max_segment_length=0.6,
        max_path_length=10.0,
        max_pose_count=100,
    )


def test_valid_dense_map_path_is_accepted():
    poses = [_pose(0.5 + index * 0.5, 1.0) for index in range(6)]
    decision = _gate().validate(path_frame_id="map", poses=poses)
    assert decision.accepted is True
    assert decision.reason == "accepted"
    assert decision.path_length == 2.5


def test_nonfinite_wrong_frame_boundary_and_bad_quaternion_fail_closed():
    gate = _gate()
    assert gate.validate(path_frame_id="odom", poses=[_pose(1, 1), _pose(1.5, 1)]).reason == "path_frame_mismatch"
    assert gate.validate(path_frame_id="map", poses=[_pose(1, 1), _pose(math.nan, 1)]).reason == "non_finite_pose"
    assert gate.validate(path_frame_id="map", poses=[_pose(1, 1), _pose(1.5, 1, frame="odom")]).reason == "pose_frame_mismatch"
    assert gate.validate(path_frame_id="map", poses=[_pose(1, 1), _pose(1.5, 1, quaternion=(0, 0, 0, 0))]).reason == "invalid_quaternion"
    assert gate.validate(path_frame_id="map", poses=[_pose(0, 1), _pose(0.5, 1)]).reason == "outside_geofence"


def test_sparse_duplicate_keepout_and_length_violations_fail_closed():
    gate = _gate()
    assert gate.validate(path_frame_id="map", poses=[_pose(1, 1), _pose(1, 1)]).reason == "duplicate_consecutive_pose"
    assert gate.validate(path_frame_id="map", poses=[_pose(1, 1), _pose(1.7, 1)]).reason == "segment_too_long"
    assert gate.validate(path_frame_id="map", poses=[_pose(1.8, 2.5), _pose(2.2, 2.5)]).reason == "inside_keepout"


def test_segment_cannot_jump_through_keepout_even_when_endpoints_are_free():
    gate = FormalTrajectoryGate(
        frame_id="map",
        outer_polygon=((0, 0), (4, 0), (4, 4), (0, 4)),
        keepout_polygons=(((2, 2), (3, 2), (3, 3), (2, 3)),),
        max_segment_length=5.0,
        max_path_length=10.0,
        max_pose_count=100,
    )
    decision = gate.validate(
        path_frame_id="map", poses=[_pose(1.5, 2.5), _pose(3.5, 2.5)]
    )
    assert decision.accepted is False
    assert decision.reason == "segment_crosses_keepout"
