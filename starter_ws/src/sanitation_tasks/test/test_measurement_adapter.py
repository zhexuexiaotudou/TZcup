from sanitation_tasks.measurement_adapter import diagonal_covariance


def test_diagonal_covariance_is_nonzero_only_on_requested_diagonal():
    covariance = diagonal_covariance(3, [1.0, 2.0, 3.0])
    assert covariance == [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 3.0]
