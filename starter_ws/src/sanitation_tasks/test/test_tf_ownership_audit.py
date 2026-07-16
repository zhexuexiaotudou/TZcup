from sanitation_tasks.tf_ownership_audit import summarize_ownership


def test_tf_owner_audit_accepts_exactly_one_map_to_odom_owner():
    observations = [
        {'publisher_gid': 'aa', 'child_frame_id': 'odom'},
        {'publisher_gid': 'aa', 'child_frame_id': 'odom'},
        {'publisher_gid': 'bb', 'child_frame_id': 'base_link'},
    ]
    report = summarize_ownership(observations, expected_owner_present=True)
    assert report['single_owner'] is True
    assert report['owner_count'] == 1


def test_tf_owner_audit_rejects_a_forbidden_global_owner_endpoint():
    observations = [
        {'publisher_gid': 'aa', 'child_frame_id': 'odom'},
        {'publisher_gid': 'cc', 'child_frame_id': 'odom'},
    ]
    report = summarize_ownership(
        observations, expected_owner_present=True, forbidden_owner_nodes=['amcl']
    )
    assert report['single_owner'] is False
    assert report['owner_count'] == 0
