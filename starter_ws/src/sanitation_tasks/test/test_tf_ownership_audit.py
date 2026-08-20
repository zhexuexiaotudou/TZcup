from sanitation_tasks.tf_ownership_audit import summarize_ownership


def test_tf_owner_audit_accepts_exactly_one_map_to_odom_owner():
    observations = [
        {'publisher_gid': 'aa', 'child_frame_id': 'odom'},
        {'publisher_gid': 'aa', 'child_frame_id': 'odom'},
        {'publisher_gid': 'bb', 'child_frame_id': 'base_link'},
    ]
    endpoints = {
        'aa': {'node_name': 'amcl', 'node_namespace': '/'},
        'bb': {'node_name': 'ekf_filter_node', 'node_namespace': '/'},
    }
    report = summarize_ownership(
        observations, expected_owner_node='amcl', endpoint_names=endpoints
    )
    assert report['single_owner'] is True
    assert report['owner_count'] == 1


def test_tf_owner_audit_rejects_a_forbidden_global_owner_endpoint():
    observations = [
        {'publisher_gid': 'aa', 'child_frame_id': 'odom'},
        {'publisher_gid': 'cc', 'child_frame_id': 'odom'},
    ]
    endpoints = {
        'aa': {'node_name': 'slam_toolbox', 'node_namespace': '/'},
        'cc': {'node_name': 'amcl', 'node_namespace': '/'},
    }
    report = summarize_ownership(
        observations,
        expected_owner_node='slam_toolbox',
        endpoint_names=endpoints,
        forbidden_owner_nodes=['amcl'],
    )
    assert report['single_owner'] is False
    assert report['owner_count'] == 2
    assert report['forbidden_owner_nodes_present'] == ['amcl']


def test_tf_owner_audit_rejects_unattributed_matching_publisher():
    observations = [{'publisher_gid': 'missing', 'child_frame_id': 'odom'}]
    report = summarize_ownership(
        observations,
        expected_owner_node='amcl',
        endpoint_names={'aa': {'node_name': 'amcl', 'node_namespace': '/'}},
    )
    assert report['single_owner'] is False
    assert report['unattributed_publisher_gids'] == ['missing']


def test_tf_owner_audit_reports_explicit_fallback_when_rmw_gid_is_empty():
    observations = [{'publisher_gid': '', 'child_frame_id': 'odom'}]
    report = summarize_ownership(
        observations,
        expected_owner_node='amcl',
        endpoint_names={'aa': {'node_name': 'amcl', 'node_namespace': '/'}},
    )
    assert report['single_owner'] is True
    assert report['publisher_gid_available'] is False
    assert report['owner_nodes'] == ['amcl']
    assert report['attribution_method'].startswith('configured_owner_plus_')
