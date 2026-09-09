from journey6_hil_gateway.placement import audit_pc_nodes


def test_sensor_plant_and_evaluator_nodes_are_allowed_on_pc():
    report = audit_pc_nodes(
        [
            "/journey6_hil_gateway",
            "/gazebo",
            "/camera_bridge",
            "/independent_evaluator",
            "/actuator_bridge",
        ]
    )
    assert report["placement_gate_pass"] is True
    assert report["pc_duplicate_algorithm_nodes"] == 0


def test_duplicate_algorithm_and_oracle_nodes_are_rejected():
    report = audit_pc_nodes(
        [
            "/planner_server",
            "/coverage_server",
            "/product_perception_node",
            "/garbage_oracle",
        ]
    )
    assert report["placement_gate_pass"] is False
    assert report["pc_duplicate_algorithm_nodes"] == 4
    assert {row["category"] for row in report["violations"]} == {
        "planning_and_navigation",
        "coverage_and_cleaning_intelligence",
        "product_perception",
        "oracle_or_ground_truth",
    }


def test_j6_namespaced_algorithm_nodes_are_remote_not_pc_duplicates():
    report = audit_pc_nodes(["/j6/planner_server", "/j6/product_perception_node"])
    assert report["placement_gate_pass"] is True
    assert report["remote_j6_nodes"] == [
        "/j6/planner_server",
        "/j6/product_perception_node",
    ]
