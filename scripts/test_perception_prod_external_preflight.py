from perception_prod_external_preflight import build_field_status, build_toolchain_lock


def test_toolchain_lock_does_not_promote_historical_package_audit():
    discovery = {
        "official_source": {
            "oe_version": "3.7.0", "archive_sha256": "a" * 64,
            "archive_bytes": 123, "archive_integrity_pass": True,
        },
        "required_versions": {"hbdk4_compiler": "4.7.5", "hmct": "2.6.5"},
    }
    lock = build_toolchain_lock(discovery, official_docs_version="3.9.0")
    assert lock["version_compatibility_resolved"] is False
    assert lock["installation_root"] is None
    assert lock["PRODUCT_J6_TOOLCHAIN_READY"] is False
    assert lock["J6_MODEL_BLOCKED_INTERNAL"] is True


def test_field_status_requires_real_rgbd_recording_and_independent_gt():
    sensor = {
        "detected_camera_summary": "Integrated Camera only",
        "rgbd_device_present": False,
        "auditable_rgbd_recording_present": False,
        "independent_map_gt_present": False,
    }
    field = build_field_status(sensor, {"all_required_software_present": True})
    assert field["software_preparation_complete"] is True
    assert field["Integrated_Camera_accepted_as_RGBD"] is False
    assert field["REAL_DOMAIN_BLOCKED_EXTERNAL"] is True
    assert field["PRODUCT_FIELD_READY"] is False
    assert all(value is None for value in field["metrics"].values())
