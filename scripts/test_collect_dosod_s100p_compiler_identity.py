from __future__ import annotations

from collect_dosod_s100p_compiler_identity import (
    EXPECTED_ARCHIVE_SHA256,
    EXPECTED_OE_VERSION,
    EXPECTED_VERSIONS,
    evaluate_identity,
)


def _discovery() -> dict:
    return {
        "official_source": {
            "oe_version": EXPECTED_OE_VERSION,
            "archive_sha256": EXPECTED_ARCHIVE_SHA256,
        },
        "required_versions": EXPECTED_VERSIONS,
        "official_toolchain_package_ready": True,
    }


def test_exact_live_compiler_identity_passes_without_claiming_hbm() -> None:
    report = evaluate_identity(
        system="Linux",
        machine="x86_64",
        versions=EXPECTED_VERSIONS,
        executable_path="/venv/bin/hb_compile",
        executable_sha256="a" * 64,
        probe_returncode=0,
        probe_output=b"Usage: hb_compile --config PATH --march nash-m",
        discovery=_discovery(),
    )
    assert report["identity_verified"] is True
    assert report["blockers"] == []
    assert report["compile_executed"] is False
    assert report["hbm_status"] == "HBM_NOT_PRODUCED"


def test_windows_or_version_drift_is_blocked() -> None:
    versions = dict(EXPECTED_VERSIONS)
    versions["hmct"] = "9.9.9"
    report = evaluate_identity(
        system="Windows",
        machine="AMD64",
        versions=versions,
        executable_path="C:/fake/hb_compile.exe",
        executable_sha256="b" * 64,
        probe_returncode=0,
        probe_output=b"Usage: hb_compile --config PATH --march nash-m",
        discovery=_discovery(),
    )
    assert report["identity_verified"] is False
    assert report["blockers"] == [
        "compiler_host_not_linux",
        "compiler_package_versions_mismatch",
    ]


def test_probe_must_expose_nash_m_and_config() -> None:
    report = evaluate_identity(
        system="Linux",
        machine="x86_64",
        versions=EXPECTED_VERSIONS,
        executable_path="/venv/bin/hb_compile",
        executable_sha256="c" * 64,
        probe_returncode=0,
        probe_output=b"Usage: hb_compile",
        discovery=_discovery(),
    )
    assert report["identity_verified"] is False
    assert "hb_compile_probe_contract_missing" in report["blockers"]
