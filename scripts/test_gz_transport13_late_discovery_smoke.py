import shutil
import subprocess
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
RUNNER = ROOT / "scripts/run_gz_transport13_late_discovery_smoke.sh"
CMAKE = ROOT / "scripts/gz_transport13_late_discovery_smoke/CMakeLists.txt"
SOURCE = ROOT / "scripts/gz_transport13_late_discovery_smoke/main.cc"
TARGET_FIXTURE = ROOT / "scripts/gz_transport13_late_discovery_smoke/test_target_contract"


def test_runner_is_transport_only_and_uses_one_generated_partition() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert "never launches Gazebo, ROS 2, or Docker" in source
    assert "\nsource /opt/ros/jazzy/setup.bash" not in source
    assert "GZ_PARTITION=\"${partition}\"" in source
    assert "GZ_IP=127.0.0.1" in source
    assert "run_case publisher_first publisher" in source
    assert "run_case subscriber_first subscriber" in source
    assert "publish_count=40" in source
    assert "minimum_post_discovery_count=3" in source
    assert "refusing stale --output-dir" in source
    assert "gz_cmake_vendor" in source
    assert "gz_utils_vendor" in source
    assert "gz_msgs_vendor" in source
    assert "gz_math_vendor" in source
    assert "do not add gz_transport_vendor" in source
    assert '-DCMAKE_PREFIX_PATH="${cmake_prefix_path}"' in source
    assert "audit_gz_transport13_late_discovery_dependencies.py" in source
    assert "dependency-closure.json" in source


def test_runner_fails_closed_on_frozen_runtime_binding_and_live_maps() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert "libgz-transport13.so.13.5.0" in source
    assert "frozen transport aliases are not byte-identical" in source
    assert "smoke binary does not resolve gz-transport13 from frozen runtime" in source
    assert '"/proc/${pid}/maps"' in source
    assert "mapped transport hash differs from frozen alias" in source
    assert "transport-process-binding.tsv" in source
    assert "GZ_TRANSPORT13_LATE_DISCOVERY_SMOKE_PASSED" in source


def test_runner_has_closed_runtime_loader_contract_for_audited_vendor_libraries() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    runtime_block = source.split("runtime_vendor_library_prefixes=(", 1)[1].split(")", 1)[0]

    assert "gz_utils_vendor" in runtime_block
    assert "gz_msgs_vendor" in runtime_block
    assert "gz_math_vendor" in runtime_block
    assert "gz_cmake_vendor" not in runtime_block
    assert 'formal_dynamic_dependencies.sh" "${binary}"' in source
    assert "not found" in source
    assert 'assert_ldd_library "libgz-utils2.so.2" "/opt/ros/jazzy/opt/gz_utils_vendor/lib"' in source
    assert 'assert_ldd_library "libgz-msgs10.so.10" "/opt/ros/jazzy/opt/gz_msgs_vendor/lib"' in source
    assert 'assert_ldd_library "libgz-math7.so.7"' not in source
    assert "does not have" in source
    assert 'LD_LIBRARY_PATH="${runtime_library_path}"' in source
    assert "runtime-library-path-contract.txt" in source
    assert "runtime-library-bindings.tsv" in source


def test_endpoint_source_records_topic_info_and_distinct_receive_counts() -> None:
    source = SOURCE.read_text(encoding="utf-8")
    assert "node.TopicInfo(topic, publishers)" in source
    assert "unique_received_count" in source
    assert "max_consecutive_sequence_count" in source
    assert "MaxConsecutiveSequenceCount" in source
    assert "numericSequenceReceiveOrder" in source
    assert "publisher.Publish(message)" in source
    assert "node.Subscribe<gz::msgs::StringMsg>" in source
    assert "receivedCount.load() >= options.count" in source
    assert "#include <gz/transport/Node.hh>" in source


def test_late_publisher_case_requires_post_discovery_suffix_not_replay() -> None:
    source = RUNNER.read_text(encoding="utf-8")
    assert 'subscriber["received_count"] >= minimum_post_discovery_count' in source
    assert (
        'subscriber["max_consecutive_sequence_count"] '
        '>= minimum_post_discovery_count'
    ) in source
    assert 'subscriber["received_count"] >= publish_count' not in source


def test_cmake_requires_frozen_transport_target_before_building() -> None:
    source = CMAKE.read_text(encoding="utf-8")
    assert "TZCUP_FROZEN_RUNTIME_PREFIX is required" in source
    assert "find_package(gz-transport13 REQUIRED CONFIG)" in source
    assert "gz-transport13 config escaped frozen runtime" in source
    assert "gz-transport13 did not resolve to the frozen runtime" in source
    assert "BUILD_RPATH" in source


def test_concrete_shared_target_fixture_rejects_extra_transport_target(
    tmp_path: Path,
) -> None:
    cmake = shutil.which("cmake")
    if cmake is None:
        pytest.skip("CMake is unavailable on the low-memory Windows host")
    passing = subprocess.run(
        [cmake, "-S", str(TARGET_FIXTURE), "-B", str(tmp_path / "passing")],
        capture_output=True,
        text=True,
        check=False,
    )
    assert passing.returncode == 0, passing.stderr
    assert (tmp_path / "passing/resolved.txt").read_text(encoding="utf-8").endswith(
        "libgz-transport13.so.13\n"
    )
    rejected = subprocess.run(
        [
            cmake,
            "-S",
            str(TARGET_FIXTURE),
            "-B",
            str(tmp_path / "rejected"),
            "-DTZCUP_FIXTURE_EXTRA_TRANSPORT_LINK=ON",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert rejected.returncode != 0
    assert "must link only the concrete frozen shared target" in rejected.stderr
