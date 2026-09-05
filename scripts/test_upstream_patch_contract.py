from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PATCH = (
    ROOT
    / "patches"
    / "upstream"
    / "opennav_coverage"
    / "2241180-test-path-fixed-seed.patch"
)
EXPECTED_PATCH_SHA256 = "c101a9bfa3078139566fe8577f63a4cc525bde71d8fb3f244fdc2beb846af0b1"


def test_opennav_path_patch_is_narrow_and_deterministic(tmp_path: Path) -> None:
    assert (ROOT / "patches" / ".gitattributes").read_text(encoding="utf-8") == (
        "*.patch text eol=lf\n"
    )
    patch_text = PATCH.read_text(encoding="utf-8")
    assert "-  f2c::Random rand;" in patch_text
    assert "+  f2c::Random rand(42U);" in patch_text
    assert patch_text.count("diff --git ") == 1
    assert hashlib.sha256(PATCH.read_bytes()).hexdigest() == EXPECTED_PATCH_SHA256

    repo = tmp_path / "opennav_coverage"
    target = repo / "opennav_coverage" / "test" / "test_path.cpp"
    target.parent.mkdir(parents=True)
    target.write_text(
        "TEST(PathTests, TestpathGeneration)\n"
        "{\n"
        "\n"
        "  // Generate some toy route\n"
        "  f2c::Random rand;\n"
        "  auto field = rand.generateRandField(1e5, 5);\n"
        "  opennav_coverage_msgs::msg::SwathMode sw_settings;\n"
        "  auto swaths = swath_gen.generateSwaths(field.getField().getGeometry(0), sw_settings);\n"
        "}\n",
        encoding="utf-8",
    )
    subprocess.run(["git", "init", "-q", str(repo)], check=True)
    subprocess.run(["git", "-C", str(repo), "apply", "--check", str(PATCH)], check=True)
    subprocess.run(["git", "-C", str(repo), "apply", str(PATCH)], check=True)
    assert "f2c::Random rand(42U);" in target.read_text(encoding="utf-8")


def test_import_and_stage1_audit_the_patched_tree() -> None:
    importer = (ROOT / "scripts" / "import_upstream.sh").read_text(encoding="utf-8")
    stage1 = (ROOT / "scripts" / "stage1_ci.sh").read_text(encoding="utf-8")

    assert "git -C \"$repo\" apply --check \"$patch_file\"" in importer
    assert 'OPENNAV_COVERAGE_PATCH_SHA256="c101a9bfa3078139566fe8577f63a4cc525bde71d8fb3f244fdc2beb846af0b1"' in importer
    assert "patched_diff_sha256" in importer
    assert '"working_tree_clean": True' in importer
    assert '"patched_commit": os.environ["OPENNAV_PATCH_PATCHED_COMMIT"]' in importer
    assert '"patched_tree": os.environ["OPENNAV_PATCH_PATCHED_TREE"]' in importer
    assert '"status_porcelain": []' in importer
    assert 'GIT_AUTHOR_DATE="$OPENNAV_COVERAGE_PATCH_DATE"' in importer
    assert 'GIT_COMMITTER_DATE="$OPENNAV_COVERAGE_PATCH_DATE"' in importer
    assert 'OPENNAV_COVERAGE_PATCH_DATE="2000-01-01T00:00:00+00:00"' in importer
    assert '"patch_path": os.environ["OPENNAV_PATCH_PATH"]' in importer
    assert "verify_opennav_patch_state" in stage1
    assert 'patch_sha256 = hashlib.sha256(patch_path.read_bytes()).hexdigest()' in stage1
    assert 'patched_commit != report["patched_commit"]' in stage1
    assert 'patched_tree != report["patched_tree"]' in stage1
    assert "actual_metadata != expected_metadata" in stage1
    assert "opennav_coverage_patch_state_before.txt" in stage1
    assert "opennav_coverage_patch_state_after.txt" in stage1
    assert 'repositories[name]["patch"] = patch_report' in stage1
