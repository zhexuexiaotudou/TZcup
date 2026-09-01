from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_dependency_probe_fallback_is_native_linux_and_proot_specific() -> None:
    source = (ROOT / "scripts/formal_dynamic_dependencies.sh").read_text(
        encoding="utf-8"
    )
    assert "/usr/bin/ldd" in source
    assert 'FORMAL_NATIVE_LINUX_RUNTIME:-}' in source
    assert "grep -qi microsoft /proc/sys/kernel/osrelease" in source
    assert "you do not have read permission" in source
    assert "/lib/x86_64-linux-gnu/ld-linux-x86-64.so.2" in source
    assert "os.stat(path, follow_symlinks=False)" in source
    assert 'stream.read(4) != b"\\x7fELF"' in source
    assert 'exec "${loader}" --list "${target}"' in source


def test_runtime_callers_use_the_audited_dependency_probe() -> None:
    for relative in (
        "scripts/run_gz_transport13_late_discovery_smoke.sh",
        "scripts/run_formal_visual_single_topic_diagnostic.sh",
    ):
        source = (ROOT / relative).read_text(encoding="utf-8")
        assert "scripts/formal_dynamic_dependencies.sh" in source
