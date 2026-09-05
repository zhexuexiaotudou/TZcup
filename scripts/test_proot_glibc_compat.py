from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_proot_glibc_compat_is_narrow_and_fail_closed() -> None:
    source = (ROOT / "scripts/proot_glibc_compat.c").read_text(encoding="utf-8")
    assert "int statx(" in source
    assert "fstatat(dirfd, pathname" in source
    assert "int faccessat2(" in source
    assert "syscall(SYS_faccessat" in source
    assert "geteuid() == 0" in source
    assert "errno = ENOTSUP" in source
    assert "STATX_BASIC_STATS" in source
    assert "AT_SYMLINK_NOFOLLOW" in source

