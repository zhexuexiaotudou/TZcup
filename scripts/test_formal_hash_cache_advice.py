import hashlib
import importlib
from pathlib import Path

import pytest


MODULE_AND_HASHER = (
    ("formal_final_runtime_closure", "_sha256"),
    ("generate_formal_vehicle_snapshot", "_sha256"),
    ("aggregate_integrated_functional_acceptance", "sha256_file"),
    ("run_formal_final_acceptance", "_sha256"),
)


@pytest.mark.parametrize("module_name,hasher_name", MODULE_AND_HASHER)
def test_hash_is_unchanged_and_advises_whole_file_cache_drop(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
    hasher_name: str,
) -> None:
    module = importlib.import_module(module_name)
    payload = (b"formal-cache-advice\x00" * 100_000) + b"tail"
    path = tmp_path / f"{module_name}.bin"
    path.write_bytes(payload)
    calls: list[tuple[int, int, int, int]] = []
    monkeypatch.setattr(module.os, "POSIX_FADV_DONTNEED", 4, raising=False)
    monkeypatch.setattr(
        module.os,
        "posix_fadvise",
        lambda fd, offset, length, advice: calls.append(
            (fd, offset, length, advice)
        ),
        raising=False,
    )

    assert getattr(module, hasher_name)(path) == hashlib.sha256(payload).hexdigest()
    assert len(calls) == 1
    assert calls[0][1:] == (0, 0, 4)


@pytest.mark.parametrize("module_name,hasher_name", MODULE_AND_HASHER)
def test_missing_cache_advice_api_never_changes_hash_contract(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
    hasher_name: str,
) -> None:
    module = importlib.import_module(module_name)
    payload = b"cache advice API is optional"
    path = tmp_path / f"{module_name}.bin"
    path.write_bytes(payload)
    monkeypatch.delattr(module.os, "POSIX_FADV_DONTNEED", raising=False)
    monkeypatch.delattr(module.os, "posix_fadvise", raising=False)

    assert getattr(module, hasher_name)(path) == hashlib.sha256(payload).hexdigest()


@pytest.mark.parametrize("module_name,hasher_name", MODULE_AND_HASHER)
@pytest.mark.parametrize("advice_error", (OSError, ValueError))
def test_cache_advice_errors_never_change_hash_contract(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
    hasher_name: str,
    advice_error: type[Exception],
) -> None:
    module = importlib.import_module(module_name)
    payload = b"cache advice is optional; identity is not"
    path = tmp_path / f"{module_name}.bin"
    path.write_bytes(payload)
    monkeypatch.setattr(module.os, "POSIX_FADV_DONTNEED", 4, raising=False)

    def reject_advice(*_args: object) -> None:
        raise advice_error("cache advice unavailable")

    monkeypatch.setattr(module.os, "posix_fadvise", reject_advice, raising=False)

    assert getattr(module, hasher_name)(path) == hashlib.sha256(payload).hexdigest()


@pytest.mark.parametrize("module_name,hasher_name", MODULE_AND_HASHER)
def test_read_failure_never_requests_cache_drop(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    module_name: str,
    hasher_name: str,
) -> None:
    module = importlib.import_module(module_name)
    path = tmp_path / f"{module_name}.bin"
    path.write_bytes(b"unreadable payload")
    calls: list[tuple[int, int, int, int]] = []
    monkeypatch.setattr(module.os, "POSIX_FADV_DONTNEED", 4, raising=False)
    monkeypatch.setattr(
        module.os,
        "posix_fadvise",
        lambda fd, offset, length, advice: calls.append(
            (fd, offset, length, advice)
        ),
        raising=False,
    )

    class FailingStream:
        def __enter__(self) -> "FailingStream":
            return self

        def __exit__(self, *_args: object) -> bool:
            return False

        def read(self, _size: int) -> bytes:
            raise OSError("synthetic read failure")

        def fileno(self) -> int:
            return 42

    monkeypatch.setattr(module.Path, "open", lambda *_args, **_kwargs: FailingStream())

    with pytest.raises(OSError, match="synthetic read failure"):
        getattr(module, hasher_name)(path)
    assert calls == []
