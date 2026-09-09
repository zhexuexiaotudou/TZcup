from pathlib import Path

import yaml

from mcap_validation import MCAP_MAGIC, inspect_mcap_bag


def _write_metadata(bag: Path) -> None:
    (bag / "metadata.yaml").write_text(
        yaml.safe_dump({
            "rosbag2_bagfile_information": {
                "message_count": 1,
                "duration": {"nanoseconds": 1},
                "relative_file_paths": ["bag_0.mcap"],
                "topics_with_message_count": [],
            }
        }),
        encoding="utf-8",
    )


def test_inspect_mcap_bag_requires_terminal_footer(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    _write_metadata(bag)
    (bag / "bag_0.mcap").write_bytes(MCAP_MAGIC + b"unfinished")

    evidence = inspect_mcap_bag(bag)

    assert evidence["metadata_present"] is True
    assert evidence["metadata_valid"] is True
    assert evidence["footer_complete"] is False
    assert evidence["sealed"] is False


def test_inspect_mcap_bag_accepts_terminal_footer(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    _write_metadata(bag)
    footer = b"\x02" + (20).to_bytes(8, "little") + (b"\0" * 20) + MCAP_MAGIC
    (bag / "bag_0.mcap").write_bytes(MCAP_MAGIC + footer)

    evidence = inspect_mcap_bag(bag)

    assert evidence["mcap_files"] == ["bag_0.mcap"]
    assert evidence["footer_complete"] is True
    assert evidence["sealed"] is True
