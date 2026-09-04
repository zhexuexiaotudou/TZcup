from __future__ import annotations

import json
from pathlib import Path
from types import SimpleNamespace

from audit_s100p_windows_connectivity_dependencies import HISTORICAL_G0, ROOT, audit


def test_read_only_audit_redacts_endpoints_and_keeps_isolation_unknown(tmp_path: Path) -> None:
    fixture_root = tmp_path / "repository"
    inventory = fixture_root / HISTORICAL_G0.relative_to(ROOT)
    inventory.parent.mkdir(parents=True)
    inventory.write_text(
        json.dumps(
            {
                "identity": {"hostname": {"value": "s100p-fixture"}},
                "commands": {
                    "network": {
                        "command": "ip -details address show",
                        "stdout": (
                            "2: eth0: <BROADCAST>\n"
                            "    inet 192.0.2.10/24 scope global eth0\n"
                            "3: eth1: <BROADCAST>\n"
                            "    inet 198.51.100.20/24 scope global eth1\n"
                        ),
                    }
                },
            }
        ),
        encoding="utf-8",
    )
    ssh_dir = tmp_path / ".ssh"
    ssh_dir.mkdir()
    (ssh_dir / "known_hosts").write_text("192.0.2.10 ssh-ed25519 public-data\n", encoding="utf-8")
    calls: list[list[str]] = []

    def runner(command: list[str], **_: object) -> SimpleNamespace:
        calls.append(command)
        if "Get-NetRoute" in command[-1]:
            data = [{"ifIndex": 23, "InterfaceAlias": "Ethernet", "DestinationPrefix": "192.0.2.0/24", "NextHop": "0.0.0.0", "RouteMetric": 25}]
        elif "Get-NetNeighbor" in command[-1]:
            data = [{"InterfaceAlias": "Ethernet", "IPAddress": "192.0.2.10", "State": "Stale"}]
        else:
            data = [{"Name": "Tailscale", "Status": "Up", "ifIndex": 19}, {"Name": "vEthernet (FSE HostVnic)", "Status": "Up", "ifIndex": 44}]
        return SimpleNamespace(returncode=0, stdout=json.dumps(data))

    report = audit(fixture_root, runner=runner, ssh_dir=ssh_dir)
    rendered = json.dumps(report)
    assert len(calls) == 3
    assert all(command[0].endswith("powershell.exe") for command in calls)
    assert "ssh" not in " ".join(" ".join(command).lower() for command in calls)
    assert "192.0.2.10" not in rendered
    assert report["historical_board_inventory"]["candidate_endpoint_count"] == 2
    assert report["runtime_connectivity_verified"] is False
    assert report["isolation_products"]["tailscale"]["effect_on_board_connectivity"] == "UNKNOWN_NO_PACKET_OR_FILTER_EVIDENCE"
    assert report["isolation_products"]["fse"]["effect_on_board_connectivity"] == "UNKNOWN_NO_PACKET_OR_FILTER_EVIDENCE"
