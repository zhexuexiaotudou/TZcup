#!/usr/bin/env python3
"""Read-only Windows routing dependency audit for the historical RDK S100P.

The audit never opens a socket, invokes SSH, pings, scans, or transmits a
packet.  It deliberately reports candidate board networks rather than host
addresses and records ``UNKNOWN`` whenever local routing data cannot prove an
isolation product's effect on board traffic.
"""

from __future__ import annotations

import argparse
import ipaddress
import json
import platform
import re
import subprocess
from pathlib import Path
from typing import Any, Callable


ROOT = Path(__file__).resolve().parents[1]
HISTORICAL_G0 = ROOT / "artifacts/s100p_hardware_bringup/20260830T1530Z_g0_readonly/G0/board_identity_and_inventory.json"
SSH_DIR = Path.home() / ".ssh"
REPORT_ID = "tzcup_s100p_windows_connectivity_dependency_audit_v1"


def _powershell_json(command: str, runner: Callable[..., Any] = subprocess.run) -> list[dict[str, Any]]:
    completed = runner(
        [
            "powershell.exe",
            "-NoProfile",
            "-NonInteractive",
            "-Command",
            "$OutputEncoding=[Console]::OutputEncoding=[System.Text.UTF8Encoding]::new($false); " + command,
        ],
        check=False,
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )
    output = getattr(completed, "stdout", "") or ""
    if completed.returncode != 0 or not output.strip():
        return []
    payload = json.loads(output)
    return payload if isinstance(payload, list) else [payload]


def _historical_board_endpoints(path: Path) -> tuple[set[str], set[str]]:
    """Extract only private IPv4 endpoints and names from a retained G0 record."""
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (FileNotFoundError, json.JSONDecodeError):
        return set(), set()
    identity = payload.get("identity") if isinstance(payload.get("identity"), dict) else {}
    hostname = identity.get("hostname") if isinstance(identity, dict) else None
    if isinstance(hostname, str):
        hostnames = {hostname}
    elif isinstance(hostname, dict) and isinstance(hostname.get("value"), str):
        hostnames = {str(hostname["value"])}
    else:
        hostnames = set()
    endpoints: set[str] = set()
    commands = payload.get("commands") if isinstance(payload.get("commands"), dict) else {}
    network = commands.get("network") if isinstance(commands, dict) else None
    if isinstance(network, dict) and network.get("command") == "ip -details address show":
        interface = ""
        for line in str(network.get("stdout", "")).splitlines():
            heading = re.match(r"^\d+:\s+([^:]+):", line)
            if heading:
                interface = heading.group(1)
                continue
            address_match = re.search(r"\binet\s+(\d+\.\d+\.\d+\.\d+)/\d+", line)
            if not address_match or not interface.startswith("eth"):
                continue
            address = ipaddress.ip_address(address_match.group(1))
            if address.is_private and not address.is_link_local:
                endpoints.add(str(address))
    return endpoints, hostnames


def _redacted_subnet(address: str) -> str:
    return str(ipaddress.ip_network(f"{address}/24", strict=False))


def _known_hosts_summary(known_hosts: Path, endpoints: set[str], hostnames: set[str]) -> dict[str, object]:
    if not known_hosts.exists():
        return {"present": False, "plaintext_relevant_endpoint_entries": 0, "plaintext_relevant_hostname_entries": 0, "hashed_relevant_entry_state": "UNKNOWN"}
    endpoint_count = 0
    hostname_count = 0
    for line in known_hosts.read_text(encoding="utf-8", errors="replace").splitlines():
        hosts = line.split(maxsplit=1)[0] if line and not line.startswith("#") else ""
        entries = set(hosts.split(","))
        endpoint_count += sum(address in entries or f"[{address}]:22" in entries for address in endpoints)
        hostname_count += sum(hostname in entries for hostname in hostnames)
    return {
        "present": True,
        "plaintext_relevant_endpoint_entries": endpoint_count,
        "plaintext_relevant_hostname_entries": hostname_count,
        "hashed_relevant_entry_state": "UNKNOWN",  # hashes cannot be associated without querying SSH tooling
    }


def _ssh_config_summary(config: Path, endpoints: set[str], hostnames: set[str]) -> dict[str, object]:
    if not config.exists():
        return {"present": False, "matching_host_blocks": 0, "relevant_aliases_declared": False}
    matching = 0
    for line in config.read_text(encoding="utf-8", errors="replace").splitlines():
        match = re.match(r"^\s*Host\s+(.+?)\s*$", line, re.IGNORECASE)
        if not match:
            continue
        aliases = set(match.group(1).split())
        if aliases & (endpoints | hostnames):
            matching += 1
    return {"present": True, "matching_host_blocks": matching, "relevant_aliases_declared": matching > 0}


def _route_for_candidate(routes: list[dict[str, Any]], endpoint: str) -> dict[str, object]:
    address = ipaddress.ip_address(endpoint)
    candidates: list[tuple[int, dict[str, Any]]] = []
    for route in routes:
        try:
            network = ipaddress.ip_network(str(route.get("DestinationPrefix")), strict=False)
        except ValueError:
            continue
        if address in network:
            candidates.append((network.prefixlen, route))
    if not candidates:
        return {"route_state": "NO_MATCHING_ROUTE"}
    route = sorted(candidates, key=lambda row: row[0], reverse=True)[0][1]
    return {
        "route_state": "ROUTE_PRESENT",
        "interface_index": route.get("ifIndex"),
        "destination_prefix": _redacted_subnet(endpoint),
        "next_hop_is_on_link": str(route.get("NextHop")) == "0.0.0.0",
    }


def _neighbor_state(value: object) -> str:
    if isinstance(value, str):
        return value
    return {0: "UNREACHABLE", 1: "INCOMPLETE", 2: "PROBE", 3: "DELAY", 4: "STALE", 5: "REACHABLE", 6: "PERMANENT"}.get(value, "UNKNOWN")


def audit(
    root: Path = ROOT,
    *,
    runner: Callable[..., Any] = subprocess.run,
    ssh_dir: Path = SSH_DIR,
) -> dict[str, object]:
    root = root.resolve()
    endpoints, hostnames = _historical_board_endpoints(root / HISTORICAL_G0.relative_to(ROOT))
    routes = _powershell_json(
        "Get-NetRoute -AddressFamily IPv4 | Select-Object ifIndex,InterfaceAlias,DestinationPrefix,NextHop,RouteMetric | ConvertTo-Json -Compress",
        runner,
    )
    neighbors = _powershell_json(
        "Get-NetNeighbor -AddressFamily IPv4 | Select-Object InterfaceAlias,IPAddress,State | ConvertTo-Json -Compress",
        runner,
    )
    adapters = _powershell_json(
        "Get-NetAdapter -IncludeHidden | Select-Object Name,Status,ifIndex | ConvertTo-Json -Compress",
        runner,
    )
    candidate_routes: list[dict[str, object]] = []
    for endpoint in sorted(endpoints):
        route = _route_for_candidate(routes, endpoint)
        neighbor = next((row for row in neighbors if str(row.get("IPAddress")) == endpoint), None)
        route["candidate_subnet"] = _redacted_subnet(endpoint)
        route["neighbor_state"] = _neighbor_state(neighbor.get("State")) if neighbor else "NO_NEIGHBOR_ENTRY"
        route["endpoint_redacted"] = True
        candidate_routes.append(route)
    def adapter_rows(term: str) -> list[dict[str, object]]:
        return [
            {"name": row.get("Name"), "status": row.get("Status"), "interface_index": row.get("ifIndex")}
            for row in adapters
            if term.casefold() in str(row.get("Name", "")).casefold()
        ]
    tailscale = adapter_rows("tailscale")
    fse = adapter_rows("fse")
    route_indexes = {row.get("interface_index") for row in candidate_routes if row.get("route_state") == "ROUTE_PRESENT"}
    return {
        "report_id": REPORT_ID,
        "audit_mode": "windows_read_only_no_network_transmission",
        "execution_prohibited": ["SSH", "ping", "port scan", "socket connection", "packet transmission", "board command execution"],
        "historical_board_inventory": {
            "path": HISTORICAL_G0.relative_to(ROOT).as_posix(),
            "candidate_endpoint_count": len(endpoints),
            "candidate_hostname_count": len(hostnames),
            "endpoint_values_redacted": True,
        },
        "project_connectivity_declaration": "NO_BOARD_HOST_OR_SSH_ROUTE_DECLARED_IN_LOCAL_PREDEPLOY_CONTRACTS",
        "ssh_configuration": {
            "config": _ssh_config_summary(ssh_dir / "config", endpoints, hostnames),
            "known_hosts": _known_hosts_summary(ssh_dir / "known_hosts", endpoints, hostnames),
            "private_keys_read": False,
            "credentials_exposed": False,
        },
        "candidate_routes": candidate_routes,
        "isolation_products": {
            "tailscale": {
                "adapters": tailscale,
                "selected_for_candidate_route": any(row.get("interface_index") in route_indexes for row in tailscale),
                "effect_on_board_connectivity": "UNKNOWN_NO_PACKET_OR_FILTER_EVIDENCE",
            },
            "fse": {
                "adapters": fse,
                "selected_for_candidate_route": any(row.get("interface_index") in route_indexes for row in fse),
                "effect_on_board_connectivity": "UNKNOWN_NO_PACKET_OR_FILTER_EVIDENCE",
            },
        },
        "conclusion": "Current route selection may indicate a direct Ethernet path, but stale ARP entries and routing metadata do not prove board reachability. Tailscale/FSE isolation impact remains UNKNOWN without prohibited packet, filter, or board-side evidence.",
        "status": "S100P_WINDOWS_CONNECTIVITY_DEPENDENCIES_OBSERVED_NOT_CONNECTED",
        "runtime_connectivity_verified": False,
        "host_os": platform.system(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, default=ROOT / "reports/engineering/s100p_windows_connectivity_dependency_audit.json")
    args = parser.parse_args()
    report = audit(args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"{report['status']}: wrote {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
