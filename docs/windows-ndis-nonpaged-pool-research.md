# Windows NDIS / nonpaged-pool source research

Research date: 2026-08-31. This note supplements the local, read-only snapshot at `reports/engineering/windows_ndis_nonpaged_pool_diagnostic.json`; the structured source inventory is `reports/engineering/windows_ndis_nonpaged_pool_source_research.json`.

## Scope and evidence boundary

The local snapshot observed 21.80 GiB of nonpaged pool on Windows 11 build 26200. It found active Tailscale/Wintun, FSE/Hyper-V switching, Realtek RTL8125, and Intel AX211 components; iKuuuVPN had only a running helper service. PoolMon was absent and the NDIS Operational log was disabled. No driver, service, adapter, registry, WSL, or installation action was performed for this research.

Microsoft documents PoolMon plus `pooltag.txt` Mapped_Driver output as the way to associate a growing pool tag with a component. See [Use PoolMon to Find a Kernel-Mode Memory Leak](https://learn.microsoft.com/en-us/windows-hardware/drivers/debugger/using-poolmon-to-find-a-kernel-mode-memory-leak). Therefore the aggregate `PoolNonpagedBytes` measurement cannot establish that an active adapter, VPN, or filter is the allocator.

## Official evidence

- Microsoft’s [`!ndiskd.nbpool` documentation](https://learn.microsoft.com/en-us/windows-hardware/drivers/debuggercmds/-ndiskd-nbpool) gives `Nnbf` as a NETIO NB-pool example (`NETIO!NetioInitializeNetBufferListLibrary`). It does not identify a leak, an attached miniport/filter, or the owner of local `Nbuf`/`Nnbl` tags.
- Intel’s [AX211 Windows driver page](https://www.intel.com/content/www/us/en/download/19351/intel-wireless-wi-fi-drivers-for-windows-10-and-windows-11.html) lists AX211 and says versions from 23.170.0 onward were validated for Windows 11 25H2. The installed 23.50.0.6 predates that validation. This is compatibility evidence, not a nonpaged-pool bug fix.
- Tailscale’s [official changelog](https://tailscale.com/changelog) has current Windows maintenance information but no reviewed entry names a Wintun 0.14 nonpaged-pool leak or remediation version. The [WireGuard Wintun source](https://github.com/WireGuard/wintun) documents packet-buffer release discipline but not the requested Windows 26200 leak.
- MSI’s [Vector 17 HX A13V specification](https://www.msi.com/Laptop/Vector-17-HX-A13VX/Specification) corroborates this model line’s Windows 11, 2.5G LAN, and Wi-Fi 6E hardware. It is not a driver release note.

## Related but not confirmation

A [user-filed issue in Microsoft’s WSL repository](https://github.com/microsoft/WSL/issues/40804) reports a similar 21–23 GiB/hour nonpaged-pool pattern on build 26200.8655, but attributes it to `NtFC` in `ntfs.sys` while WSL2 VHDX I/O is active. It is closed as not planned, has no Microsoft confirmation or fix, and does not establish a connection to this computer or to the NDIS-style tags.

Another [user-filed WSL report](https://github.com/microsoft/WSL/issues/12006) alleges a `netio.sys` leak in mirrored networking, but targets build 22631.4112 rather than this build. It is subsystem-adjacent evidence only.

## No matching official fix found

No matching Microsoft Release Health, Tailscale/WireGuard, Intel, Realtek, or MSI primary source was located for a Windows 11 build 26200 nonpaged-pool leak specifically tied to `Nbuf`, `Nnbl`, Wintun 0.14, FSE/vmswitch, RTL8125, or AX211. That is a search result, not evidence that no private, future, or differently described fix exists.

The evidence ranking remains: no `likely` cause; Tailscale/Wintun, FSE/vmswitch, RTL8125, and AX211 are `possible` from local activity alone; iKuuuVPN remains `unproven`. If tag capture later shows `NtFC`, investigate WSL/Hyper-V VHDX I/O before altering any network component. If `Nnbf`, `Nbuf`, or `Nnbl` dominates, capture a tag-to-driver mapping before assigning NETIO or an attached filter/miniport as the owner.
