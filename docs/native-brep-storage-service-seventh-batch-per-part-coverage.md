# Storage/service seventh-batch per-part native-source coverage

This independent batch maps the reconstruction manifest's **24 storage** and
**10 service** project-authored mesh rows one-to-one to named, lazy CadQuery
builder functions. It is static design-input evidence only: no mesh conversion,
CadQuery execution, FCStd/STEP export, manufacturing release, or runtime
acceptance is claimed.

```powershell
py -3 .\scripts\validate_native_brep_storage_service_seventh_batch_contract.py
py -3 -m pytest .\scripts\test_native_brep_storage_service_seventh_batch_contract.py -q
```

Each source mesh, manifest ID, and builder symbol must be unique and must exactly
match the still-pending reconstruction manifest. The test parses Python AST and
hashes source/contract files; it never loads CadQuery, WSL, Gazebo, Docker, or
a mesh.

Dry-bin source parts are distinct from wastewater source parts, with
`storage_dry_wet_partition` mapped independently. The dry mass interface
retains mutually-exclusive resident cardboard/PP/PET/aluminium bodies versus
the bounded aggregate payload. The wet interface retains the requirement that
recovered standing water increment wastewater payload. These are static
interfaces—not physical garbage/water validation.

The charge housing, receptacle, hinged door, and lock are separate mappings.
The drain pipe, valve body, visible stem/indicator, actuator mount, cap, and
coupling are also separate. A fused service box is forbidden. Manufacturing
dimensions, material/mass, holes/threads, seals, ingress, pressure, charge
ratings/interlock, drain internals, and all native export/runtime gates remain
fail-closed.
