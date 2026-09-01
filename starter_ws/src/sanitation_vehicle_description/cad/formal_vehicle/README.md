# Formal vehicle CAD source

`formal_vehicle_layout.scad` is the project-owned, millimetre-based packaging
assembly used to review service clearances and sensor placement. It deliberately
mirrors the link groups in `formal_competition_vehicle.urdf.xacro`; the URDF
remains the dynamics authority.

`generate_cleaning_storage_meshes.py` is the dependency-free parametric source
for the first detailed cleaning, recovery and split-bin visual set. It generates
binary STL files under `meshes/project/{cleaning,storage}` and
`meshes/generated/platform` in metre units:

```powershell
py -3 starter_ws/src/sanitation_vehicle_description/cad/formal_vehicle/generate_cleaning_storage_meshes.py
```

`generate_product_bodywork_meshes.py` is the dependency-free product-exterior
source. It generates 47 deterministic STL assets for moulded front/rear shells,
the recessed arm work bay, lower tub, service doors, wheel arches, safety trim,
lights, bumpers and brush guards. The four service-door panels use hinge-local
coordinates and share detailed hinge-barrel and rotary-latch hardware meshes:

```powershell
py -3 starter_ws/src/sanitation_vehicle_description/cad/formal_vehicle/generate_product_bodywork_meshes.py
```

The current source-expanded formal product profile references 42 unique
bodywork meshes. Optional comparison assets remain generated but intentionally
unreferenced. Primitive geometry remains collision and inertia authority only.

The generated set includes service details absent from the packaging model:
37D motor housings and flanges, gearboxes, keyed shafts, ribbed brush discs,
48 curved side-brush bundles, a four-start helical central roller, bearing
housings, a curved guard, real helical suspension springs, curved twin squeegee
blades, a shaped suction nozzle, corrugated hoses, filter and coupling hardware,
an externally detailed pump assembly, and ribbed dry/wet storage panels.
It also generates the rotating pump rotor, inline flow-sensor body, open
robot-deposition hopper/gate/chute, XW540 waterproof gate actuator and horn,
three-piece manual over-centre service latches shared by the dry-bin and
wastewater lids (body base, moving hand lever and lid keeper),
fused power-distribution enclosure,
isolated DC/DC module and hardwired safety relay used by the functional-position
register.

`generate_power_service_hardware_meshes.py` additionally generates the service
hardware that must be visible and articulated in simulation: A300 battery/BMS
details, charge receptacle/door/lock, mushroom E-stop, the complete wastewater
drain train, a lockable rotary main-battery isolator and a sealed traction
contactor with a separately moving armature witness. The isolator and contactor
are not decorative meshes: their URDF collision/inertia links and joint
positions participate in the fail-closed main-power state machine.

Open the file in OpenSCAD and set `$explode` to a non-zero value for an exploded
packaging view. Exported STL files are visual-only and must not replace the URDF
collision primitives or inertial data.

This is a nominal digital design, not metrology from a purchased vehicle. The
external component envelopes use published/open reference dimensions; brackets,
cleaning hardware, tanks and cabinet are project-designed parts. In particular,
the Pololu, Actuonix and Jabsco meshes model observable housings, interfaces and
mounting features only. They do not assert exact hidden gears, windings,
diaphragms, manufacturing tolerances or unit-specific calibration.
