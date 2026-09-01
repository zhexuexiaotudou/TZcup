# A300 power, charging and wastewater service hardware preparation

Status: source-integrated physical design; final session-bound Gazebo acceptance
is pending. The final vehicle Xacro, component register, safety authority and
controller ownership now include the hardware described below. This document
does not claim physical A300 teardown accuracy or live-board acceptance.

## Battery and BMS mass truth

The public A300 values used here are 25.6 V LiFePO4, 1024 Wh for the 40 Ah
variant, two installed packs, 78.5 kg vehicle mass for 40 Ah and 93.5 kg for
80 Ah. The 15 kg whole-vehicle difference corresponds to two additional packs,
so 7.5 kg per pack is a transparent engineering inference. It is not a
published Clearpath single-pack mass.

The two 40 Ah-vehicle pack assemblies are therefore modelled at 7.5 kg each.
Each assembly is provisionally split into a 7.35 kg cell/case link and a
0.15 kg internal BMS link. The BMS split, envelopes and internal locations are
engineering placeholders pending physical measurement or licensed internal
CAD. Both pack and BMS joints are fixed.

The 15 kg is transferred out of the existing A300 chassis aggregate. The
drivetrain mass contract now carries 35.5 kg for chassis/internal structure
excluding batteries plus 15.0 kg for two explicit batteries; motors, fixed
beams, spacers, wheels and standard top plate remain unchanged. The sum stays
78.5 kg. Adding the new links without subtracting the aggregate is forbidden.

The modelled BMS state includes per-pack voltage, current and temperature,
aggregate SOC, charging contact and latched fault. Invalid or stale pack state,
either pack fault, overtemperature, overcurrent or undervoltage inhibits the
complete traction path. A connected charger also inhibits traction. These
signals enter the same whole-vehicle safety authority; they do not form a
second final actuator publisher.

## Physical main-power chain

The fused high-power branch is no longer represented by a Boolean alone. The
vehicle has a visible 90-degree rotary main-isolator handle and housing plus a
separate contactor housing and 4 mm armature. Their links, joints, collision,
inertia, command and measured feedback are registered under
`fused_power_distribution`. The safety path uses the measured isolator and
contactor positions with freshness timeouts; a requested high-power state
cannot become applied power until both physical feedback conditions hold.
The added 0.4 kg is transferred out of the previous aggregate distribution-box
mass, so this refinement does not increase the frozen whole-vehicle mass.

## Physical charging interface

The implemented right-side charge datum remains at base-link coordinates
`[0.250, 0.402, 0.330] m`, rotated 90 degrees about X. It is expanded into a
fixed housing, a bounded 110-degree manual door hinge, a fixed receptacle and
a 6 mm spring-latch prismatic lock. The 0.100 kg engineering mass allocation is
transferred from the existing aggregate bodywork trim instead of added.

The charger plug is an external service-world model, not a child of the
vehicle. Charging requires verified plug/receptacle contact, fully engaged
lock, stationary vehicle, traction inhibit, valid BMS state and no BMS fault.
Door, contact and lock states are separate so a Boolean command cannot pretend
that a connector is physically inserted.

## Physical wastewater drain

The implemented exterior drain datum is retained at base-link coordinates
`[-0.490, -0.305, 0.315] m`. Its physical load/fluid path descends from
`wastewater_tank_link`, whose current transform makes the service valve body
`[-0.285, -0.060, -0.024] m` in the tank frame. The implementation provides separate
pipe, valve body, rotating ball, actuator, service cap and coupling links.

The 0.900 kg assembly is an engineering allocation within the existing 3 kg
filters/valves/strainer payload allowance. It is custom cleaning equipment,
not an A300 factory component. The selected engineering actuator model is a 24 V spring-return,
normally-closed device: loss of command, power or safety permission returns
the ball joint to zero. Opening is permitted only when stationary, cleaning is
disabled, the recovery pump is stopped, the service permit is true, the cap is
open, a drain hose is connected and tank-level input is valid.

The powered ball joint is `wastewater_drain_valve_joint`, exported as a
position command/state interface and owned by `service_controller`. The
separate `wastewater_drain_service_cap_joint` remains a passive measured
interlock. The component register must never classify the powered valve joint
as passive.

The product safety manager derives `cap_open` from
`wastewater_drain_service_cap_joint` crossing the configured opening angle; it
does not trust a free-standing Boolean command. Hose presence comes from the
dedicated drain-coupling contact sensor on
`/formal_vehicle/service/raw/drain_hose_contact`. A missing or stale joint/contact
observation closes the normally-closed valve.

Initial drainage simulation uses valve-angle-dependent orifice flow tied to
the existing tank mass/volume ledger. It must close water mass balance within
one percent, but it does not claim CFD fidelity.

## Current integration and remaining acceptance

Detailed meshes, declared links/joints, coordinate frames and mass
redistribution are present in the frozen source snapshot. The remaining gates
are runtime or physical-correlation gates:

1. Live-validate the implemented main-isolator/contactor, BMS,
   charge-contact/lock and drain-actuator watchdogs under publisher loss,
   invalid input and power loss.
2. Prove the implemented charge connector and lock chain through non-empty
   `/formal_vehicle/service/raw/charge_plug_contact`, not command state;
   prove drive stays inhibited throughout charging.
3. Prove the drain valve physically rotates, spring-returns closed, cannot open
   while moving/cleaning, and conserves tank water mass.
4. Re-run whole-vehicle safety, mobility, water recovery, map lifecycle and
   dynamic-obstacle acceptance with one final traction command authority.
5. Correlate pack mass/envelope, connectors, isolator/contactor travel and
   electrical thresholds against physical hardware before any real-vehicle claim.
