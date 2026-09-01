from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "starter_ws/src/sanitation_gazebo_control/src/DynamicPayloadSystem.cc"
).read_text(encoding="utf-8")
VEHICLE_XACRO = (
    ROOT
    / "starter_ws/src/sanitation_vehicle_description/urdf/formal_competition_vehicle.urdf.xacro"
).read_text(encoding="utf-8")
STORAGE_XACRO = (
    ROOT
    / "starter_ws/src/sanitation_vehicle_description/urdf/high_fidelity/storage_system.xacro"
).read_text(encoding="utf-8")
WATER_SOURCE = (
    ROOT
    / "starter_ws/src/sanitation_gazebo_control/src/WaterRecoverySystem.cc"
).read_text(encoding="utf-8")
LAUNCH_SOURCE = (
    ROOT
    / "starter_ws/src/sanitation_vehicle_description/launch/formal_vehicle_sim.launch.py"
).read_text(encoding="utf-8")


def test_fixed_joint_lumped_payload_targets_base_and_reserve_frames() -> None:
    assert 'model.LinkByName(_ecm, "base_footprint")' in SOURCE
    assert '"dry_bin_payload_reserve_link"' in SOURCE
    assert '"wastewater_payload_reserve_link"' in SOURCE
    assert "components::ModelSdf" in SOURCE
    assert "FrameByName" in SOURCE
    assert 'SemanticPose().Resolve(frameInModel, "__model__")' in SOURCE
    assert "const auto frameWorld = _modelWorld * frameInModel" in SOURCE
    assert "baseWorld.Inverse()" in SOURCE
    assert "dryEntity" not in SOURCE
    assert "waterEntity" not in SOURCE
    assert "SetBoxLoad" not in SOURCE


def test_structural_baseline_removes_both_seed_reserves_once() -> None:
    assert "this->structuralInertial = baseInertial->Data()" in SOURCE
    assert "this->structuralInertial -= initialDry" in SOURCE
    assert "this->structuralInertial -= initialWater" in SOURCE
    assert "2.0 * this->minimumMassKg" not in SOURCE
    assert "drySeedMassKg - waterSeedMassKg" in SOURCE
    assert "std::max(initialDryMassKg, this->minimumMassKg)" in SOURCE
    assert "std::max(initialWaterMassKg, this->minimumMassKg)" in SOURCE
    assert "this->structuralInertial.MassMatrix().Mass() + _dryMass + _waterMass" in SOURCE


def test_water_and_dry_boxes_match_current_xacro_envelopes() -> None:
    assert "const double drySizeX{0.485}" in SOURCE
    assert "const double drySizeY{0.355}" in SOURCE
    assert "const double drySizeZ{0.233}" in SOURCE
    assert "const double waterSizeX{0.350}" in SOURCE
    assert "const double waterSizeY{0.250}" in SOURCE
    assert "_mass / (this->waterDensity * this->waterSizeX * this->waterSizeY)" in SOURCE


def test_applied_ack_requires_successful_composite_and_mass_readback() -> None:
    apply_call = SOURCE.index("if (!this->ApplyCompositeInertial")
    apply_return = SOURCE.index("return;", apply_call)
    dry_ack = SOURCE.index("this->PublishApplied(this->dryAppliedPublisher", apply_return)
    water_ack = SOURCE.index("this->PublishApplied(this->waterAppliedPublisher", dry_ack)
    assert apply_call < apply_return < dry_ack < water_ack
    assert "_ecm.HasEntity(this->baseEntity)" in SOURCE
    assert "_ecm.HasEntity(this->modelEntity)" in SOURCE
    assert "modelSdf->Data(), this->dryFrameName" in SOURCE
    assert "modelSdf->Data(), this->waterFrameName" in SOURCE
    assert "const auto *readback" in SOURCE
    assert "std::abs(readback->Data().MassMatrix().Mass() - expectedMass)" in SOURCE


def test_physical_cube_compatibility_boundary_is_explicit() -> None:
    assert "A physically retained cube must not also be" in SOURCE
    assert "sent to the dry payload topic" in SOURCE


def test_physical_resident_mode_rejects_duplicate_aggregate_dry_mass() -> None:
    assert 'this->dryAccountingMode != "aggregate"' in SOURCE
    assert 'this->dryAccountingMode != "physical_resident"' in SOURCE
    assert "physical_resident mode requires initial_dry_mass_kg == 0" in SOURCE
    assert "if (this->physicalResidentDry)" in SOURCE
    assert "this->dryAggregateInputRejected.store(true)" in SOURCE
    assert '\\"aggregate_dry_input_rejected\\"' in SOURCE
    assert 'independent_rigid_bodies_contact' in SOURCE
    assert '<xacro:arg name="dry_accounting_mode" default="physical_resident"/>' in VEHICLE_XACRO
    assert '<dry_accounting_mode>$(arg dry_accounting_mode)</dry_accounting_mode>' in VEHICLE_XACRO
    assert 'LaunchConfiguration("dry_accounting_mode")' in LAUNCH_SOURCE
    assert '"dry_accounting_mode",' in LAUNCH_SOURCE


def test_initial_payload_is_clamped_and_shared_by_geometry_and_plugins() -> None:
    assert 'initial_dry_mass_kg>$(arg dry_load_mass_kg)<' in VEHICLE_XACRO
    assert (
        'initial_wastewater_mass_kg>$(arg wastewater_load_mass_kg)<'
        in VEHICLE_XACRO
    )
    assert 'initial_tank_mass_kg>$(arg wastewater_load_mass_kg)<' in VEHICLE_XACRO
    assert "max(min(float(dry_load_mass_kg), 1.512), 0.001)" in STORAGE_XACRO
    assert (
        "max(min(float(wastewater_load_mass_kg), 8.30), 0.001)"
        in STORAGE_XACRO
    )
    assert "std::clamp(initialDryMassKg, 0.0, this->dryCapacityKg)" in SOURCE
    assert "std::clamp(initialWaterMassKg, 0.0, this->waterCapacityKg)" in SOURCE
    assert "this->dryMassKg.store(initialDryMassKg)" in SOURCE
    assert "this->waterMassKg.store(initialWaterMassKg)" in SOURCE
    assert "this->tankMassKg, 0.0, this->tankCapacityKg" in WATER_SOURCE
    assert 'LaunchConfiguration("dry_load_mass_kg")' in LAUNCH_SOURCE
    assert 'LaunchConfiguration("wastewater_load_mass_kg")' in LAUNCH_SOURCE
