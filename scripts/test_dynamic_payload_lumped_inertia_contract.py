from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SOURCE = (
    ROOT
    / "starter_ws/src/sanitation_gazebo_control/src/DynamicPayloadSystem.cc"
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
    assert "2.0 * this->minimumMassKg" in SOURCE
    assert "this->structuralInertial.MassMatrix().Mass() + _dryMass + _waterMass" in SOURCE


def test_water_and_dry_boxes_match_current_xacro_envelopes() -> None:
    assert "const double drySizeX{0.485}" in SOURCE
    assert "const double drySizeY{0.355}" in SOURCE
    assert "const double drySizeZ{0.232}" in SOURCE
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
