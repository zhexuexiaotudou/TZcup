// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <string>

#include <gz/math/Inertial.hh>
#include <gz/math/MassMatrix3.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/transport/Node.hh>
#include <sdf/Frame.hh>

namespace sanitation_gazebo_control
{
namespace
{
bool FrameInBase(
    const sdf::Model &_modelSdf,
    const std::string &_frameName,
    const gz::math::Pose3d &_modelWorld,
    const gz::math::Pose3d &_baseWorld,
    gz::math::Pose3d &_frameInBase)
{
  const auto *frame = _modelSdf.FrameByName(_frameName);
  if (frame == nullptr)
    return false;

  gz::math::Pose3d frameInModel;
  if (!frame->SemanticPose().Resolve(frameInModel, "__model__").empty())
    return false;

  const auto frameWorld = _modelWorld * frameInModel;
  _frameInBase = _baseWorld.Inverse() * frameWorld;
  return true;
}

gz::math::Inertiald BoxInertial(
    const double _mass,
    const double _sizeX,
    const double _sizeY,
    const double _sizeZ,
    const gz::math::Pose3d &_pose)
{
  const gz::math::Vector3d diagonal(
      _mass * (_sizeY * _sizeY + _sizeZ * _sizeZ) / 12.0,
      _mass * (_sizeX * _sizeX + _sizeZ * _sizeZ) / 12.0,
      _mass * (_sizeX * _sizeX + _sizeY * _sizeY) / 12.0);
  return gz::math::Inertiald(
      gz::math::MassMatrix3d(_mass, diagonal, gz::math::Vector3d::Zero),
      _pose);
}
}

/// Update the dry and wastewater payload which sdformat fixed-joint reduction
/// lumps into base_footprint.
///
/// The two reserve links become SDF frames rather than physical links. The
/// original base inertial already contains their 1 g numerical-stability
/// masses, so Configure removes those two seed boxes once. Every later update
/// writes one composite inertial: structural baseline + current dry payload +
/// current wastewater payload. A physically retained cube must not also be
/// sent to the dry payload topic; that interface remains only for aggregate
/// loads which are not represented by separate rigid bodies.
class DynamicPayloadSystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->modelEntity = _entity;
    const gz::sim::Model model(_entity);
    this->baseEntity = model.LinkByName(_ecm, "base_footprint");

    if (_sdf->HasElement("dry_topic"))
      this->dryTopic = _sdf->Get<std::string>("dry_topic");
    if (_sdf->HasElement("wastewater_topic"))
      this->waterTopic = _sdf->Get<std::string>("wastewater_topic");
    if (_sdf->HasElement("dry_capacity_kg"))
      this->dryCapacityKg = _sdf->Get<double>("dry_capacity_kg");
    if (_sdf->HasElement("wastewater_capacity_kg"))
      this->waterCapacityKg = _sdf->Get<double>("wastewater_capacity_kg");

    if (this->baseEntity == gz::sim::kNullEntity)
      return;

    const auto *baseInertial =
        _ecm.Component<gz::sim::components::Inertial>(this->baseEntity);
    const auto *modelSdf =
        _ecm.Component<gz::sim::components::ModelSdf>(this->modelEntity);
    if (baseInertial == nullptr || modelSdf == nullptr)
      return;

    const auto modelWorld = gz::sim::worldPose(this->modelEntity, _ecm);
    const auto baseWorld = gz::sim::worldPose(this->baseEntity, _ecm);
    if (!FrameInBase(
            modelSdf->Data(), this->dryFrameName,
            modelWorld, baseWorld, this->dryFrameInBase) ||
        !FrameInBase(
            modelSdf->Data(), this->waterFrameName,
            modelWorld, baseWorld, this->waterFrameInBase))
      return;

    this->structuralInertial = baseInertial->Data();
    const auto initialDry = BoxInertial(
        this->minimumMassKg,
        this->drySizeX, this->drySizeY, this->drySizeZ,
        this->dryFrameInBase);
    const double initialWaterHeight = this->WaterHeight(this->minimumMassKg);
    const auto initialWater = BoxInertial(
        this->minimumMassKg,
        this->waterSizeX, this->waterSizeY, initialWaterHeight,
        this->waterFrameInBase * gz::math::Pose3d(
            0.0, 0.0, initialWaterHeight * 0.5, 0.0, 0.0, 0.0));
    this->structuralInertial -= initialDry;
    this->structuralInertial -= initialWater;

    const double expectedStructuralMass =
        baseInertial->Data().MassMatrix().Mass() - 2.0 * this->minimumMassKg;
    if (!this->structuralInertial.MassMatrix().IsValid() ||
        std::abs(this->structuralInertial.MassMatrix().Mass() -
            expectedStructuralMass) > this->massToleranceKg)
      return;

    this->node.Subscribe(this->dryTopic, &DynamicPayloadSystem::OnDryMass, this);
    this->node.Subscribe(
        this->waterTopic, &DynamicPayloadSystem::OnWaterMass, this);
    this->dryAppliedPublisher =
        this->node.Advertise<gz::msgs::Double>(this->dryTopic + "/applied");
    this->waterAppliedPublisher =
        this->node.Advertise<gz::msgs::Double>(this->waterTopic + "/applied");
    this->configured = true;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;

    const auto dryRevision = this->dryRevision.load();
    const auto waterRevision = this->waterRevision.load();
    const bool dryChanged = dryRevision != this->appliedDryRevision;
    const bool waterChanged = waterRevision != this->appliedWaterRevision;
    if (!dryChanged && !waterChanged)
      return;

    const double dryMass = std::clamp(
        this->dryMassKg.load(), 0.0, this->dryCapacityKg);
    const double waterMass = std::clamp(
        this->waterMassKg.load(), 0.0, this->waterCapacityKg);
    if (!this->ApplyCompositeInertial(_ecm, dryMass, waterMass))
      return;

    // Acknowledge only after the target, both frames and the base inertial
    // survived validation, the composite update was committed, and its mass
    // was read back from the ECM within tolerance.
    if (dryChanged)
    {
      this->PublishApplied(this->dryAppliedPublisher, dryMass);
      this->appliedDryRevision = dryRevision;
    }
    if (waterChanged)
    {
      this->PublishApplied(this->waterAppliedPublisher, waterMass);
      this->appliedWaterRevision = waterRevision;
    }
  }

  private: void OnDryMass(const gz::msgs::Double &_message)
  {
    this->dryMassKg.store(_message.data());
    this->dryRevision.fetch_add(1);
  }

  private: void OnWaterMass(const gz::msgs::Double &_message)
  {
    this->waterMassKg.store(_message.data());
    this->waterRevision.fetch_add(1);
  }

  private: double WaterHeight(const double _mass) const
  {
    return std::max(
        _mass / (this->waterDensity * this->waterSizeX * this->waterSizeY),
        this->minimumDimension);
  }

  private: bool ApplyCompositeInertial(
      gz::sim::EntityComponentManager &_ecm,
      const double _dryMass,
      const double _waterMass)
  {
    if (!_ecm.HasEntity(this->baseEntity) ||
        !_ecm.HasEntity(this->modelEntity))
      return false;

    auto *baseInertial =
        _ecm.Component<gz::sim::components::Inertial>(this->baseEntity);
    const auto *modelSdf =
        _ecm.Component<gz::sim::components::ModelSdf>(this->modelEntity);
    if (baseInertial == nullptr || modelSdf == nullptr)
      return false;

    const auto modelWorld = gz::sim::worldPose(this->modelEntity, _ecm);
    const auto baseWorld = gz::sim::worldPose(this->baseEntity, _ecm);
    gz::math::Pose3d dryFrameInBase;
    gz::math::Pose3d waterFrameInBase;
    if (!FrameInBase(
            modelSdf->Data(), this->dryFrameName,
            modelWorld, baseWorld, dryFrameInBase) ||
        !FrameInBase(
            modelSdf->Data(), this->waterFrameName,
            modelWorld, baseWorld, waterFrameInBase))
      return false;

    auto composite = this->structuralInertial;
    if (_dryMass > 0.0)
    {
      composite += BoxInertial(
          _dryMass,
          this->drySizeX, this->drySizeY, this->drySizeZ,
          dryFrameInBase);
    }
    if (_waterMass > 0.0)
    {
      const double height = this->WaterHeight(_waterMass);
      composite += BoxInertial(
          _waterMass,
          this->waterSizeX, this->waterSizeY, height,
          waterFrameInBase * gz::math::Pose3d(
              0.0, 0.0, height * 0.5, 0.0, 0.0, 0.0));
    }
    if (!composite.MassMatrix().IsValid())
      return false;

    baseInertial->Data() = composite;
    _ecm.SetChanged(
        this->baseEntity, gz::sim::components::Inertial::typeId,
        gz::sim::ComponentState::OneTimeChange);

    const auto *readback =
        _ecm.Component<gz::sim::components::Inertial>(this->baseEntity);
    const double expectedMass =
        this->structuralInertial.MassMatrix().Mass() + _dryMass + _waterMass;
    return readback != nullptr &&
        std::abs(readback->Data().MassMatrix().Mass() - expectedMass) <=
            this->massToleranceKg;
  }

  private: static void PublishApplied(
      gz::transport::Node::Publisher &_publisher,
      const double _mass)
  {
    gz::msgs::Double applied;
    applied.set_data(_mass);
    _publisher.Publish(applied);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher dryAppliedPublisher;
  private: gz::transport::Node::Publisher waterAppliedPublisher;
  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity baseEntity{gz::sim::kNullEntity};
  private: gz::math::Pose3d dryFrameInBase;
  private: gz::math::Pose3d waterFrameInBase;
  private: gz::math::Inertiald structuralInertial;
  private: const std::string dryFrameName{"dry_bin_payload_reserve_link"};
  private: const std::string waterFrameName{
      "wastewater_payload_reserve_link"};
  private: std::string dryTopic{
      "/model/tzcup_formal_sanitation_vehicle/payload/dry_mass_kg"};
  private: std::string waterTopic{
      "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg"};
  private: std::atomic<double> dryMassKg{0.0};
  private: std::atomic<double> waterMassKg{0.0};
  private: std::atomic<unsigned long> dryRevision{1};
  private: std::atomic<unsigned long> waterRevision{1};
  private: unsigned long appliedDryRevision{0};
  private: unsigned long appliedWaterRevision{0};
  private: bool configured{false};
  private: double dryCapacityKg{1.512};
  private: double waterCapacityKg{9.7064};
  private: const double drySizeX{0.485};
  private: const double drySizeY{0.355};
  private: const double drySizeZ{0.232};
  private: const double waterSizeX{0.350};
  private: const double waterSizeY{0.250};
  private: const double waterDensity{1000.0};
  private: const double minimumMassKg{0.001};
  private: const double minimumDimension{0.00001};
  private: const double massToleranceKg{1e-9};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
    sanitation_gazebo_control::DynamicPayloadSystem,
    gz::sim::System,
    sanitation_gazebo_control::DynamicPayloadSystem::ISystemConfigure,
    sanitation_gazebo_control::DynamicPayloadSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_control::DynamicPayloadSystem,
    "sanitation_gazebo_control::DynamicPayloadSystem")
