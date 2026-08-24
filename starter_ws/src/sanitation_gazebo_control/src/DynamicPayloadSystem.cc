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
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>

namespace sanitation_gazebo_control
{
/// Update the two dedicated payload-reserve links as material is collected.
///
/// The dry load is modeled as a uniformly distributed box in the dry bin. The
/// wastewater load is a rectangular liquid column whose height and centre of
/// mass rise with volume. Hydrodynamic slosh is intentionally not claimed by
/// this L1 plugin; the tank baffle remains explicit collision/visual geometry.
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
    const gz::sim::Model model(_entity);
    this->dryEntity = model.LinkByName(_ecm, "dry_bin_payload_reserve_link");
    this->waterEntity = model.LinkByName(_ecm, "wastewater_payload_reserve_link");

    if (_sdf->HasElement("dry_topic"))
      this->dryTopic = _sdf->Get<std::string>("dry_topic");
    if (_sdf->HasElement("wastewater_topic"))
      this->waterTopic = _sdf->Get<std::string>("wastewater_topic");
    if (_sdf->HasElement("dry_capacity_kg"))
      this->dryCapacityKg = _sdf->Get<double>("dry_capacity_kg");
    if (_sdf->HasElement("wastewater_capacity_kg"))
      this->waterCapacityKg = _sdf->Get<double>("wastewater_capacity_kg");

    this->node.Subscribe(this->dryTopic, &DynamicPayloadSystem::OnDryMass, this);
    this->node.Subscribe(this->waterTopic, &DynamicPayloadSystem::OnWaterMass, this);
    this->dryAppliedPublisher =
        this->node.Advertise<gz::msgs::Double>(this->dryTopic + "/applied");
    this->waterAppliedPublisher =
        this->node.Advertise<gz::msgs::Double>(this->waterTopic + "/applied");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    const auto dryRevision = this->dryRevision.load();
    if (dryRevision != this->appliedDryRevision)
    {
      const double mass = std::clamp(
          this->dryMassKg.load(), 0.0, this->dryCapacityKg);
      this->SetBoxLoad(
          _ecm, this->dryEntity,
          mass,
          this->drySizeX, this->drySizeY, this->drySizeZ,
          this->dryCentreZ);
      gz::msgs::Double applied;
      applied.set_data(mass);
      this->dryAppliedPublisher.Publish(applied);
      this->appliedDryRevision = dryRevision;
    }

    const auto waterRevision = this->waterRevision.load();
    if (waterRevision != this->appliedWaterRevision)
    {
      const double mass = std::clamp(
          this->waterMassKg.load(), 0.0, this->waterCapacityKg);
      const double height = std::max(
          mass / (this->waterDensity * this->waterSizeX * this->waterSizeY),
          this->minimumDimension);
      this->SetBoxLoad(
          _ecm, this->waterEntity, mass,
          this->waterSizeX, this->waterSizeY, height, height * 0.5);
      gz::msgs::Double applied;
      applied.set_data(mass);
      this->waterAppliedPublisher.Publish(applied);
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

  private: void SetBoxLoad(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _entity,
      const double _requestedMass,
      const double _sizeX,
      const double _sizeY,
      const double _sizeZ,
      const double _centreZ)
  {
    if (_entity == gz::sim::kNullEntity)
      return;
    auto component = _ecm.Component<gz::sim::components::Inertial>(_entity);
    if (nullptr == component)
      return;

    // Gazebo rejects a zero-mass dynamic link. The 1 g reserve is below the
    // smallest competition object and is removed from reported collected mass.
    const double mass = std::max(_requestedMass, this->minimumMassKg);
    const gz::math::Vector3d diagonal(
        mass * (_sizeY * _sizeY + _sizeZ * _sizeZ) / 12.0,
        mass * (_sizeX * _sizeX + _sizeZ * _sizeZ) / 12.0,
        mass * (_sizeX * _sizeX + _sizeY * _sizeY) / 12.0);
    const gz::math::MassMatrix3d matrix(
        mass, diagonal, gz::math::Vector3d::Zero);
    gz::math::Inertiald inertial;
    inertial.SetMassMatrix(matrix);
    inertial.SetPose(gz::math::Pose3d(0.0, 0.0, _centreZ, 0.0, 0.0, 0.0));
    component->Data() = inertial;
    _ecm.SetChanged(
        _entity, gz::sim::components::Inertial::typeId,
        gz::sim::ComponentState::OneTimeChange);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher dryAppliedPublisher;
  private: gz::transport::Node::Publisher waterAppliedPublisher;
  private: gz::sim::Entity dryEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity waterEntity{gz::sim::kNullEntity};
  private: std::string dryTopic{"/model/tzcup_formal_sanitation_vehicle/payload/dry_mass_kg"};
  private: std::string waterTopic{"/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg"};
  private: std::atomic<double> dryMassKg{0.0};
  private: std::atomic<double> waterMassKg{0.0};
  private: std::atomic<unsigned long> dryRevision{1};
  private: std::atomic<unsigned long> waterRevision{1};
  private: unsigned long appliedDryRevision{0};
  private: unsigned long appliedWaterRevision{0};
  private: double dryCapacityKg{1.512};
  private: double waterCapacityKg{9.7064};
  private: const double drySizeX{0.470};
  private: const double drySizeY{0.285};
  private: const double drySizeZ{0.298};
  private: const double dryCentreZ{0.137};
  private: const double waterSizeX{0.380};
  private: const double waterSizeY{0.250};
  private: const double waterDensity{1000.0};
  private: const double minimumMassKg{0.001};
  private: const double minimumDimension{0.00001};
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
