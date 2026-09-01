// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

// DART does not implement SDF mimic constraints.  This model system preserves
// the public one-joint Robotiq command API while applying bounded compliant
// efforts to the five physical follower joints.  Unlike a position reset, the
// effort-domain linkage continues to react to object contact and cannot
// teleport a finger through a grasped object.

#include <array>
#include <string>

#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>

#include "sanitation_gazebo_control/GripperMimicEffortCore.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr std::size_t kFollowerCount = 5;

constexpr std::array<const char *, kFollowerCount> kFollowerJointNames{
  "robotiq_85_right_knuckle_joint",
  "robotiq_85_left_inner_knuckle_joint",
  "robotiq_85_right_inner_knuckle_joint",
  "robotiq_85_left_finger_tip_joint",
  "robotiq_85_right_finger_tip_joint"};

constexpr std::array<double, kFollowerCount> kFollowerMultipliers{
  -1.0, 1.0, -1.0, -1.0, 1.0};
}  // namespace

class GripperMimicEffortSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    if (sdf->HasElement("master_joint")) {
      this->masterJointName = sdf->Get<std::string>("master_joint");
    }
    if (sdf->HasElement("position_gain_nm_rad")) {
      this->positionGainNmRad = sdf->Get<double>("position_gain_nm_rad");
    }
    if (sdf->HasElement("velocity_gain_nm_s_rad")) {
      this->velocityGainNmSRad = sdf->Get<double>("velocity_gain_nm_s_rad");
    }
    if (sdf->HasElement("maximum_effort_nm")) {
      this->maximumEffortNm = sdf->Get<double>("maximum_effort_nm");
    }

    const gz::sim::Model model(entity);
    this->masterJoint = model.JointByName(ecm, this->masterJointName);
    if (this->masterJoint == gz::sim::kNullEntity) {
      return;
    }
    gz::sim::enableComponent<gz::sim::components::JointPosition>(
      ecm, this->masterJoint, true);
    gz::sim::enableComponent<gz::sim::components::JointVelocity>(
      ecm, this->masterJoint, true);

    for (std::size_t index = 0; index < kFollowerCount; ++index) {
      this->followers[index] = model.JointByName(ecm, kFollowerJointNames[index]);
      if (this->followers[index] == gz::sim::kNullEntity) {
        return;
      }
      gz::sim::enableComponent<gz::sim::components::JointPosition>(
        ecm, this->followers[index], true);
      gz::sim::enableComponent<gz::sim::components::JointVelocity>(
        ecm, this->followers[index], true);
    }
    this->configured = true;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured || info.paused || info.dt.count() <= 0) {
      return;
    }

    const auto * masterPosition =
      ecm.Component<gz::sim::components::JointPosition>(this->masterJoint);
    const auto * masterVelocity =
      ecm.Component<gz::sim::components::JointVelocity>(this->masterJoint);
    if (masterPosition == nullptr || masterPosition->Data().empty() ||
        masterVelocity == nullptr || masterVelocity->Data().empty()) {
      return;
    }

    for (std::size_t index = 0; index < kFollowerCount; ++index) {
      const auto * followerPosition =
        ecm.Component<gz::sim::components::JointPosition>(this->followers[index]);
      const auto * followerVelocity =
        ecm.Component<gz::sim::components::JointVelocity>(this->followers[index]);
      if (followerPosition == nullptr || followerPosition->Data().empty() ||
          followerVelocity == nullptr || followerVelocity->Data().empty()) {
        continue;
      }

      GripperMimicEffortParameters parameters;
      parameters.multiplier = kFollowerMultipliers[index];
      parameters.position_gain_nm_rad = this->positionGainNmRad;
      parameters.velocity_gain_nm_s_rad = this->velocityGainNmSRad;
      parameters.maximum_effort_nm = this->maximumEffortNm;
      const auto output = ComputeGripperMimicEffort(
        parameters,
        masterPosition->Data().front(), masterVelocity->Data().front(),
        followerPosition->Data().front(), followerVelocity->Data().front());
      if (!output.valid) {
        continue;
      }

      auto * effort = ecm.Component<gz::sim::components::JointForceCmd>(
        this->followers[index]);
      if (effort == nullptr) {
        ecm.CreateComponent(
          this->followers[index],
          gz::sim::components::JointForceCmd({output.effort_nm}));
      } else {
        effort->Data() = {output.effort_nm};
      }
    }
  }

private:
  gz::sim::Entity masterJoint{gz::sim::kNullEntity};
  std::array<gz::sim::Entity, kFollowerCount> followers{};
  std::string masterJointName{"robotiq_85_left_knuckle_joint"};
  double positionGainNmRad{4.0};
  double velocityGainNmSRad{0.03};
  double maximumEffortNm{12.0};
  bool configured{false};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
  sanitation_gazebo_control::GripperMimicEffortSystem,
  gz::sim::System,
  sanitation_gazebo_control::GripperMimicEffortSystem::ISystemConfigure,
  sanitation_gazebo_control::GripperMimicEffortSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  sanitation_gazebo_control::GripperMimicEffortSystem,
  "sanitation_gazebo_control::GripperMimicEffortSystem")
