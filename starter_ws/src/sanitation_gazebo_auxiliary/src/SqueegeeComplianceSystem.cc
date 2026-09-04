// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include "sanitation_gazebo_auxiliary/SqueegeeComplianceCore.hh"

namespace sanitation_gazebo_auxiliary
{

/// Passive, preloaded two-axis compliance for the rear squeegee.
///
/// The vertical spring is biased below the nominal ground-tangent position so
/// that the ground reaction, rather than an artificial position lock, sets the
/// blade contact load.  Pitch compliance keeps the blade conformal while
/// retaining bounded force at both URDF joint limits.
class SqueegeeComplianceSystem final:
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
    if (!model.Valid(_ecm))
    {
      gzerr << "SqueegeeComplianceSystem must be attached to a model\n";
      return;
    }
    this->floatJoint = model.JointByName(_ecm, "squeegee_float_joint");
    this->pitchJoint = model.JointByName(_ecm, "squeegee_pitch_joint");
    if (this->floatJoint == gz::sim::kNullEntity ||
        this->pitchJoint == gz::sim::kNullEntity)
    {
      gzerr << "SqueegeeComplianceSystem cannot find both compliance joints\n";
      return;
    }
    ReadPositive(_sdf, "float_stiffness_n_per_m", this->floatParameters.stiffness);
    ReadNonNegative(_sdf, "float_damping_ns_per_m", this->floatParameters.damping);
    ReadFinite(_sdf, "float_preload_reference_m", this->floatParameters.reference);
    ReadPositive(_sdf, "float_max_force_n", this->floatParameters.maximumEffort);
    ReadPositive(_sdf, "pitch_stiffness_nm_per_rad", this->pitchParameters.stiffness);
    ReadNonNegative(_sdf, "pitch_damping_nms_per_rad", this->pitchParameters.damping);
    ReadFinite(_sdf, "pitch_reference_rad", this->pitchParameters.reference);
    ReadPositive(_sdf, "pitch_max_torque_nm", this->pitchParameters.maximumEffort);
    if (!SqueegeeComplianceCore::Valid(this->floatParameters) ||
        !SqueegeeComplianceCore::Valid(this->pitchParameters))
    {
      gzerr << "SqueegeeComplianceSystem received invalid spring parameters\n";
      return;
    }

    gz::sim::enableComponent<gz::sim::components::JointPosition>(
        _ecm, this->floatJoint, true);
    gz::sim::enableComponent<gz::sim::components::JointVelocity>(
        _ecm, this->floatJoint, true);
    gz::sim::enableComponent<gz::sim::components::JointPosition>(
        _ecm, this->pitchJoint, true);
    gz::sim::enableComponent<gz::sim::components::JointVelocity>(
        _ecm, this->pitchJoint, true);
    std::string prefix =
        "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance";
    if (_sdf && _sdf->HasElement("telemetry_topic_prefix"))
      prefix = _sdf->Get<std::string>("telemetry_topic_prefix");
    this->floatPositionPublisher = this->node.Advertise<gz::msgs::Double>(
        prefix + "/float_position_m");
    this->floatVelocityPublisher = this->node.Advertise<gz::msgs::Double>(
        prefix + "/float_velocity_m_s");
    this->floatForcePublisher = this->node.Advertise<gz::msgs::Double>(
        prefix + "/float_force_n");
    this->pitchPositionPublisher = this->node.Advertise<gz::msgs::Double>(
        prefix + "/pitch_position_rad");
    this->pitchVelocityPublisher = this->node.Advertise<gz::msgs::Double>(
        prefix + "/pitch_velocity_rad_s");
    this->pitchTorquePublisher = this->node.Advertise<gz::msgs::Double>(
        prefix + "/pitch_torque_nm");
    this->configured = true;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;
    const double floatPosition = Scalar(_ecm, this->floatJoint, false);
    const double floatVelocity = Scalar(_ecm, this->floatJoint, true);
    const double pitchPosition = Scalar(_ecm, this->pitchJoint, false);
    const double pitchVelocity = Scalar(_ecm, this->pitchJoint, true);
    const double floatForce = SqueegeeComplianceCore::Effort(
        this->floatParameters, floatPosition, floatVelocity);
    const double pitchTorque = SqueegeeComplianceCore::Effort(
        this->pitchParameters, pitchPosition, pitchVelocity);
    ApplyEffort(_ecm, this->floatJoint, floatForce);
    ApplyEffort(_ecm, this->pitchJoint, pitchTorque);

    // Publish measured state and the exact effort sent to physics at 50 Hz.
    // The formal validator consumes these live topics together with the blade
    // contact sensor; it never infers compliance from URDF tokens alone.
    if (_info.simTime < this->lastPublishTime ||
        _info.simTime - this->lastPublishTime >= std::chrono::milliseconds(20))
    {
      if (std::isfinite(floatForce) && std::isfinite(pitchTorque))
      {
        Publish(this->floatPositionPublisher, floatPosition);
        Publish(this->floatVelocityPublisher, floatVelocity);
        Publish(this->floatForcePublisher, floatForce);
        Publish(this->pitchPositionPublisher, pitchPosition);
        Publish(this->pitchVelocityPublisher, pitchVelocity);
        Publish(this->pitchTorquePublisher, pitchTorque);
      }
      this->lastPublishTime = _info.simTime;
    }
  }

  private: static void ReadPositive(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
    {
      const double candidate = _sdf->Get<double>(_name);
      if (std::isfinite(candidate) && candidate > 0.0)
        _value = candidate;
    }
  }

  private: static void ReadFinite(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
    {
      const double candidate = _sdf->Get<double>(_name);
      if (std::isfinite(candidate))
        _value = candidate;
    }
  }

  private: static void ReadNonNegative(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double &_value)
  {
    if (_sdf && _sdf->HasElement(_name))
    {
      const double candidate = _sdf->Get<double>(_name);
      if (std::isfinite(candidate) && candidate >= 0.0)
        _value = candidate;
    }
  }

  private: static double Scalar(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint,
      const bool _velocity)
  {
    if (_velocity)
    {
      const auto *component =
          _ecm.Component<gz::sim::components::JointVelocity>(_joint);
      return component != nullptr && !component->Data().empty() ?
          component->Data().front() : 0.0;
    }
    const auto *component =
        _ecm.Component<gz::sim::components::JointPosition>(_joint);
    return component != nullptr && !component->Data().empty() ?
        component->Data().front() : std::numeric_limits<double>::quiet_NaN();
  }

  private: static void ApplyEffort(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint,
      const double _effort)
  {
    if (!std::isfinite(_effort))
      return;
    const std::vector<double> command{_effort};
    auto *component =
        _ecm.Component<gz::sim::components::JointForceCmd>(_joint);
    if (component == nullptr)
      _ecm.CreateComponent(_joint, gz::sim::components::JointForceCmd(command));
    else
    {
      component->Data() = command;
      _ecm.SetChanged(
          _joint, gz::sim::components::JointForceCmd::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }
  }

  private: static void Publish(
      gz::transport::Node::Publisher &_publisher,
      const double _value)
  {
    gz::msgs::Double message;
    message.set_data(_value);
    _publisher.Publish(message);
  }

  private: gz::sim::Entity floatJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity pitchJoint{gz::sim::kNullEntity};
  private: bool configured{false};
  private: ComplianceAxisParameters floatParameters{1800.0, 45.0, -0.00692, 120.0};
  private: ComplianceAxisParameters pitchParameters{32.0, 6.0, 0.0, 24.0};
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher floatPositionPublisher;
  private: gz::transport::Node::Publisher floatVelocityPublisher;
  private: gz::transport::Node::Publisher floatForcePublisher;
  private: gz::transport::Node::Publisher pitchPositionPublisher;
  private: gz::transport::Node::Publisher pitchVelocityPublisher;
  private: gz::transport::Node::Publisher pitchTorquePublisher;
  private: std::chrono::steady_clock::duration lastPublishTime{0};
};

}  // namespace sanitation_gazebo_auxiliary

GZ_ADD_PLUGIN(
    sanitation_gazebo_auxiliary::SqueegeeComplianceSystem,
    gz::sim::System,
    sanitation_gazebo_auxiliary::SqueegeeComplianceSystem::ISystemConfigure,
    sanitation_gazebo_auxiliary::SqueegeeComplianceSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_auxiliary::SqueegeeComplianceSystem,
    "sanitation_gazebo_auxiliary::SqueegeeComplianceSystem")
