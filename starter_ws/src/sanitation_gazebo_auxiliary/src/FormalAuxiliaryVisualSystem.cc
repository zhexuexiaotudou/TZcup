// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Angle.hh>
#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Conversions.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/VisualCmd.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>
#include <sdf/Light.hh>

#include "sanitation_gazebo_auxiliary/EstopLatchCore.hh"
#include "sanitation_gazebo_auxiliary/LightingCore.hh"

namespace sanitation_gazebo_auxiliary
{
namespace
{
std::string StringParameter(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    const std::string &_fallback)
{
  return _sdf->HasElement(_name) ? _sdf->Get<std::string>(_name) : _fallback;
}

double DoubleParameter(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    const double _fallback)
{
  return _sdf->HasElement(_name) ? _sdf->Get<double>(_name) : _fallback;
}

bool BoolParameter(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    const bool _fallback)
{
  return _sdf->HasElement(_name) ? _sdf->Get<bool>(_name) : _fallback;
}
}  // namespace

/// Gazebo System plugin which applies product lighting state to existing
/// vehicle visuals and exposes a fail-closed physical E-stop latch on Gazebo
/// Transport. The formal launch bridges its latched output one-way into the
/// sole ROS /emergency_stop state topic.
class FormalAuxiliaryVisualSystem final:
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
      gzerr << "FormalAuxiliaryVisualSystem must be attached to a model\n";
      return;
    }
    this->modelEntity = _entity;
    this->plungerJointName = StringParameter(
        _sdf, "plunger_joint_name", "emergency_stop_plunger_joint");
    this->plungerTravel = DoubleParameter(_sdf, "plunger_travel_m", 0.006);
    this->plungerPressedThreshold = DoubleParameter(
        _sdf, "plunger_pressed_threshold_m", 0.005);
    this->plungerReleasedClearance = DoubleParameter(
        _sdf, "plunger_released_clearance_m", 0.00002);
    this->plungerPositionGain = DoubleParameter(
        _sdf, "plunger_position_gain_n_m", 1500.0);
    this->plungerDampingGain = DoubleParameter(
        _sdf, "plunger_damping_gain_n_s_m", 15.0);
    this->plungerMaximumForce = DoubleParameter(
        _sdf, "plunger_maximum_force_n", 20.0);
    this->isolatorJointName = StringParameter(
        _sdf, "isolator_joint_name", "main_power_isolator_handle_joint");
    this->contactorJointName = StringParameter(
        _sdf, "contactor_joint_name", "main_power_contactor_armature_joint");
    this->isolatorTravel = DoubleParameter(
        _sdf, "isolator_travel_rad", 1.5707963267948966);
    this->isolatorClosedThreshold = DoubleParameter(
        _sdf, "isolator_closed_threshold_rad", 1.40);
    this->isolatorOpenClearance = DoubleParameter(
        _sdf, "isolator_open_clearance_rad", 0.002);
    this->contactorTravel = DoubleParameter(
        _sdf, "contactor_travel_m", 0.004);
    this->contactorClosedThreshold = DoubleParameter(
        _sdf, "contactor_closed_threshold_m", 0.0035);
    this->contactorOpenClearance = DoubleParameter(
        _sdf, "contactor_open_clearance_m", 0.00002);
    if (!std::isfinite(this->plungerTravel) || this->plungerTravel <= 0.0 ||
        !std::isfinite(this->plungerPressedThreshold) ||
        this->plungerPressedThreshold <= 0.0 ||
        this->plungerPressedThreshold > this->plungerTravel ||
        !std::isfinite(this->plungerReleasedClearance) ||
        this->plungerReleasedClearance < 0.0 ||
        this->plungerReleasedClearance >= this->plungerPressedThreshold ||
        !std::isfinite(this->plungerPositionGain) ||
        this->plungerPositionGain <= 0.0 ||
        !std::isfinite(this->plungerDampingGain) ||
        this->plungerDampingGain < 0.0 ||
        !std::isfinite(this->plungerMaximumForce) ||
        this->plungerMaximumForce <= 0.0 ||
        !std::isfinite(this->isolatorTravel) || this->isolatorTravel <= 0.0 ||
        !std::isfinite(this->isolatorClosedThreshold) ||
        this->isolatorClosedThreshold <= 0.0 ||
        this->isolatorClosedThreshold > this->isolatorTravel ||
        !std::isfinite(this->isolatorOpenClearance) ||
        this->isolatorOpenClearance < 0.0 ||
        this->isolatorOpenClearance >= this->isolatorClosedThreshold ||
        !std::isfinite(this->contactorTravel) || this->contactorTravel <= 0.0 ||
        !std::isfinite(this->contactorClosedThreshold) ||
        this->contactorClosedThreshold <= 0.0 ||
        this->contactorClosedThreshold > this->contactorTravel ||
        !std::isfinite(this->contactorOpenClearance) ||
        this->contactorOpenClearance < 0.0 ||
        this->contactorOpenClearance >= this->contactorClosedThreshold)
    {
      gzerr << "Invalid emergency-stop plunger configuration\n";
      return;
    }
    this->plungerJoint = model.JointByName(_ecm, this->plungerJointName);
    this->isolatorJoint = model.JointByName(_ecm, this->isolatorJointName);
    this->contactorJoint = model.JointByName(_ecm, this->contactorJointName);
    if (this->plungerJoint == gz::sim::kNullEntity ||
        this->isolatorJoint == gz::sim::kNullEntity ||
        this->contactorJoint == gz::sim::kNullEntity)
    {
      gzerr << "FormalAuxiliaryVisualSystem cannot find required joints "
            << this->plungerJointName << ", " << this->isolatorJointName
            << ", " << this->contactorJointName << "\n";
      return;
    }
    for (const auto joint : {
        this->plungerJoint, this->isolatorJoint, this->contactorJoint})
    {
      gz::sim::enableComponent<gz::sim::components::JointPosition>(
          _ecm, joint, true);
      gz::sim::enableComponent<gz::sim::components::JointVelocity>(
          _ecm, joint, true);
    }
    // Initialise passive switchgear just clear of the lower hard stops.
    // DART's unilateral limit solver can keep a zero-position joint locked
    // even when force is commanded away from the stop.  These sub-threshold
    // mechanical clearances model real free play and retain fail-open state.
    _ecm.CreateComponent(
        this->plungerJoint,
        gz::sim::components::JointPositionReset(
        std::vector<double>{this->plungerReleasedClearance}));
    _ecm.CreateComponent(
        this->isolatorJoint,
        gz::sim::components::JointPositionReset(
        std::vector<double>{this->isolatorOpenClearance}));
    _ecm.CreateComponent(
        this->contactorJoint,
        gz::sim::components::JointPositionReset(
        std::vector<double>{this->contactorOpenClearance}));

    LightingConfig lightingConfig;
    lightingConfig.warningFrequencyHz =
        DoubleParameter(_sdf, "warning_frequency_hz", 1.0);
    lightingConfig.warningDutyCycle =
        DoubleParameter(_sdf, "warning_duty_cycle", 0.5);
    try
    {
      this->lightingCore = std::make_unique<LightingCore>(lightingConfig);
    }
    catch (const std::invalid_argument &_error)
    {
      gzerr << "Invalid lighting configuration: " << _error.what() << "\n";
      return;
    }
    this->estopCore = std::make_unique<EstopLatchCore>(
        BoolParameter(_sdf, "initial_estop_latched", true));

    this->photometricEnabled =
        BoolParameter(_sdf, "enable_photometric_lights", true);
    this->workLightIntensity =
        DoubleParameter(_sdf, "work_light_intensity", 1.8);
    this->workLightRange =
        DoubleParameter(_sdf, "work_light_range_m", 12.0);
    this->workSpotInnerAngle =
        DoubleParameter(_sdf, "work_spot_inner_angle_rad", 0.34);
    this->workSpotOuterAngle =
        DoubleParameter(_sdf, "work_spot_outer_angle_rad", 0.62);
    this->warningLightIntensity =
        DoubleParameter(_sdf, "warning_light_intensity", 0.55);
    this->warningLightRange =
        DoubleParameter(_sdf, "warning_light_range_m", 2.5);
    this->tailLightIntensity =
        DoubleParameter(_sdf, "tail_light_intensity", 0.18);
    this->tailLightRange =
        DoubleParameter(_sdf, "tail_light_range_m", 1.5);
    if (this->workLightIntensity < 0.0 || this->warningLightIntensity < 0.0 ||
        this->tailLightIntensity < 0.0 ||
        this->workLightRange <= 0.0 || this->warningLightRange <= 0.0 ||
        this->tailLightRange <= 0.0 ||
        this->workSpotInnerAngle <= 0.0 ||
        this->workSpotOuterAngle <= this->workSpotInnerAngle ||
        this->workSpotOuterAngle >= 3.14159265358979323846)
    {
      gzerr << "Invalid photometric light configuration\n";
      return;
    }

    this->visualGroups = {
      {"front_work_light_left_visual", {gz::math::Color(0.90, 0.96, 1.0, 1.0), Group::Work}},
      {"front_work_light_right_visual", {gz::math::Color(0.90, 0.96, 1.0, 1.0), Group::Work}},
      {"rear_tail_light_left_visual", {gz::math::Color(0.75, 0.015, 0.020, 1.0), Group::Tail}},
      {"rear_tail_light_right_visual", {gz::math::Color(0.75, 0.015, 0.020, 1.0), Group::Tail}},
      {"corner_beacons_visual", {gz::math::Color(1.0, 0.45, 0.015, 1.0), Group::Warning}},
    };

    this->workTopic = StringParameter(
        _sdf, "work_topic", "/formal_vehicle/lighting/work_lights_on");
    this->tailTopic = StringParameter(
        _sdf, "tail_topic", "/formal_vehicle/lighting/tail_lights_on");
    this->warningTopic = StringParameter(
        _sdf, "warning_topic", "/formal_vehicle/lighting/warning_lights_on");
    this->externalEstopTopic = StringParameter(
        _sdf, "external_estop_topic", "/formal_vehicle/simulation/command/emergency_stop");
    this->physicalButtonTopic = StringParameter(
        _sdf, "physical_button_topic", "/formal_vehicle/simulation/command/emergency_stop_plunger_pressed");
    this->resetTopic = StringParameter(
        _sdf, "reset_topic", "/formal_vehicle/simulation/command/emergency_stop_reset");
    this->safetyPowerTopic = StringParameter(
        _sdf, "safety_power_topic", "/formal_vehicle/power/branches/safety/enabled");
    this->mainPowerRequestTopic = StringParameter(
        _sdf, "main_power_request_topic",
        "/formal_vehicle/simulation/command/main_power");
    this->contactorCommandTopic = StringParameter(
        _sdf, "contactor_command_topic",
        "/formal_vehicle/power/main_contactor_command");
    this->isolatorStateTopic = StringParameter(
        _sdf, "isolator_state_topic",
        "/formal_vehicle/power/main_isolator_closed");
    this->contactorStateTopic = StringParameter(
        _sdf, "contactor_state_topic",
        "/formal_vehicle/power/main_contactor_closed");
    this->latchedStateTopic = StringParameter(
        _sdf, "latched_state_topic", "/emergency_stop");
    this->workAppliedTopic = StringParameter(
        _sdf, "work_applied_topic", this->workTopic + "/applied");
    this->tailAppliedTopic = StringParameter(
        _sdf, "tail_applied_topic", this->tailTopic + "/applied");
    this->warningAppliedTopic = StringParameter(
        _sdf, "warning_applied_topic", this->warningTopic + "/applied");

    bool subscriptionsOk = true;
    subscriptionsOk &= this->node.Subscribe(
        this->workTopic, &FormalAuxiliaryVisualSystem::OnWork, this);
    subscriptionsOk &= this->node.Subscribe(
        this->tailTopic, &FormalAuxiliaryVisualSystem::OnTail, this);
    subscriptionsOk &= this->node.Subscribe(
        this->warningTopic, &FormalAuxiliaryVisualSystem::OnWarning, this);
    subscriptionsOk &= this->node.Subscribe(
        this->externalEstopTopic,
        &FormalAuxiliaryVisualSystem::OnExternalEstop, this);
    subscriptionsOk &= this->node.Subscribe(
        this->physicalButtonTopic,
        &FormalAuxiliaryVisualSystem::OnPhysicalButton, this);
    subscriptionsOk &= this->node.Subscribe(
        this->resetTopic, &FormalAuxiliaryVisualSystem::OnReset, this);
    subscriptionsOk &= this->node.Subscribe(
        this->safetyPowerTopic,
        &FormalAuxiliaryVisualSystem::OnSafetyPower, this);
    subscriptionsOk &= this->node.Subscribe(
        this->mainPowerRequestTopic,
        &FormalAuxiliaryVisualSystem::OnMainPowerRequest, this);
    subscriptionsOk &= this->node.Subscribe(
        this->contactorCommandTopic,
        &FormalAuxiliaryVisualSystem::OnContactorCommand, this);
    if (!subscriptionsOk)
    {
      gzerr << "FormalAuxiliaryVisualSystem failed to subscribe to one or more topics\n";
      return;
    }

    this->latchedPublisher =
        this->node.Advertise<gz::msgs::Boolean>(this->latchedStateTopic);
    this->workAppliedPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->workAppliedTopic);
    this->tailAppliedPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->tailAppliedTopic);
    this->warningAppliedPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->warningAppliedTopic);
    this->isolatorStatePublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->isolatorStateTopic);
    this->contactorStatePublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->contactorStateTopic);
    this->configured = true;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;

    if (!this->photometricLightsCreated &&
        !this->CreatePhotometricLights(_ecm))
    {
      return;
    }

    // The ROS test/operator command moves the real 6 mm prismatic joint. The
    // measured joint position, not the command alone, is the physical switch
    // input; command assertion is also included so a physics delay cannot
    // postpone an emergency stop by one integration step.
    const bool commandedPressed = this->physicalButtonPressed.load();
    const auto *position =
        _ecm.Component<gz::sim::components::JointPosition>(this->plungerJoint);
    const bool positionValid =
        position != nullptr && !position->Data().empty() &&
        std::isfinite(position->Data().front());
    if (positionValid)
    {
      this->CommandJoint(
          _ecm, this->plungerJoint,
          commandedPressed ? this->plungerTravel : this->plungerReleasedClearance,
          position->Data().front(), this->plungerTravel,
          this->plungerPositionGain, this->plungerDampingGain,
          this->plungerMaximumForce, 0.02,
          std::chrono::duration<double>(_info.dt).count());
    }
    // Loss of joint feedback is an electrical open-circuit equivalent and
    // therefore asserted. A healthy released switch requires a valid reading.
    const bool physicalPressed = !positionValid || commandedPressed ||
        position->Data().front() >= this->plungerPressedThreshold;
    const bool emergencyInput = this->externalEstop.load() || physicalPressed;
    const bool resetRequested = this->resetRequested.exchange(false);
    const bool latched = this->estopCore->Update(
        emergencyInput, resetRequested, this->safetyPowerAvailable.load());

    // Operator intent drives a real 90 degree isolator shaft.  The downstream
    // normally-open contactor can close only after the measured shaft reaches
    // ON and the independently computed safety command is present.  The
    // physical E-stop latch is repeated here as a hard plugin-side cut so a
    // stale or malicious topic cannot hold the armature closed.
    const double isolatorPosition = this->JointScalar(
        _ecm, this->isolatorJoint, false);
    const bool isolatorPositionValid = std::isfinite(isolatorPosition);
    if (isolatorPositionValid)
    {
      this->CommandJoint(
          _ecm, this->isolatorJoint,
          this->mainPowerRequested.load() ? this->isolatorTravel :
          this->isolatorOpenClearance,
          isolatorPosition, this->isolatorTravel,
          this->isolatorPositionGain, this->isolatorDampingGain,
          this->isolatorMaximumTorque, 1.2,
          std::chrono::duration<double>(_info.dt).count());
    }
    const bool isolatorClosed = isolatorPositionValid &&
        isolatorPosition >= this->isolatorClosedThreshold;
    const bool contactorPermitted = isolatorClosed && !latched &&
        this->safetyPowerAvailable.load() &&
        this->contactorCommanded.load();
    const double contactorPosition = this->JointScalar(
        _ecm, this->contactorJoint, false);
    const bool contactorPositionValid = std::isfinite(contactorPosition);
    if (contactorPositionValid)
    {
      this->CommandJoint(
          _ecm, this->contactorJoint,
          contactorPermitted ? this->contactorTravel :
          this->contactorOpenClearance,
          contactorPosition, this->contactorTravel,
          this->contactorPositionGain, this->contactorDampingGain,
          this->contactorMaximumForce, 0.08,
          std::chrono::duration<double>(_info.dt).count());
    }
    const bool contactorClosed = contactorPermitted &&
        contactorPositionValid &&
        contactorPosition >= this->contactorClosedThreshold;

    LightingInputs inputs;
    inputs.workRequested = this->workRequested.load();
    inputs.tailRequested = this->tailRequested.load();
    inputs.warningRequested = this->warningRequested.load();
    inputs.emergencyStopLatched = latched;
    inputs.safetyPowerAvailable = this->safetyPowerAvailable.load();
    const double simulationSeconds =
        std::chrono::duration<double>(_info.simTime).count();
    const auto outputs = this->lightingCore->Evaluate(simulationSeconds, inputs);

    if (!this->haveAppliedOutputs || outputs != this->appliedOutputs)
    {
      const auto matched = this->ApplyOutputs(_ecm, outputs);
      const bool photometricApplied =
          this->ApplyPhotometricOutputs(_ecm, outputs);
      if (!this->lightingBindingDiagnosticPublished && simulationSeconds > 1.0 &&
          matched != this->visualGroups.size())
      {
        gzerr << "Formal auxiliary lighting binding incomplete: matched "
              << matched << " of " << this->visualGroups.size()
              << " named lamp visuals\n";
        this->lightingBindingDiagnosticPublished = true;
      }
      if (matched == this->visualGroups.size() && photometricApplied)
      {
        this->appliedOutputs = outputs;
        this->haveAppliedOutputs = true;
      }
    }
    if (this->haveAppliedOutputs)
    {
      this->Publish(this->workAppliedPublisher, this->appliedOutputs.workOn);
      this->Publish(this->tailAppliedPublisher, this->appliedOutputs.tailOn);
      this->Publish(
          this->warningAppliedPublisher, this->appliedOutputs.warningOn);
    }
    // Publish every physics update. This is the safety heartbeat as well as
    // the state edge, so a bridge or consumer started after model insertion
    // still receives the fail-closed initial latch.
    this->Publish(this->latchedPublisher, latched);
    this->Publish(this->isolatorStatePublisher, isolatorClosed);
    this->Publish(this->contactorStatePublisher, contactorClosed);
  }

  private: double JointScalar(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint,
      const bool _velocity) const
  {
    if (_velocity)
    {
      const auto *component =
          _ecm.Component<gz::sim::components::JointVelocity>(_joint);
      return component != nullptr && !component->Data().empty() &&
          std::isfinite(component->Data().front()) ?
          component->Data().front() : 0.0;
    }
    const auto *component =
        _ecm.Component<gz::sim::components::JointPosition>(_joint);
    return component != nullptr && !component->Data().empty() &&
        std::isfinite(component->Data().front()) ?
        component->Data().front() :
        std::numeric_limits<double>::quiet_NaN();
  }

  private: void CommandJoint(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint,
      const double _targetPosition,
      const double _measuredPosition,
      const double _travel,
      const double _positionGain,
      const double _dampingGain,
      const double _maximumForce,
      const double _maximumVelocity,
      const double _stepSeconds)
  {
    const double measuredVelocity = this->JointScalar(_ecm, _joint, true);
    // A bounded spring/damper drive preserves real joint dynamics and permits
    // contact forces to depress the mushroom; it does not teleport the joint.
    const double force = std::clamp(
        _positionGain *
            (std::clamp(_targetPosition, 0.0, _travel) -
             _measuredPosition) -
            _dampingGain * measuredVelocity,
        -_maximumForce,
        _maximumForce);
    // DART ignores effort and velocity command components on joints exported
    // as state-only by gz_ros2_control.  Advance the measured physical joint
    // by a speed-bounded reset step instead of teleporting to the target.
    // Electrical state still depends on measured travel and all interlocks.
    const double maxStep = std::max(0.0, _maximumVelocity * _stepSeconds);
    const double step = std::abs(force) <= 1.0e-12 ? 0.0 : std::clamp(
        _targetPosition - _measuredPosition, -maxStep, maxStep);
    const std::vector<double> command{_measuredPosition + step};
    auto *component =
        _ecm.Component<gz::sim::components::JointPositionReset>(
            _joint);
    if (component == nullptr)
    {
      _ecm.CreateComponent(
          _joint,
          gz::sim::components::JointPositionReset(command));
    }
    else if (component->Data() != command)
    {
      component->Data() = command;
    }
  }

  private: enum class Group {Work, Tail, Warning};

  private: struct VisualSpec
  {
    gz::math::Color emissiveColor;
    Group group;
  };

  private: struct PhotometricLight
  {
    gz::sim::Entity entity{gz::sim::kNullEntity};
    Group group{Group::Work};
  };

  private: sdf::Light WorkSpot(
      const std::string &_name,
      const gz::math::Pose3d &_pose) const
  {
    sdf::Light light;
    light.SetName(_name);
    light.SetType(sdf::LightType::SPOT);
    light.SetRawPose(_pose);
    light.SetLightOn(false);
    // Gazebo Harmonic only allocates a light-visual entity when this is true.
    // LightCmd later hides that helper visual; allocating it prevents the
    // upstream RenderUtil from recording entity 0 and flooding SceneManager.
    light.SetVisualize(true);
    light.SetCastShadows(false);
    light.SetIntensity(this->workLightIntensity);
    light.SetDiffuse(gz::math::Color(0.90, 0.96, 1.0, 1.0));
    light.SetSpecular(gz::math::Color(0.22, 0.24, 0.25, 1.0));
    light.SetAttenuationRange(this->workLightRange);
    light.SetConstantAttenuationFactor(0.25);
    light.SetLinearAttenuationFactor(0.02);
    light.SetQuadraticAttenuationFactor(0.01);
    light.SetDirection(gz::math::Vector3d(1.0, 0.0, -0.16));
    light.SetSpotInnerAngle(gz::math::Angle(this->workSpotInnerAngle));
    light.SetSpotOuterAngle(gz::math::Angle(this->workSpotOuterAngle));
    light.SetSpotFalloff(0.8);
    return light;
  }

  private: sdf::Light WarningPoint(
      const std::string &_name,
      const gz::math::Pose3d &_pose) const
  {
    sdf::Light light;
    light.SetName(_name);
    light.SetType(sdf::LightType::POINT);
    light.SetRawPose(_pose);
    light.SetLightOn(false);
    light.SetVisualize(true);
    light.SetCastShadows(false);
    light.SetIntensity(this->warningLightIntensity);
    light.SetDiffuse(gz::math::Color(1.0, 0.36, 0.01, 1.0));
    light.SetSpecular(gz::math::Color(0.20, 0.07, 0.0, 1.0));
    light.SetAttenuationRange(this->warningLightRange);
    light.SetConstantAttenuationFactor(0.35);
    light.SetLinearAttenuationFactor(0.08);
    light.SetQuadraticAttenuationFactor(0.02);
    return light;
  }

  private: sdf::Light TailPoint(
      const std::string &_name,
      const gz::math::Pose3d &_pose) const
  {
    sdf::Light light;
    light.SetName(_name);
    light.SetType(sdf::LightType::POINT);
    light.SetRawPose(_pose);
    light.SetLightOn(false);
    light.SetVisualize(true);
    light.SetCastShadows(false);
    light.SetIntensity(this->tailLightIntensity);
    light.SetDiffuse(gz::math::Color(0.82, 0.01, 0.015, 1.0));
    light.SetSpecular(gz::math::Color(0.12, 0.005, 0.005, 1.0));
    light.SetAttenuationRange(this->tailLightRange);
    light.SetConstantAttenuationFactor(0.45);
    light.SetLinearAttenuationFactor(0.12);
    light.SetQuadraticAttenuationFactor(0.04);
    return light;
  }

  private: bool AddPhotometricLight(
      gz::sim::EntityComponentManager &_ecm,
      const sdf::Light &_light,
      const gz::math::Pose3d &_pose,
      const Group _group)
  {
    const auto entity = _ecm.CreateEntity();
    _ecm.CreateComponent(
        entity, gz::sim::components::Name(_light.Name()));
    _ecm.CreateComponent(
        entity, gz::sim::components::ParentEntity(this->modelEntity));
    _ecm.CreateComponent(entity, gz::sim::components::Pose(_pose));
    _ecm.CreateComponent(entity, gz::sim::components::Light(_light));

    const bool complete =
        _ecm.Component<gz::sim::components::Name>(entity) != nullptr &&
        _ecm.Component<gz::sim::components::ParentEntity>(entity) != nullptr &&
        _ecm.Component<gz::sim::components::Pose>(entity) != nullptr &&
        _ecm.Component<gz::sim::components::Light>(entity) != nullptr;
    if (!complete)
    {
      gzerr << "Failed to create photometric light " << _light.Name() << "\n";
      return false;
    }
    this->photometricLights.push_back({entity, _group});
    return true;
  }

  private: bool CreatePhotometricLights(
      gz::sim::EntityComponentManager &_ecm)
  {
    if (!this->photometricEnabled)
    {
      this->photometricLightsCreated = true;
      return true;
    }
    // A partial creation is a hard configuration failure. Do not create
    // duplicate lights on later simulation ticks.
    if (!this->photometricLights.empty())
      return false;

    // Poses are in the formal vehicle model frame. ParentEntity makes both
    // positions and spot directions follow vehicle translation and attitude.
    const std::vector<std::pair<sdf::Light, gz::math::Pose3d>> workSpots = {
      {this->WorkSpot(
          "formal_front_work_spot_left",
          gz::math::Pose3d(0.535, 0.225, 0.435, 0.0, 0.0, 0.0)),
       gz::math::Pose3d(0.535, 0.225, 0.435, 0.0, 0.0, 0.0)},
      {this->WorkSpot(
          "formal_front_work_spot_right",
          gz::math::Pose3d(0.535, -0.225, 0.435, 0.0, 0.0, 0.0)),
       gz::math::Pose3d(0.535, -0.225, 0.435, 0.0, 0.0, 0.0)},
    };
    for (const auto &entry : workSpots)
    {
      if (!this->AddPhotometricLight(
          _ecm, entry.first, entry.second, Group::Work))
      {
        return false;
      }
    }

    const std::vector<gz::math::Pose3d> warningPoses = {
      {0.420, 0.340, 0.620, 0.0, 0.0, 0.0},
      {0.420, -0.340, 0.620, 0.0, 0.0, 0.0},
      {-0.420, 0.340, 0.620, 0.0, 0.0, 0.0},
      {-0.420, -0.340, 0.620, 0.0, 0.0, 0.0},
    };
    for (std::size_t index = 0; index < warningPoses.size(); ++index)
    {
      const auto name = "formal_warning_point_" + std::to_string(index);
      const auto light = this->WarningPoint(name, warningPoses[index]);
      if (!this->AddPhotometricLight(
          _ecm, light, warningPoses[index], Group::Warning))
      {
        return false;
      }
    }
    const std::vector<gz::math::Pose3d> tailPoses = {
      {-0.515, 0.225, 0.430, 0.0, 0.0, 0.0},
      {-0.515, -0.225, 0.430, 0.0, 0.0, 0.0},
    };
    for (std::size_t index = 0; index < tailPoses.size(); ++index)
    {
      const auto name = "formal_tail_point_" + std::to_string(index);
      const auto light = this->TailPoint(name, tailPoses[index]);
      if (!this->AddPhotometricLight(
          _ecm, light, tailPoses[index], Group::Tail))
      {
        return false;
      }
    }
    this->photometricLightsCreated = true;
    return true;
  }

  private: bool ApplyPhotometricOutputs(
      gz::sim::EntityComponentManager &_ecm,
      const LightingOutputs &_outputs)
  {
    if (!this->photometricEnabled)
      return true;
    if (!this->photometricLightsCreated || this->photometricLights.size() != 8u)
      return false;

    for (const auto &entry : this->photometricLights)
    {
      auto *light =
          _ecm.Component<gz::sim::components::Light>(entry.entity);
      if (light == nullptr)
        return false;
      const bool on = entry.group == Group::Work ? _outputs.workOn :
          entry.group == Group::Tail ? _outputs.tailOn : _outputs.warningOn;
      auto commandedLight = light->Data();
      commandedLight.SetLightOn(on);
      auto message = gz::sim::convert<gz::msgs::Light>(commandedLight);
      // RenderUtil targets the ECM entity that owns LightCmd.  Preserve the
      // same non-zero ID in the payload as self-describing metadata; it does
      // not replace the component-owner identity.
      message.set_id(entry.entity);
      message.set_parent_id(this->modelEntity);
      message.set_visualize_visual(false);
      auto *command =
          _ecm.Component<gz::sim::components::LightCmd>(entry.entity);
      if (command == nullptr)
      {
        _ecm.CreateComponent(
            entry.entity,
            gz::sim::components::LightCmd(message));
      }
      else
      {
        command->Data() = message;
        _ecm.SetChanged(
            entry.entity, gz::sim::components::LightCmd::typeId,
            gz::sim::ComponentState::OneTimeChange);
      }
    }
    return true;
  }

  private: bool IsDescendantOfModel(
      const gz::sim::Entity _entity,
      const gz::sim::EntityComponentManager &_ecm) const
  {
    auto current = _entity;
    std::unordered_set<gz::sim::Entity> visited;
    while (current != gz::sim::kNullEntity && visited.insert(current).second)
    {
      if (current == this->modelEntity)
        return true;
      const auto *parent =
          _ecm.Component<gz::sim::components::ParentEntity>(current);
      if (parent == nullptr)
        return false;
      current = parent->Data();
    }
    return false;
  }

  private: std::size_t ApplyOutputs(
      gz::sim::EntityComponentManager &_ecm,
      const LightingOutputs &_outputs)
  {
    std::unordered_set<std::string> matchedNames;
    _ecm.Each<
        gz::sim::components::Name,
        gz::sim::components::Visual,
        gz::sim::components::Material>(
        [&](const gz::sim::Entity &_entity,
            gz::sim::components::Name *_name,
            gz::sim::components::Visual *,
            gz::sim::components::Material *_material)
        {
          auto spec = this->visualGroups.end();
          for (auto candidate = this->visualGroups.begin();
               candidate != this->visualGroups.end(); ++candidate)
          {
            const auto lumpedToken =
                "__" + candidate->first + "_visual_";
            if (_name->Data() == candidate->first ||
                _name->Data().find(lumpedToken) != std::string::npos)
            {
              spec = candidate;
              break;
            }
          }
          if (spec == this->visualGroups.end() ||
              !this->IsDescendantOfModel(_entity, _ecm))
          {
            return true;
          }

          bool on = false;
          switch (spec->second.group)
          {
            case Group::Work: on = _outputs.workOn; break;
            case Group::Tail: on = _outputs.tailOn; break;
            case Group::Warning: on = _outputs.warningOn; break;
          }
          auto commandedMaterial = _material->Data();
          commandedMaterial.SetEmissive(
              on ? spec->second.emissiveColor : gz::math::Color::Black);
          gz::msgs::Visual visualCommand;
          visualCommand.set_id(_entity);
          visualCommand.set_name(_name->Data());
          const auto *parent =
              _ecm.Component<gz::sim::components::ParentEntity>(_entity);
          if (parent == nullptr || parent->Data() == gz::sim::kNullEntity)
          {
            return true;
          }
          visualCommand.set_parent_id(parent->Data());
          *visualCommand.mutable_material() =
              gz::sim::convert<gz::msgs::Material>(commandedMaterial);
          auto *command =
              _ecm.Component<gz::sim::components::VisualCmd>(_entity);
          if (command == nullptr)
          {
            _ecm.CreateComponent(
                _entity,
                gz::sim::components::VisualCmd(visualCommand));
          }
          else
          {
            command->Data() = visualCommand;
            _ecm.SetChanged(
                _entity, gz::sim::components::VisualCmd::typeId,
                gz::sim::ComponentState::OneTimeChange);
          }
          // Count the canonical URDF visual name. sdformat rewrites fixed-link
          // visuals to names such as
          // base_footprint_fixed_joint_lump__<name>_visual_20.
          matchedNames.insert(spec->first);
          return true;
        });
    return matchedNames.size();
  }

  private: static void Publish(
      gz::transport::Node::Publisher &_publisher,
      const bool _value)
  {
    gz::msgs::Boolean message;
    message.set_data(_value);
    _publisher.Publish(message);
  }

  private: void OnWork(const gz::msgs::Boolean &_message)
  {
    this->workRequested.store(_message.data());
  }
  private: void OnTail(const gz::msgs::Boolean &_message)
  {
    this->tailRequested.store(_message.data());
  }
  private: void OnWarning(const gz::msgs::Boolean &_message)
  {
    this->warningRequested.store(_message.data());
  }
  private: void OnExternalEstop(const gz::msgs::Boolean &_message)
  {
    this->externalEstop.store(_message.data());
  }
  private: void OnPhysicalButton(const gz::msgs::Boolean &_message)
  {
    this->physicalButtonPressed.store(_message.data());
  }
  private: void OnReset(const gz::msgs::Boolean &_message)
  {
    if (_message.data())
      this->resetRequested.store(true);
  }
  private: void OnSafetyPower(const gz::msgs::Boolean &_message)
  {
    this->safetyPowerAvailable.store(_message.data());
  }
  private: void OnMainPowerRequest(const gz::msgs::Boolean &_message)
  {
    this->mainPowerRequested.store(_message.data());
  }
  private: void OnContactorCommand(const gz::msgs::Boolean &_message)
  {
    this->contactorCommanded.store(_message.data());
  }

  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity plungerJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity isolatorJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity contactorJoint{gz::sim::kNullEntity};
  private: std::unique_ptr<LightingCore> lightingCore;
  private: std::unique_ptr<EstopLatchCore> estopCore;
  private: std::unordered_map<std::string, VisualSpec> visualGroups;
  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher latchedPublisher;
  private: gz::transport::Node::Publisher workAppliedPublisher;
  private: gz::transport::Node::Publisher tailAppliedPublisher;
  private: gz::transport::Node::Publisher warningAppliedPublisher;
  private: gz::transport::Node::Publisher isolatorStatePublisher;
  private: gz::transport::Node::Publisher contactorStatePublisher;
  private: std::string workTopic;
  private: std::string tailTopic;
  private: std::string warningTopic;
  private: std::string externalEstopTopic;
  private: std::string physicalButtonTopic;
  private: std::string resetTopic;
  private: std::string safetyPowerTopic;
  private: std::string mainPowerRequestTopic;
  private: std::string contactorCommandTopic;
  private: std::string isolatorStateTopic;
  private: std::string contactorStateTopic;
  private: std::string latchedStateTopic;
  private: std::string workAppliedTopic;
  private: std::string tailAppliedTopic;
  private: std::string warningAppliedTopic;
  private: std::string plungerJointName;
  private: std::string isolatorJointName;
  private: std::string contactorJointName;
  private: std::atomic<bool> workRequested{false};
  private: std::atomic<bool> tailRequested{false};
  private: std::atomic<bool> warningRequested{false};
  private: std::atomic<bool> externalEstop{false};
  private: std::atomic<bool> physicalButtonPressed{false};
  private: std::atomic<bool> resetRequested{false};
  private: std::atomic<bool> safetyPowerAvailable{false};
  private: std::atomic<bool> mainPowerRequested{false};
  private: std::atomic<bool> contactorCommanded{false};
  private: LightingOutputs appliedOutputs;
  private: bool haveAppliedOutputs{false};
  private: bool configured{false};
  private: bool photometricEnabled{true};
  private: bool photometricLightsCreated{false};
  private: bool lightingBindingDiagnosticPublished{false};
  private: double workLightIntensity{1.8};
  private: double workLightRange{12.0};
  private: double workSpotInnerAngle{0.34};
  private: double workSpotOuterAngle{0.62};
  private: double warningLightIntensity{0.55};
  private: double warningLightRange{2.5};
  private: double tailLightIntensity{0.18};
  private: double tailLightRange{1.5};
  private: double plungerTravel{0.006};
  private: double plungerPressedThreshold{0.005};
  private: double plungerReleasedClearance{0.00002};
  private: double plungerPositionGain{1500.0};
  private: double plungerDampingGain{15.0};
  private: double plungerMaximumForce{20.0};
  private: double isolatorTravel{1.5707963267948966};
  private: double isolatorClosedThreshold{1.40};
  private: double isolatorOpenClearance{0.002};
  private: double isolatorPositionGain{18.0};
  private: double isolatorDampingGain{2.2};
  private: double isolatorMaximumTorque{9.0};
  private: double contactorTravel{0.004};
  private: double contactorClosedThreshold{0.0035};
  private: double contactorOpenClearance{0.00002};
  private: double contactorPositionGain{8500.0};
  private: double contactorDampingGain{90.0};
  private: double contactorMaximumForce{35.0};
  private: std::vector<PhotometricLight> photometricLights;
};

}  // namespace sanitation_gazebo_auxiliary

GZ_ADD_PLUGIN(
    sanitation_gazebo_auxiliary::FormalAuxiliaryVisualSystem,
    gz::sim::System,
    sanitation_gazebo_auxiliary::FormalAuxiliaryVisualSystem::ISystemConfigure,
    sanitation_gazebo_auxiliary::FormalAuxiliaryVisualSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_auxiliary::FormalAuxiliaryVisualSystem,
    "sanitation_gazebo_auxiliary::FormalAuxiliaryVisualSystem")
