// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <ostream>
#include <string>
#include <tuple>
#include <vector>

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

namespace sanitation_gazebo_auxiliary
{

/// Evaluation drive for the four manually serviced body doors. Commands are
/// applied as bounded spring/damper forces to the real URDF joints. A hinge
/// target is accepted only after its measured latch angle is unlocked, and a
/// latch is held unlocked until the measured hinge has returned to closed.
class ServiceDoorSystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
  private: struct Door
  {
    std::string id;
    std::string hingeName;
    std::string latchName;
    double lower{0.0};
    double upper{0.0};
    gz::sim::Entity hinge{gz::sim::kNullEntity};
    gz::sim::Entity latch{gz::sim::kNullEntity};
    std::atomic<double> requestedHinge{0.0};
    std::atomic<double> requestedLatch{0.0};
    std::atomic<std::uint64_t> receivedHingeMessages{0};
    std::atomic<std::uint64_t> receivedLatchMessages{0};
    std::uint64_t hingeForceWrites{0};
    std::uint64_t latchForceWrites{0};
    bool hingeForceCommandPresent{false};
    bool latchForceCommandPresent{false};
    // A missing command is represented by its companion `*_present=0`, not a
    // NaN.  Formal telemetry accepts finite values only, including the first
    // PreUpdate before any PostUpdate observation exists.
    double postUpdateHingeForce{0.0};
    double postUpdateLatchForce{0.0};
  };

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    const gz::sim::Model model(_entity);
    if (!model.Valid(_ecm))
    {
      // Gazebo's gzmsg/gzerr console stream is not guaranteed to be captured
      // by a ros2 launch redirection.  The formal evaluator retains its
      // launch stderr, so lifecycle evidence must use this direct stream.
      std::cerr << "SERVICE_DOOR_LIFECYCLE event=configure_failed"
                << " reason=model_not_valid" << std::endl;
      return;
    }
    const std::array<std::tuple<const char *, double, double>, 4> specs{{
      {"power", 0.0, 1.745329252},
      {"compute", -1.745329252, 0.0},
      {"wet", -1.745329252, 0.0},
      {"rear_dry", -1.745329252, 0.0},
    }};
    for (std::size_t index = 0; index < specs.size(); ++index)
    {
      auto door = std::make_unique<Door>();
      door->id = std::get<0>(specs[index]);
      door->lower = std::get<1>(specs[index]);
      door->upper = std::get<2>(specs[index]);
      door->hingeName = "bodywork_" + door->id + "_service_door_hinge_joint";
      door->latchName = "bodywork_" + door->id + "_service_door_latch_joint";
      door->hinge = model.JointByName(_ecm, door->hingeName);
      door->latch = model.JointByName(_ecm, door->latchName);
      if (door->hinge == gz::sim::kNullEntity ||
          door->latch == gz::sim::kNullEntity)
      {
        std::cerr << "SERVICE_DOOR_LIFECYCLE event=configure_failed"
                  << " reason=joint_not_found door=" << door->id << std::endl;
        return;
      }
      gz::sim::enableComponent<gz::sim::components::JointPosition>(
          _ecm, door->hinge, true);
      gz::sim::enableComponent<gz::sim::components::JointVelocity>(
          _ecm, door->hinge, true);
      gz::sim::enableComponent<gz::sim::components::JointPosition>(
          _ecm, door->latch, true);
      gz::sim::enableComponent<gz::sim::components::JointVelocity>(
          _ecm, door->latch, true);
      const std::string prefix =
          "/formal_vehicle/evaluation/bodywork_service/" + door->id;
      Door *doorPtr = door.get();
      const std::function<void(const gz::msgs::Double &)> hingeCallback =
          [doorPtr](const gz::msgs::Double &_message)
          {
            if (std::isfinite(_message.data()))
            {
              doorPtr->requestedHinge.store(_message.data());
              ++doorPtr->receivedHingeMessages;
            }
          };
      const std::function<void(const gz::msgs::Double &)> latchCallback =
          [doorPtr](const gz::msgs::Double &_message)
          {
            if (std::isfinite(_message.data()))
            {
              doorPtr->requestedLatch.store(_message.data());
              ++doorPtr->receivedLatchMessages;
            }
          };
      const bool hingeOk = this->node.Subscribe<gz::msgs::Double>(
          prefix + "/hinge_target_rad", hingeCallback);
      const bool latchOk = this->node.Subscribe<gz::msgs::Double>(
          prefix + "/latch_target_rad", latchCallback);
      if (!hingeOk || !latchOk)
      {
        std::cerr << "SERVICE_DOOR_LIFECYCLE event=configure_failed"
                  << " reason=subscription_failed door=" << door->id
                  << " hinge_subscribed=" << hingeOk
                  << " latch_subscribed=" << latchOk << std::endl;
        return;
      }
      std::cerr << "SERVICE_DOOR_LIFECYCLE event=subscription"
                << " door=" << door->id
                << " hinge_subscribed=" << hingeOk
                << " latch_subscribed=" << latchOk << std::endl;
      this->doors[index] = std::move(door);
    }
    this->configured = true;
    std::cerr << "SERVICE_DOOR_LIFECYCLE event=configured"
              << " configured=1 doors=" << this->doors.size()
              << " model_entity=" << _entity << std::endl;
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused)
      return;
    for (const auto &door : this->doors)
    {
      const double hingePosition = Position(_ecm, door->hinge);
      const double latchPosition = Position(_ecm, door->latch);
      if (!std::isfinite(hingePosition) || !std::isfinite(latchPosition))
        continue;

      const double receivedLatch = door->requestedLatch.load();
      const double requestedLatch = std::clamp(
          receivedLatch, -this->latchLimit, this->latchLimit);
      double effectiveLatch = requestedLatch;
      // The rotary tongue cannot relock across an open panel. Keep it clear
      // until the measured hinge, not merely its command, is closed.
      if (std::abs(requestedLatch) < this->unlockThreshold &&
          std::abs(hingePosition) > this->closedTolerance)
      {
        effectiveLatch = std::copysign(
            this->serviceLatchAngle,
            std::abs(latchPosition) > this->unlockThreshold ? latchPosition : 1.0);
      }
      const bool measuredUnlocked =
          std::abs(latchPosition) >= this->unlockThreshold;
      const double receivedHinge = door->requestedHinge.load();
      const double requestedHinge = std::clamp(
          receivedHinge, door->lower, door->upper);
      const double effectiveHinge = measuredUnlocked ? requestedHinge : 0.0;
      const double latchForce = ApplyPd(
          _ecm, door->latch, effectiveLatch, latchPosition,
          this->latchGain, this->latchDamping, this->latchMaximumForce);
      ++door->latchForceWrites;
      const double hingeForce = ApplyPd(
          _ecm, door->hinge, effectiveHinge, hingePosition,
          this->hingeGain, this->hingeDamping, this->hingeMaximumForce);
      ++door->hingeForceWrites;
      if (_info.simTime >= this->nextDiagnosticTime)
      {
        std::cerr << "SERVICE_DOOR_DIAGNOSTIC door=" << door->id
              << " sim_time_sec="
              << std::chrono::duration<double>(_info.simTime).count()
              << " received_hinge_messages="
              << door->receivedHingeMessages.load()
              << " received_latch_messages="
              << door->receivedLatchMessages.load()
              << " received_hinge_target_rad=" << receivedHinge
              << " received_latch_target_rad=" << receivedLatch
              << " requested_hinge_rad=" << requestedHinge
              << " requested_latch_rad=" << requestedLatch
              << " effective_hinge_rad=" << effectiveHinge
              << " effective_latch_rad=" << effectiveLatch
              << " hinge_position_rad=" << hingePosition
              << " latch_position_rad=" << latchPosition
              << " hinge_force_nm=" << hingeForce
              << " latch_force_nm=" << latchForce
              << " hinge_force_writes=" << door->hingeForceWrites
              << " latch_force_writes=" << door->latchForceWrites
              << " postupdate_hinge_force_present=" << door->hingeForceCommandPresent
              << " postupdate_latch_force_present=" << door->latchForceCommandPresent
              << " postupdate_hinge_force_nm=" << door->postUpdateHingeForce
              << " postupdate_latch_force_nm=" << door->postUpdateLatchForce
              << std::endl;
      }
    }
    if (_info.simTime >= this->nextDiagnosticTime)
      this->nextDiagnosticTime = _info.simTime + std::chrono::seconds(1);
  }

  public: void PostUpdate(
      const gz::sim::UpdateInfo &,
      const gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->configured)
      return;
    for (const auto &door : this->doors)
    {
      const auto *hinge =
          _ecm.Component<gz::sim::components::JointForceCmd>(door->hinge);
      const auto *latch =
          _ecm.Component<gz::sim::components::JointForceCmd>(door->latch);
      door->hingeForceCommandPresent = hinge != nullptr && !hinge->Data().empty();
      door->latchForceCommandPresent = latch != nullptr && !latch->Data().empty();
      door->postUpdateHingeForce = door->hingeForceCommandPresent ?
          hinge->Data().front() : 0.0;
      door->postUpdateLatchForce = door->latchForceCommandPresent ?
          latch->Data().front() : 0.0;
    }
  }

  private: static double Position(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint)
  {
    const auto *position =
        _ecm.Component<gz::sim::components::JointPosition>(_joint);
    return position != nullptr && !position->Data().empty() ?
        position->Data().front() : std::numeric_limits<double>::quiet_NaN();
  }

  private: static double Velocity(
      const gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint)
  {
    const auto *velocity =
        _ecm.Component<gz::sim::components::JointVelocity>(_joint);
    return velocity != nullptr && !velocity->Data().empty() &&
        std::isfinite(velocity->Data().front()) ? velocity->Data().front() : 0.0;
  }

  private: static double ApplyPd(
      gz::sim::EntityComponentManager &_ecm,
      const gz::sim::Entity _joint,
      const double _target,
      const double _position,
      const double _gain,
      const double _damping,
      const double _maximumForce)
  {
    const double force = std::clamp(
        _gain * (_target - _position) - _damping * Velocity(_ecm, _joint),
        -_maximumForce, _maximumForce);
    const std::vector<double> command{force};
    auto *component =
        _ecm.Component<gz::sim::components::JointForceCmd>(_joint);
    if (component == nullptr)
      _ecm.CreateComponent(_joint, gz::sim::components::JointForceCmd(command));
    else
    {
      component->Data() = command;
      _ecm.SetChanged(
          _joint,
          gz::sim::components::JointForceCmd::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }
    return force;
  }

  private: gz::transport::Node node;
  private: std::array<std::unique_ptr<Door>, 4> doors;
  private: bool configured{false};
  private: std::chrono::steady_clock::duration nextDiagnosticTime{
      std::chrono::steady_clock::duration::zero()};
  private: double unlockThreshold{0.35};
  private: double serviceLatchAngle{0.60};
  private: double closedTolerance{0.08};
  private: double latchLimit{0.785398163};
  private: double hingeGain{42.0};
  private: double hingeDamping{7.0};
  private: double hingeMaximumForce{30.0};
  private: double latchGain{16.0};
  private: double latchDamping{2.0};
  private: double latchMaximumForce{8.0};
};

}  // namespace sanitation_gazebo_auxiliary

GZ_ADD_PLUGIN(
    sanitation_gazebo_auxiliary::ServiceDoorSystem,
    gz::sim::System,
    sanitation_gazebo_auxiliary::ServiceDoorSystem::ISystemConfigure,
    sanitation_gazebo_auxiliary::ServiceDoorSystem::ISystemPreUpdate,
    sanitation_gazebo_auxiliary::ServiceDoorSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_auxiliary::ServiceDoorSystem,
    "sanitation_gazebo_auxiliary::ServiceDoorSystem")
