// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

// Formal effort-domain A300 drivetrain plant. Runtime acceptance remains a
// separate gate from source integration and package/plugin loading.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

#include "sanitation_gazebo_control/A300DrivetrainPlantCore.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr std::array<const char *, kA300WheelCount> kDefaultWheelJointNames{
  "front_left_wheel_joint", "front_right_wheel_joint",
  "rear_left_wheel_joint", "rear_right_wheel_joint"};

const char * StopReasonName(const A300DrivetrainStopReason reason)
{
  switch (reason) {
    case A300DrivetrainStopReason::kNone: return "none";
    case A300DrivetrainStopReason::kActuatorDisabled: return "actuator_disabled";
    case A300DrivetrainStopReason::kEmergencyStop: return "emergency_stop";
    case A300DrivetrainStopReason::kCommandTimeout: return "command_timeout";
    case A300DrivetrainStopReason::kMotorFault: return "motor_fault";
    case A300DrivetrainStopReason::kInvalidInput: return "invalid_input";
  }
  return "unknown";
}
}  // namespace

/// Gazebo-side four-motor plant replacing the ideal velocity-controlled
/// drivetrain. It accepts planar Twist
/// references only from a product-safe typed adapter, calculates wheel
/// torques with A300 public electrical / speed boundaries, and applies effort
/// commands. It never estimates pedestrian velocity or consumes world truth.
class A300DrivetrainPlantSystem final:
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
    const gz::sim::Model model(entity);
    this->modelName = model.Name(ecm);
    if (sdf->HasElement("command_topic")) {
      this->commandTopic = sdf->Get<std::string>("command_topic");
    }
    if (sdf->HasElement("actuator_enable_topic")) {
      this->enableTopic = sdf->Get<std::string>("actuator_enable_topic");
    }
    if (sdf->HasElement("emergency_stop_topic")) {
      this->emergencyStopTopic = sdf->Get<std::string>("emergency_stop_topic");
    }
    if (sdf->HasElement("motor_fault_topic")) {
      this->motorFaultTopic = sdf->Get<std::string>("motor_fault_topic");
    }
    if (sdf->HasElement("bus_voltage_topic")) {
      this->busVoltageTopic = sdf->Get<std::string>("bus_voltage_topic");
    }
    if (sdf->HasElement("status_topic")) {
      this->statusTopic = sdf->Get<std::string>("status_topic");
    }
    if (sdf->HasElement("odometry_topic")) {
      this->odometryTopic = sdf->Get<std::string>("odometry_topic");
    }
    if (sdf->HasElement("odometry_frame_id")) {
      this->odometryFrameId = sdf->Get<std::string>("odometry_frame_id");
    }
    if (sdf->HasElement("odometry_child_frame_id")) {
      this->odometryChildFrameId =
        sdf->Get<std::string>("odometry_child_frame_id");
    }

    for (std::size_t index = 0; index < kA300WheelCount; ++index) {
      const std::string key = "wheel_joint_" + std::to_string(index);
      const std::string jointName = sdf->HasElement(key) ?
        sdf->Get<std::string>(key) : kDefaultWheelJointNames[index];
      this->wheelJoints[index] = model.JointByName(ecm, jointName);
      if (this->wheelJoints[index] == gz::sim::kNullEntity) {
        this->configured = false;
        return;
      }
      gz::sim::enableComponent<gz::sim::components::JointVelocity>(
        ecm, this->wheelJoints[index], true);
    }

    this->node.Subscribe(
      this->commandTopic, &A300DrivetrainPlantSystem::OnTwistCommand, this);
    this->node.Subscribe(
      this->enableTopic, &A300DrivetrainPlantSystem::OnActuatorEnable, this);
    this->node.Subscribe(
      this->emergencyStopTopic, &A300DrivetrainPlantSystem::OnEmergencyStop, this);
    this->node.Subscribe(
      this->motorFaultTopic, &A300DrivetrainPlantSystem::OnMotorFault, this);
    this->node.Subscribe(
      this->busVoltageTopic, &A300DrivetrainPlantSystem::OnBusVoltage, this);
    this->statusPublisher = this->node.Advertise<gz::msgs::StringMsg>(this->statusTopic);
    this->odometryPublisher = this->node.Advertise<gz::msgs::Odometry>(this->odometryTopic);
    this->configured = true;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured || info.paused) {
      return;
    }
    const double stepS = std::chrono::duration<double>(info.dt).count();
    const double simTimeS = std::chrono::duration<double>(info.simTime).count();

    A300DrivetrainPlantInput input;
    input.step_s = stepS;
    input.measured_speed_rad_s.fill(std::numeric_limits<double>::quiet_NaN());
    for (std::size_t index = 0; index < kA300WheelCount; ++index) {
      const auto * velocity =
        ecm.Component<gz::sim::components::JointVelocity>(this->wheelJoints[index]);
      if (velocity != nullptr && !velocity->Data().empty()) {
        input.measured_speed_rad_s[index] = velocity->Data().front();
      }
    }

    {
      std::lock_guard<std::mutex> lock(this->inputMutex);
      input.commanded_speed_rad_s = this->wheelCommand;
      input.motor_fault = this->motorFault;
      input.actuator_enable = this->actuatorEnable;
      input.emergency_stop = this->emergencyStop;
      input.bus_voltage_v = this->busVoltageV;
      input.command_age_s = this->commandSeen ?
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - this->lastCommandSteadyTime).count() :
        this->plant.Parameters().command_timeout_s + 1.0;
    }

    const auto output = this->plant.Step(input);
    for (std::size_t index = 0; index < kA300WheelCount; ++index) {
      // Gazebo's revolute-joint effort component is expressed in the
      // parent-on-child convention, while JointVelocity and the product A300
      // core use the child-relative-to-parent generalized-coordinate
      // convention. The two signs are opposite for this four-wheel model.
      // Keeping that conversion explicit at the simulator boundary is
      // essential: writing the core torque directly made a positive speed
      // error accelerate every measured wheel in the negative direction and
      // injected energy even while the zero-speed brake was active.
      const double gazeboJointForceNm = -output.wheel_torque_nm[index];
      auto * force =
        ecm.Component<gz::sim::components::JointForceCmd>(this->wheelJoints[index]);
      if (force == nullptr) {
        ecm.CreateComponent(
          this->wheelJoints[index],
          gz::sim::components::JointForceCmd({gazeboJointForceNm}));
      } else {
        force->Data() = {gazeboJointForceNm};
      }
    }

    this->UpdateAndPublishOdometry(input.measured_speed_rad_s, stepS, simTimeS);

    if (simTimeS - this->lastStatusTimeS >= 0.1) {
      this->PublishStatus(output);
      this->lastStatusTimeS = simTimeS;
    }
  }

private:
  void OnTwistCommand(const gz::msgs::Twist & message)
  {
    const double linearMps = message.linear().x();
    const double angularRadS = message.angular().z();
    const double radius = this->plant.Parameters().control_wheel_radius_m;
    const double halfTrack = this->wheelTrackM * 0.5;
    const double leftRadS = (linearMps - angularRadS * halfTrack) / radius;
    const double rightRadS = (linearMps + angularRadS * halfTrack) / radius;
    std::lock_guard<std::mutex> lock(this->inputMutex);
    this->wheelCommand = {leftRadS, rightRadS, leftRadS, rightRadS};
    this->lastCommandSteadyTime = std::chrono::steady_clock::now();
    this->commandSeen = true;
  }

  void OnActuatorEnable(const gz::msgs::Boolean & message)
  {
    std::lock_guard<std::mutex> lock(this->inputMutex);
    this->actuatorEnable = message.data();
  }

  void OnEmergencyStop(const gz::msgs::Boolean & message)
  {
    std::lock_guard<std::mutex> lock(this->inputMutex);
    this->emergencyStop = message.data();
  }

  void OnMotorFault(const gz::msgs::Double_V & message)
  {
    std::lock_guard<std::mutex> lock(this->inputMutex);
    if (message.data_size() != static_cast<int>(kA300WheelCount)) {
      this->motorFault.fill(true);
      return;
    }
    for (std::size_t index = 0; index < kA300WheelCount; ++index) {
      const double value = message.data(static_cast<int>(index));
      this->motorFault[index] = !std::isfinite(value) || value != 0.0;
    }
  }

  void OnBusVoltage(const gz::msgs::Double & message)
  {
    std::lock_guard<std::mutex> lock(this->inputMutex);
    this->busVoltageV = message.data();
  }

  void PublishStatus(const A300DrivetrainPlantOutput & output)
  {
    std::ostringstream stream;
    stream << "{\"model\":\"" << this->modelName << "\","
      << "\"drive_permitted\":" << (output.drive_permitted ? "true" : "false") << ','
      << "\"stop_reason\":\"" << StopReasonName(output.stop_reason) << "\","
      << "\"resistive_brake_active\":"
      << (output.resistive_brake_active ? "true" : "false") << ','
      << "\"current_limited\":" << (output.current_limited ? "true" : "false") << ','
      << "\"power_limited\":" << (output.power_limited ? "true" : "false") << ','
      << "\"mechanical_power_w\":" << output.total_mechanical_power_w << ','
      << "\"estimated_battery_current_a\":" << output.estimated_battery_current_a
      << '}';
    gz::msgs::StringMsg status;
    status.set_data(stream.str());
    this->statusPublisher.Publish(status);
  }

  void UpdateAndPublishOdometry(
    const std::array<double, kA300WheelCount> & wheelSpeedRadS,
    const double stepS,
    const double simTimeS)
  {
    if (!std::all_of(
        wheelSpeedRadS.begin(), wheelSpeedRadS.end(),
        [](const double value) {return std::isfinite(value);}) ||
      !std::isfinite(stepS) || stepS <= 0.0)
    {
      return;
    }
    const double leftRadS = (wheelSpeedRadS[0] + wheelSpeedRadS[2]) * 0.5;
    const double rightRadS = (wheelSpeedRadS[1] + wheelSpeedRadS[3]) * 0.5;
    const double radius = this->plant.Parameters().control_wheel_radius_m;
    const double linearMps = radius * (leftRadS + rightRadS) * 0.5;
    const double angularRadS = radius * (rightRadS - leftRadS) / this->wheelTrackM;
    this->odomYawRad += angularRadS * stepS;
    this->odomX += linearMps * std::cos(this->odomYawRad) * stepS;
    this->odomY += linearMps * std::sin(this->odomYawRad) * stepS;

    if (simTimeS - this->lastOdometryTimeS < 0.02) {
      return;
    }
    gz::msgs::Odometry odometry;
    const double nonnegativeTimeS = std::max(0.0, simTimeS);
    const auto seconds = static_cast<std::int64_t>(std::floor(nonnegativeTimeS));
    const auto nanoseconds = static_cast<std::int32_t>(
      (nonnegativeTimeS - static_cast<double>(seconds)) * 1.0e9);
    odometry.mutable_header()->mutable_stamp()->set_sec(seconds);
    odometry.mutable_header()->mutable_stamp()->set_nsec(nanoseconds);
    auto * frame = odometry.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value(this->odometryFrameId);
    auto * childFrame = odometry.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->odometryChildFrameId);
    odometry.mutable_pose()->mutable_position()->set_x(this->odomX);
    odometry.mutable_pose()->mutable_position()->set_y(this->odomY);
    odometry.mutable_pose()->mutable_position()->set_z(0.0);
    odometry.mutable_pose()->mutable_orientation()->set_z(std::sin(this->odomYawRad * 0.5));
    odometry.mutable_pose()->mutable_orientation()->set_w(std::cos(this->odomYawRad * 0.5));
    odometry.mutable_twist()->mutable_linear()->set_x(linearMps);
    odometry.mutable_twist()->mutable_angular()->set_z(angularRadS);
    this->odometryPublisher.Publish(odometry);
    this->lastOdometryTimeS = simTimeS;
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher statusPublisher;
  gz::transport::Node::Publisher odometryPublisher;
  A300DrivetrainPlantCore plant;
  std::array<gz::sim::Entity, kA300WheelCount> wheelJoints{};
  std::mutex inputMutex;
  std::array<double, kA300WheelCount> wheelCommand{};
  std::array<bool, kA300WheelCount> motorFault{};
  std::chrono::steady_clock::time_point lastCommandSteadyTime{};
  bool commandSeen{false};
  bool actuatorEnable{false};
  bool emergencyStop{false};
  bool configured{false};
  double busVoltageV{25.6};
  double lastStatusTimeS{-1.0};
  double lastOdometryTimeS{-1.0};
  double odomX{0.0};
  double odomY{0.0};
  double odomYawRad{0.0};
  const double wheelTrackM{0.562};
  std::string modelName;
  std::string commandTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/cmd_vel"};
  std::string enableTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/actuator_enable"};
  std::string emergencyStopTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/emergency_stop"};
  std::string motorFaultTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/motor_fault"};
  std::string busVoltageTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/bus_voltage_v"};
  std::string statusTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status"};
  std::string odometryTopic{
    "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom"};
  std::string odometryFrameId{"odom"};
  std::string odometryChildFrameId{"base_footprint"};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
  sanitation_gazebo_control::A300DrivetrainPlantSystem,
  gz::sim::System,
  sanitation_gazebo_control::A300DrivetrainPlantSystem::ISystemConfigure,
  sanitation_gazebo_control::A300DrivetrainPlantSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  sanitation_gazebo_control::A300DrivetrainPlantSystem,
  "sanitation_gazebo_control::A300DrivetrainPlantSystem")
