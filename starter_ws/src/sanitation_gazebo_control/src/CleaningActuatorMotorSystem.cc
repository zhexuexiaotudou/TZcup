// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

#include "sanitation_gazebo_control/CleaningActuatorMotorCore.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr std::array<const char *, kCleaningActuatorCount> kJointNames{
  "left_side_brush_joint", "right_side_brush_joint", "central_roller_joint",
  "cleaning_lift_joint", "recovery_pump_joint"};

constexpr std::array<const char *, 8> kPublishTopicTags{
  "fault_active", "motor_current_a", "motor_temperature_c",
  "estimated_output_load", "total_current_a", "total_power_w",
  "telemetry_snapshot", "status_json"};

double FirstValue(
  const gz::sim::EntityComponentManager & ecm,
  const gz::sim::Entity entity, const bool velocity)
{
  if (entity == gz::sim::kNullEntity) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (velocity) {
    const auto * component =
      ecm.Component<gz::sim::components::JointVelocity>(entity);
    return component != nullptr && !component->Data().empty() ?
      component->Data().front() : std::numeric_limits<double>::quiet_NaN();
  }
  const auto * component =
    ecm.Component<gz::sim::components::JointPosition>(entity);
  return component != nullptr && !component->Data().empty() ?
    component->Data().front() : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

/// Read-only electromechanical / thermal observer for the cleaning actuators.
/// gz_ros2_control remains the only joint command authority. This system reads
/// joint state and post-safety controller references, estimates data-sheet
/// bounded torque/thrust, current and temperature, then publishes a protection
/// fault consumed by the whole-vehicle safety manager. It never writes a
/// JointForceCmd, JointVelocityCmd or JointPositionCmd component.
class CleaningActuatorMotorSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ~CleaningActuatorMotorSystem() override
  {
    this->telemetryRunning.store(false);
    if (this->telemetryThread.joinable()) {
      this->telemetryThread.join();
    }
  }

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    const gz::sim::Model model(entity);
    this->modelName = model.Name(ecm);
    if (sdf != nullptr && sdf->HasElement("status_json_publish_rate_hz")) {
      this->statusJsonPublishRateHz = sdf->Get<double>("status_json_publish_rate_hz");
    }
    if (sdf != nullptr && sdf->HasElement("realtime_telemetry_enabled")) {
      this->realtimeTelemetryEnabled = sdf->Get<bool>("realtime_telemetry_enabled");
    }
    if (sdf != nullptr && sdf->HasElement("status_json_enabled")) {
      this->statusJsonEnabled = sdf->Get<bool>("status_json_enabled");
    }
    if (!std::isfinite(this->statusJsonPublishRateHz) ||
      this->statusJsonPublishRateHz <= 0.0 || this->statusJsonPublishRateHz > 20.0)
    {
      std::cerr << "[CleaningActuatorMotorSystem] invalid "
                << "status_json_publish_rate_hz=" << this->statusJsonPublishRateHz
                << "; expected (0, 20]" << std::endl;
      this->configured = false;
      return;
    }
    for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
      this->joints[index] = model.JointByName(ecm, kJointNames[index]);
      if (this->joints[index] == gz::sim::kNullEntity) {
        this->configured = false;
        return;
      }
      gz::sim::enableComponent<gz::sim::components::JointVelocity>(
        ecm, this->joints[index], true);
      gz::sim::enableComponent<gz::sim::components::JointPosition>(
        ecm, this->joints[index], true);
    }

    this->node.Subscribe(this->brushCommandTopic,
      &CleaningActuatorMotorSystem::OnBrushCommand, this);
    this->node.Subscribe(this->pumpCommandTopic,
      &CleaningActuatorMotorSystem::OnPumpCommand, this);
    this->node.Subscribe(this->liftCommandTopic,
      &CleaningActuatorMotorSystem::OnLiftCommand, this);
    this->node.Subscribe(this->enableTopic,
      &CleaningActuatorMotorSystem::OnEnable, this);
    this->node.Subscribe(this->resetTopic,
      &CleaningActuatorMotorSystem::OnReset, this);
    this->statusPublisher = this->node.Advertise<gz::msgs::StringMsg>(
      this->stateRoot + "/status_json");
    this->faultPublisher = this->node.Advertise<gz::msgs::Boolean>(
      this->stateRoot + "/fault_active");
    this->currentPublisher = this->node.Advertise<gz::msgs::Double_V>(
      this->stateRoot + "/motor_current_a");
    this->temperaturePublisher = this->node.Advertise<gz::msgs::Double_V>(
      this->stateRoot + "/motor_temperature_c");
    this->loadPublisher = this->node.Advertise<gz::msgs::Double_V>(
      this->stateRoot + "/estimated_output_load");
    this->telemetrySnapshotPublisher = this->node.Advertise<gz::msgs::Double_V>(
      this->stateRoot + "/telemetry_snapshot");
    this->totalCurrentPublisher = this->node.Advertise<gz::msgs::Double>(
      this->stateRoot + "/total_current_a");
    this->totalPowerPublisher = this->node.Advertise<gz::msgs::Double>(
      this->stateRoot + "/total_power_w");
    this->configured = true;
    this->telemetryRunning.store(true);
    this->telemetryThread = std::thread([this]() {this->TelemetryLoop();});
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured || info.paused || info.dt.count() <= 0) {
      return;
    }
    const double simTimeS = std::chrono::duration<double>(info.simTime).count();
    CleaningActuatorMotorInput input;
    input.step_s = std::chrono::duration<double>(info.dt).count();
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      if (this->commandRevision != this->appliedCommandRevision) {
        this->lastCommandSimTimeS = simTimeS;
        this->appliedCommandRevision = this->commandRevision;
      }
      input.command = this->command;
      input.actuator_enable = this->enabled;
      input.bus_voltage_v = this->busVoltageV;
      input.command_age_s = this->lastCommandSimTimeS < 0.0 ?
        this->core.Parameters().command_timeout_s + 1.0 :
        simTimeS - this->lastCommandSimTimeS;
      if (this->resetRequested) {
        this->core.ResetFaults();
        this->resetRequested = false;
      }
    }
    bool jointFeedbackFinite = true;
    for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
      input.measured_position[index] = FirstValue(ecm, this->joints[index], false);
      input.measured_speed[index] = FirstValue(ecm, this->joints[index], true);
      jointFeedbackFinite = jointFeedbackFinite &&
        std::isfinite(input.measured_position[index]) &&
        std::isfinite(input.measured_speed[index]);
    }
    if (!jointFeedbackFinite && !this->validPhysicsSampleSeen) {
      // Joint state components exist before gz_ros2_control provides their
      // first sample.  Keep the finite, fail-closed startup heartbeat until
      // real feedback is available instead of publishing NaN measurements.
      return;
    }
    if (jointFeedbackFinite) {
      this->validPhysicsSampleSeen = true;
    }
    const auto output = this->core.Step(input);
    const double physicsUpdateWallTimeS = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    {
      std::lock_guard<std::mutex> lock(this->telemetryMutex);
      this->telemetryGate.Update(output, physicsUpdateWallTimeS);
    }
  }

private:
  void OnBrushCommand(const gz::msgs::Double_V & message)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (message.data_size() != 3) {
      this->command[0] = this->command[1] = this->command[2] =
        std::numeric_limits<double>::quiet_NaN();
    } else {
      for (int index = 0; index < 3; ++index) {
        this->command[static_cast<std::size_t>(index)] = message.data(index);
      }
    }
    ++this->commandRevision;
  }

  void OnPumpCommand(const gz::msgs::Double_V & message)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->command[4] = message.data_size() == 1 ? message.data(0) :
      std::numeric_limits<double>::quiet_NaN();
    ++this->commandRevision;
  }

  void OnLiftCommand(const gz::msgs::Double & message)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->command[3] = message.data();
    ++this->commandRevision;
  }

  void OnEnable(const gz::msgs::Boolean & message)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->enabled = message.data();
  }

  void OnReset(const gz::msgs::Boolean & message)
  {
    if (!message.data()) {
      return;
    }
    std::lock_guard<std::mutex> lock(this->mutex);
    if (!this->enabled) {
      this->resetRequested = true;
    }
  }

  void TelemetryLoop()
  {
    auto nextPublish = std::chrono::steady_clock::now();
    auto nextStatusPublish = nextPublish;
    const auto statusPeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->statusJsonPublishRateHz));
    while (this->telemetryRunning.load()) {
      // This timestamp is only for publisher scheduling.  Do not pass it to
      // the telemetry gate: PreUpdate may publish a newer physics timestamp
      // while this thread is waiting for telemetryMutex.
      const auto wallNow = std::chrono::steady_clock::now();
      CleaningActuatorTelemetrySnapshot snapshot;
      {
        std::lock_guard<std::mutex> lock(this->telemetryMutex);
        const double snapshotWallNowS = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        snapshot = this->telemetryGate.Snapshot(snapshotWallNowS);
      }
      if (this->realtimeTelemetryEnabled) {
        this->PublishRealtime(snapshot);
      }
      if (this->statusJsonEnabled && wallNow >= nextStatusPublish) {
        this->PublishStatusJson(snapshot);
        do {
          nextStatusPublish += statusPeriod;
        } while (nextStatusPublish <= wallNow);
      }
      nextPublish += std::chrono::milliseconds(50);
      if (nextPublish <= wallNow) {
        nextPublish = wallNow + std::chrono::milliseconds(50);
      }
      std::this_thread::sleep_until(nextPublish);
    }
  }

  template<typename MessageT>
  void PublishTagged(
    gz::transport::Node::Publisher & publisher,
    const MessageT & message, const std::size_t topicIndex)
  {
    if (!publisher.Publish(message)) {
      const auto count = ++this->publishFailureCounts[topicIndex];
      std::cerr << "[CleaningActuatorMotorSystem] gz_publish_failed topic="
                << this->stateRoot << "/" << kPublishTopicTags[topicIndex]
                << " count=" << count << std::endl;
    }
  }

  void PublishRealtime(const CleaningActuatorTelemetrySnapshot & snapshot)
  {
    const auto & output = snapshot.output;
    gz::msgs::Boolean fault;
    fault.set_data(output.fault_active);
    this->PublishTagged(this->faultPublisher, fault, 0);
    gz::msgs::Double_V currents;
    gz::msgs::Double_V temperatures;
    gz::msgs::Double_V loads;
    for (const auto & motor : output.motors) {
      currents.add_data(motor.current_a);
      temperatures.add_data(motor.temperature_c);
      loads.add_data(motor.estimated_output_load);
    }
    this->PublishTagged(this->currentPublisher, currents, 1);
    this->PublishTagged(this->temperaturePublisher, temperatures, 2);
    this->PublishTagged(this->loadPublisher, loads, 3);
    gz::msgs::Double totalCurrent;
    totalCurrent.set_data(output.total_current_a);
    this->PublishTagged(this->totalCurrentPublisher, totalCurrent, 4);
    gz::msgs::Double totalPower;
    totalPower.set_data(output.total_power_w);
    this->PublishTagged(this->totalPowerPublisher, totalPower, 5);

    gz::msgs::Double_V telemetrySnapshot;
    for (const double value : CleaningActuatorTelemetryVector(snapshot)) {
      telemetrySnapshot.add_data(value);
    }
    this->PublishTagged(this->telemetrySnapshotPublisher, telemetrySnapshot, 6);
  }

  void PublishStatusJson(const CleaningActuatorTelemetrySnapshot & snapshot)
  {
    // Keep the detailed Gazebo-only JSON forensic stream last.  A transport
    // defect in high-rate StringMsg forwarding must never block the typed,
    // fixed-layout product telemetry path above.
    gz::msgs::StringMsg status;
    status.set_data(CleaningActuatorMotorStatusJson(
      this->modelName, snapshot.output, snapshot.physics_update_stale));
    this->PublishTagged(this->statusPublisher, status, 7);
  }

  gz::transport::Node node;
  gz::transport::Node::Publisher statusPublisher;
  gz::transport::Node::Publisher faultPublisher;
  gz::transport::Node::Publisher currentPublisher;
  gz::transport::Node::Publisher temperaturePublisher;
  gz::transport::Node::Publisher loadPublisher;
  gz::transport::Node::Publisher telemetrySnapshotPublisher;
  gz::transport::Node::Publisher totalCurrentPublisher;
  gz::transport::Node::Publisher totalPowerPublisher;
  CleaningActuatorMotorCore core;
  CleaningActuatorTelemetryGate telemetryGate;
  std::array<gz::sim::Entity, kCleaningActuatorCount> joints{};
  std::array<double, kCleaningActuatorCount> command{};
  std::mutex mutex;
  std::mutex telemetryMutex;
  std::atomic<bool> telemetryRunning{false};
  std::array<std::atomic<std::uint64_t>, 8> publishFailureCounts{};
  std::thread telemetryThread;
  std::uint64_t commandRevision{0};
  std::uint64_t appliedCommandRevision{0};
  bool enabled{false};
  bool resetRequested{false};
  bool configured{false};
  bool validPhysicsSampleSeen{false};
  bool realtimeTelemetryEnabled{true};
  bool statusJsonEnabled{true};
  double busVoltageV{24.0};
  double statusJsonPublishRateHz{20.0};
  double lastCommandSimTimeS{-1.0};
  std::string modelName;
  std::string stateRoot{
    "/model/tzcup_formal_sanitation_vehicle/cleaning_motors"};
  const std::string brushCommandTopic{stateRoot + "/command/brush"};
  const std::string pumpCommandTopic{stateRoot + "/command/pump"};
  const std::string liftCommandTopic{stateRoot + "/command/lift_position"};
  const std::string enableTopic{stateRoot + "/command/enable"};
  const std::string resetTopic{stateRoot + "/command/reset_faults"};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
  sanitation_gazebo_control::CleaningActuatorMotorSystem,
  gz::sim::System,
  sanitation_gazebo_control::CleaningActuatorMotorSystem::ISystemConfigure,
  sanitation_gazebo_control::CleaningActuatorMotorSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  sanitation_gazebo_control::CleaningActuatorMotorSystem,
  "sanitation_gazebo_control::CleaningActuatorMotorSystem")
