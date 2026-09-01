#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace sanitation_gazebo_control
{

constexpr std::size_t kCleaningActuatorCount = 5;
constexpr std::size_t kCleaningTelemetryHeaderCount = 8;
constexpr std::size_t kCleaningTelemetryFieldsPerMotor = 11;
constexpr std::size_t kCleaningTelemetryValueCount =
  kCleaningTelemetryHeaderCount +
  kCleaningActuatorCount * kCleaningTelemetryFieldsPerMotor;
constexpr double kCleaningTelemetrySchemaVersion = 1.0;

enum class CleaningActuatorIndex : std::size_t
{
  kLeftSideBrush = 0,
  kRightSideBrush = 1,
  kCentralRoller = 2,
  kLift = 3,
  kRecoveryPump = 4,
};

enum class CleaningMotorFault : std::uint8_t
{
  kNone = 0,
  kCommandTimeout,
  kStall,
  kOvertemperature,
  kInvalidInput,
};

struct CleaningMotorSpec
{
  double nominal_voltage_v{24.0};
  double no_load_speed{14.6607657};
  double no_load_current_a{0.10};
  double rated_current_a{0.75};
  double stall_current_a{3.0};
  double stall_output_load{3.040473};
  double ambient_temperature_c{25.0};
  double thermal_resistance_c_per_w{4.0};
  double thermal_time_constant_s{120.0};
  double overtemperature_trip_c{90.0};
  double overtemperature_reset_c{70.0};
  double stall_speed_threshold{0.20};
  double stall_command_fraction{0.25};
  double stall_trip_time_s{1.0};
  double position_tolerance{0.0005};
  bool position_actuator{false};
};

struct CleaningActuatorMotorParameters
{
  std::array<CleaningMotorSpec, kCleaningActuatorCount> motors{};
  double command_timeout_s{0.25};
};

struct CleaningActuatorMotorInput
{
  double step_s{0.0};
  double command_age_s{0.0};
  double bus_voltage_v{24.0};
  bool actuator_enable{false};
  std::array<double, kCleaningActuatorCount> command{};
  std::array<double, kCleaningActuatorCount> measured_position{};
  std::array<double, kCleaningActuatorCount> measured_speed{};
};

struct CleaningMotorState
{
  double command{0.0};
  double measured_position{0.0};
  double measured_speed{0.0};
  double temperature_c{25.0};
  double current_a{0.0};
  double electrical_power_w{0.0};
  double estimated_output_load{0.0};
  double speed_limit{0.0};
  double stall_elapsed_s{0.0};
  bool current_above_rating{false};
  bool protection_active{false};
  bool position_actuator{false};
  CleaningMotorFault fault{CleaningMotorFault::kNone};
};

struct CleaningActuatorMotorOutput
{
  std::array<CleaningMotorState, kCleaningActuatorCount> motors{};
  double total_current_a{0.0};
  double total_power_w{0.0};
  bool fault_active{false};
  bool command_fresh{false};
};

struct CleaningActuatorTelemetrySnapshot
{
  CleaningActuatorMotorOutput output{};
  bool physics_update_stale{true};
  std::uint64_t telemetry_sequence{0};
  std::uint64_t physics_update_sequence{0};
};

/// Pure freshness gate for the wall-clock telemetry publisher. The caller
/// supplies monotonic seconds so normal heartbeat, staleness and recovery are
/// deterministic in unit tests without sleeping.
class CleaningActuatorTelemetryGate
{
public:
  explicit CleaningActuatorTelemetryGate(double stale_timeout_s = 0.75);

  void Update(const CleaningActuatorMotorOutput & output, double wall_time_s);
  CleaningActuatorTelemetrySnapshot Snapshot(double wall_time_s);

private:
  CleaningActuatorMotorOutput latest_output_{};
  double stale_timeout_s_{0.75};
  double last_physics_update_wall_time_s_{0.0};
  std::uint64_t physics_update_sequence_{0};
  std::uint64_t telemetry_sequence_{0};
  bool update_seen_{false};
};

CleaningActuatorMotorParameters DefaultCleaningActuatorMotorParameters();

/// Serialize one atomic physics-update snapshot.  Command and signed measured
/// state therefore share the same sample as current, load and fault status.
std::string CleaningActuatorMotorStatusJson(
  const std::string & model_name,
  const CleaningActuatorMotorOutput & output,
  bool physics_update_stale);

/// Fixed-layout, one-message telemetry transport.  Header indices are:
/// schema, telemetry heartbeat sequence, physics revision, physics stale,
/// command fresh, aggregate fault, total current and total power.  Each motor
/// then contributes command, measured position, measured speed, current,
/// temperature, electrical power, load, speed limit, protection, fault enum
/// and position-mode in actuator order.
std::array<double, kCleaningTelemetryValueCount> CleaningActuatorTelemetryVector(
  const CleaningActuatorTelemetrySnapshot & snapshot);

class CleaningActuatorMotorCore
{
public:
  explicit CleaningActuatorMotorCore(
    CleaningActuatorMotorParameters parameters =
    DefaultCleaningActuatorMotorParameters());

  const CleaningActuatorMotorParameters & Parameters() const;
  CleaningActuatorMotorOutput Step(const CleaningActuatorMotorInput & input);
  bool ResetFaults();

private:
  CleaningActuatorMotorParameters parameters_;
  std::array<CleaningMotorState, kCleaningActuatorCount> states_{};
  std::array<bool, kCleaningActuatorCount> latched_stall_{};
  std::array<bool, kCleaningActuatorCount> latched_overtemperature_{};
  bool commands_idle_{true};
};

}  // namespace sanitation_gazebo_control
