#include "sanitation_gazebo_control/CleaningActuatorMotorCore.hh"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace sanitation_gazebo_control
{
namespace
{
constexpr double kKgCmToNm = 0.0980665;

bool FiniteArray(const std::array<double, kCleaningActuatorCount> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
    return std::isfinite(value);
  });
}

void ValidateSpec(const CleaningMotorSpec & spec)
{
  const std::array<double, 15> values{
    spec.nominal_voltage_v, spec.no_load_speed, spec.no_load_current_a,
    spec.rated_current_a, spec.stall_current_a, spec.stall_output_load,
    spec.ambient_temperature_c, spec.thermal_resistance_c_per_w,
    spec.thermal_time_constant_s, spec.overtemperature_trip_c,
    spec.overtemperature_reset_c, spec.stall_speed_threshold,
    spec.stall_command_fraction, spec.stall_trip_time_s,
    spec.position_tolerance};
  if (!std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    }) || spec.nominal_voltage_v <= 0.0 || spec.no_load_speed <= 0.0 ||
    spec.no_load_current_a < 0.0 || spec.rated_current_a <= 0.0 ||
    spec.stall_current_a <= spec.rated_current_a ||
    spec.rated_current_a < spec.no_load_current_a ||
    spec.stall_output_load <= 0.0 || spec.thermal_resistance_c_per_w <= 0.0 ||
    spec.thermal_time_constant_s <= 0.0 ||
    spec.overtemperature_trip_c <= spec.overtemperature_reset_c ||
    spec.overtemperature_reset_c <= spec.ambient_temperature_c ||
    spec.stall_speed_threshold < 0.0 || spec.stall_command_fraction <= 0.0 ||
    spec.stall_command_fraction > 1.0 || spec.stall_trip_time_s <= 0.0 ||
    spec.position_tolerance < 0.0)
  {
    throw std::invalid_argument("invalid cleaning actuator motor specification");
  }
}

const char * FaultName(const CleaningMotorFault fault)
{
  switch (fault) {
    case CleaningMotorFault::kNone: return "none";
    case CleaningMotorFault::kCommandTimeout: return "command_timeout";
    case CleaningMotorFault::kStall: return "stall";
    case CleaningMotorFault::kOvertemperature: return "overtemperature";
    case CleaningMotorFault::kInvalidInput: return "invalid_input";
  }
  return "unknown";
}
}  // namespace

std::string CleaningActuatorMotorStatusJson(
  const std::string & model_name,
  const CleaningActuatorMotorOutput & output,
  const bool physics_update_stale)
{
  constexpr std::array<const char *, kCleaningActuatorCount> kMotorNames{
    "left_side_brush", "right_side_brush", "central_roller",
    "cleaning_lift", "recovery_pump"};
  std::ostringstream stream;
  stream << std::boolalpha << std::fixed << std::setprecision(6)
    << "{\"model\":\"" << model_name << "\"," 
    << "\"command_fresh\":" << output.command_fresh << ','
    << "\"physics_update_stale\":" << physics_update_stale << ','
    << "\"fault_active\":" << output.fault_active << ','
    << "\"total_current_a\":" << output.total_current_a << ','
    << "\"total_power_w\":" << output.total_power_w << ",\"motors\":[";
  for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
    if (index != 0) {stream << ',';}
    const auto & motor = output.motors[index];
    stream << "{\"name\":\"" << kMotorNames[index] << "\"," 
      << "\"control_mode\":\""
      << (motor.position_actuator ? "position" : "velocity") << "\"," 
      << "\"command\":" << motor.command << ','
      << "\"measured_position\":" << motor.measured_position << ','
      << "\"measured_speed\":" << motor.measured_speed << ','
      << "\"current_a\":" << motor.current_a << ','
      << "\"temperature_c\":" << motor.temperature_c << ','
      << "\"estimated_output_load\":" << motor.estimated_output_load << ','
      << "\"speed_limit\":" << motor.speed_limit << ','
      << "\"current_above_rating\":" << motor.current_above_rating << ','
      << "\"protection_active\":" << motor.protection_active << ','
      << "\"fault\":\"" << FaultName(motor.fault) << "\"}";
  }
  stream << "]}";
  return stream.str();
}

std::array<double, kCleaningTelemetryValueCount> CleaningActuatorTelemetryVector(
  const CleaningActuatorTelemetrySnapshot & snapshot)
{
  std::array<double, kCleaningTelemetryValueCount> values{};
  values[0] = kCleaningTelemetrySchemaVersion;
  values[1] = static_cast<double>(snapshot.telemetry_sequence);
  values[2] = static_cast<double>(snapshot.physics_update_sequence);
  values[3] = snapshot.physics_update_stale ? 1.0 : 0.0;
  values[4] = snapshot.output.command_fresh ? 1.0 : 0.0;
  values[5] = snapshot.output.fault_active ? 1.0 : 0.0;
  values[6] = snapshot.output.total_current_a;
  values[7] = snapshot.output.total_power_w;
  for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
    const auto & motor = snapshot.output.motors[index];
    const std::size_t base = kCleaningTelemetryHeaderCount +
      index * kCleaningTelemetryFieldsPerMotor;
    values[base + 0] = motor.command;
    values[base + 1] = motor.measured_position;
    values[base + 2] = motor.measured_speed;
    values[base + 3] = motor.current_a;
    values[base + 4] = motor.temperature_c;
    values[base + 5] = motor.electrical_power_w;
    values[base + 6] = motor.estimated_output_load;
    values[base + 7] = motor.speed_limit;
    values[base + 8] = motor.protection_active ? 1.0 : 0.0;
    values[base + 9] = static_cast<double>(motor.fault);
    values[base + 10] = motor.position_actuator ? 1.0 : 0.0;
  }
  return values;
}

CleaningActuatorTelemetryGate::CleaningActuatorTelemetryGate(
  const double stale_timeout_s)
: stale_timeout_s_(stale_timeout_s)
{
  if (!std::isfinite(stale_timeout_s_) || stale_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument("cleaning telemetry stale timeout must be positive");
  }
  // Control mode is part of the frozen transport schema, not dynamic motor
  // state.  Publish it correctly even on the fail-closed heartbeat emitted
  // before the first physics update; do not synthesize speed, current or load.
  latest_output_.motors[
    static_cast<std::size_t>(CleaningActuatorIndex::kLift)].position_actuator = true;
}

void CleaningActuatorTelemetryGate::Update(
  const CleaningActuatorMotorOutput & output, const double wall_time_s)
{
  if (!std::isfinite(wall_time_s)) {
    return;
  }
  latest_output_ = output;
  last_physics_update_wall_time_s_ = wall_time_s;
  ++physics_update_sequence_;
  update_seen_ = true;
}

CleaningActuatorTelemetrySnapshot CleaningActuatorTelemetryGate::Snapshot(
  const double wall_time_s)
{
  CleaningActuatorTelemetrySnapshot snapshot;
  snapshot.telemetry_sequence = ++telemetry_sequence_;
  snapshot.output = latest_output_;
  snapshot.physics_update_sequence = physics_update_sequence_;
  snapshot.physics_update_stale = !update_seen_ || !std::isfinite(wall_time_s) ||
    wall_time_s < last_physics_update_wall_time_s_ ||
    wall_time_s - last_physics_update_wall_time_s_ >= stale_timeout_s_;
  if (snapshot.physics_update_stale) {
    snapshot.output.fault_active = true;
  }
  return snapshot;
}

CleaningActuatorMotorParameters DefaultCleaningActuatorMotorParameters()
{
  CleaningActuatorMotorParameters parameters;
  CleaningMotorSpec pololu;
  // Pololu item 4694 public 24 V limits: 140 rpm, 0.1 A no-load,
  // 3 A extrapolated stall and 31 kg.cm extrapolated stall torque.
  pololu.no_load_speed = 140.0 * 2.0 * M_PI / 60.0;
  pololu.stall_output_load = 31.0 * kKgCmToNm;
  pololu.rated_current_a = 0.75;  // vendor recommendation: <=25% stall current
  parameters.motors[0] = pololu;
  parameters.motors[1] = pololu;
  parameters.motors[2] = pololu;

  CleaningMotorSpec lift;
  // Actuonix P16-100-256-12-P public limits: 4.8 mm/s, 300 N,
  // 1 A stall at 12 V and 20% maximum duty cycle.
  lift.nominal_voltage_v = 12.0;
  lift.no_load_speed = 0.0048;
  lift.no_load_current_a = 0.08;
  lift.rated_current_a = 0.50;
  lift.stall_current_a = 1.0;
  lift.stall_output_load = 300.0;
  lift.thermal_resistance_c_per_w = 8.0;
  lift.thermal_time_constant_s = 90.0;
  lift.overtemperature_trip_c = 55.0;
  lift.overtemperature_reset_c = 45.0;
  lift.stall_speed_threshold = 0.0002;
  lift.stall_trip_time_s = 1.5;
  lift.position_actuator = true;
  parameters.motors[3] = lift;

  CleaningMotorSpec pump;
  // The Jabsco sheet specifies a 24 V TENV PM motor, 6 A maximum draw,
  // 10 A fuse and TCO. Rotor speed / torque are explicit engineering
  // calibration parameters because the diaphragm-cam ratio is not published.
  pump.no_load_speed = 62.832;
  pump.no_load_current_a = 0.65;
  pump.rated_current_a = 6.0;
  pump.stall_current_a = 10.0;
  pump.stall_output_load = 2.0;
  pump.thermal_resistance_c_per_w = 0.55;
  pump.thermal_time_constant_s = 600.0;
  pump.overtemperature_trip_c = 80.0;
  pump.overtemperature_reset_c = 60.0;
  pump.stall_speed_threshold = 2.0;
  pump.stall_trip_time_s = 3.0;
  parameters.motors[4] = pump;
  return parameters;
}

CleaningActuatorMotorCore::CleaningActuatorMotorCore(
  CleaningActuatorMotorParameters parameters)
: parameters_(parameters)
{
  if (!std::isfinite(parameters_.command_timeout_s) ||
    parameters_.command_timeout_s <= 0.0)
  {
    throw std::invalid_argument("command_timeout_s must be finite and positive");
  }
  for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
    ValidateSpec(parameters_.motors[index]);
    states_[index].temperature_c = parameters_.motors[index].ambient_temperature_c;
  }
}

const CleaningActuatorMotorParameters & CleaningActuatorMotorCore::Parameters() const
{
  return parameters_;
}

CleaningActuatorMotorOutput CleaningActuatorMotorCore::Step(
  const CleaningActuatorMotorInput & input)
{
  CleaningActuatorMotorOutput output;
  const bool valid = std::isfinite(input.step_s) && input.step_s > 0.0 &&
    input.step_s <= 0.25 && std::isfinite(input.command_age_s) &&
    input.command_age_s >= 0.0 && std::isfinite(input.bus_voltage_v) &&
    input.bus_voltage_v > 0.0 && FiniteArray(input.command) &&
    FiniteArray(input.measured_position) && FiniteArray(input.measured_speed);
  output.command_fresh = valid &&
    input.command_age_s <= parameters_.command_timeout_s;
  commands_idle_ = true;

  for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
    const auto & spec = parameters_.motors[index];
    auto & state = states_[index];
    state.command = input.command[index];
    state.measured_position = input.measured_position[index];
    state.measured_speed = input.measured_speed[index];
    state.position_actuator = spec.position_actuator;
    // The 12 V P16 branch is fed by a regulated DC/DC converter; a higher
    // traction bus therefore cannot over-volt that actuator.
    const double applied_voltage_v = std::min(
      input.bus_voltage_v, spec.nominal_voltage_v);
    const double voltage_ratio = std::clamp(
      applied_voltage_v / spec.nominal_voltage_v, 0.0, 1.0);
    state.speed_limit = spec.no_load_speed * voltage_ratio;
    double request_fraction = 0.0;
    if (valid && output.command_fresh && input.actuator_enable) {
      if (spec.position_actuator) {
        const double error = input.command[index] - input.measured_position[index];
        if (std::abs(error) > spec.position_tolerance) {
          request_fraction = 1.0;
        }
      } else {
        request_fraction = std::clamp(
          std::abs(input.command[index]) / std::max(state.speed_limit, 1e-9),
          0.0, 1.0);
      }
    }
    commands_idle_ = commands_idle_ && request_fraction <= 1e-6;

    const double requested_speed = spec.position_actuator ?
      state.speed_limit * request_fraction : std::abs(input.command[index]);
    const double speed_error_fraction = requested_speed <= 1e-9 ? 0.0 :
      std::clamp(
      (requested_speed - std::abs(input.measured_speed[index])) /
      requested_speed, 0.0, 1.0);
    const double load_fraction = request_fraction * speed_error_fraction;
    double current_a = request_fraction <= 1e-9 ? 0.0 :
      spec.no_load_current_a * request_fraction +
      (spec.stall_current_a - spec.no_load_current_a) * load_fraction;
    current_a = std::clamp(current_a, 0.0, spec.stall_current_a);

    const bool stall_candidate = request_fraction >= spec.stall_command_fraction &&
      std::abs(input.measured_speed[index]) <= spec.stall_speed_threshold &&
      current_a >= spec.rated_current_a;
    state.stall_elapsed_s = stall_candidate ?
      state.stall_elapsed_s + (valid ? input.step_s : 0.0) : 0.0;
    if (state.stall_elapsed_s >= spec.stall_trip_time_s) {
      latched_stall_[index] = true;
    }

    const double winding_resistance_ohm =
      spec.nominal_voltage_v / spec.stall_current_a;
    const double copper_loss_w = current_a * current_a * winding_resistance_ohm;
    const double equilibrium_c = spec.ambient_temperature_c +
      copper_loss_w * spec.thermal_resistance_c_per_w;
    if (valid) {
      state.temperature_c +=
        (equilibrium_c - state.temperature_c) *
        (1.0 - std::exp(-input.step_s / spec.thermal_time_constant_s));
    }
    if (state.temperature_c >= spec.overtemperature_trip_c) {
      latched_overtemperature_[index] = true;
    }

    state.current_a = current_a;
    state.electrical_power_w = applied_voltage_v * current_a;
    state.estimated_output_load = spec.stall_output_load *
      (current_a <= spec.no_load_current_a ? 0.0 :
      (current_a - spec.no_load_current_a) /
      (spec.stall_current_a - spec.no_load_current_a));
    state.current_above_rating = current_a > spec.rated_current_a + 1e-9;
    if (!valid) {
      state.fault = CleaningMotorFault::kInvalidInput;
    } else if (!output.command_fresh) {
      state.fault = CleaningMotorFault::kCommandTimeout;
    } else if (latched_overtemperature_[index]) {
      state.fault = CleaningMotorFault::kOvertemperature;
    } else if (latched_stall_[index]) {
      state.fault = CleaningMotorFault::kStall;
    } else {
      state.fault = CleaningMotorFault::kNone;
    }
    state.protection_active = state.fault != CleaningMotorFault::kNone;
    output.fault_active = output.fault_active || state.protection_active;
    output.total_current_a += state.current_a;
    output.total_power_w += state.electrical_power_w;
    output.motors[index] = state;
  }
  return output;
}

bool CleaningActuatorMotorCore::ResetFaults()
{
  if (!commands_idle_) {
    return false;
  }
  for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
    if (states_[index].temperature_c >
      parameters_.motors[index].overtemperature_reset_c)
    {
      return false;
    }
  }
  latched_stall_.fill(false);
  latched_overtemperature_.fill(false);
  for (auto & state : states_) {
    state.stall_elapsed_s = 0.0;
  }
  return true;
}

}  // namespace sanitation_gazebo_control
