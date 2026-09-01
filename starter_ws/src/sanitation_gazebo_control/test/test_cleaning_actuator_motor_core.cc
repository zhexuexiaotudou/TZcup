#include "sanitation_gazebo_control/CleaningActuatorMotorCore.hh"

#include <cmath>
#include <stdexcept>
#include <string>

using sanitation_gazebo_control::CleaningActuatorIndex;
using sanitation_gazebo_control::CleaningActuatorMotorCore;
using sanitation_gazebo_control::CleaningActuatorMotorInput;
using sanitation_gazebo_control::CleaningActuatorMotorOutput;
using sanitation_gazebo_control::CleaningActuatorMotorStatusJson;
using sanitation_gazebo_control::CleaningActuatorTelemetryVector;
using sanitation_gazebo_control::CleaningActuatorTelemetryGate;
using sanitation_gazebo_control::CleaningActuatorTelemetrySnapshot;
using sanitation_gazebo_control::CleaningMotorFault;
using sanitation_gazebo_control::DefaultCleaningActuatorMotorParameters;
using sanitation_gazebo_control::kCleaningTelemetrySchemaVersion;
using sanitation_gazebo_control::kCleaningTelemetryValueCount;
using sanitation_gazebo_control::kCleaningActuatorCount;
using sanitation_gazebo_control::kCleaningTelemetryFieldsPerMotor;
using sanitation_gazebo_control::kCleaningTelemetryHeaderCount;

namespace
{
void Require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

std::size_t Index(CleaningActuatorIndex index)
{
  return static_cast<std::size_t>(index);
}

CleaningActuatorMotorInput ReadyInput()
{
  CleaningActuatorMotorInput input;
  input.step_s = 0.02;
  input.command_age_s = 0.0;
  input.bus_voltage_v = 24.0;
  input.actuator_enable = true;
  return input;
}
}  // namespace

int main()
{
  CleaningActuatorTelemetryGate telemetryGate;
  CleaningActuatorMotorOutput healthyTelemetry;
  healthyTelemetry.command_fresh = true;
  auto telemetry = telemetryGate.Snapshot(1.0);
  Require(telemetry.physics_update_stale && telemetry.output.fault_active,
    "telemetry must start fail-closed before the first physics update");
  const auto startupVector = CleaningActuatorTelemetryVector(telemetry);
  Require(startupVector.size() == kCleaningTelemetryValueCount &&
    startupVector[0] == kCleaningTelemetrySchemaVersion &&
    startupVector[2] == 0.0 && startupVector[3] == 1.0 &&
    startupVector[4] == 0.0 && startupVector[5] == 1.0,
    "startup heartbeat must preserve the fail-closed 63-slot header");
  for (std::size_t index = 0; index < kCleaningActuatorCount; ++index) {
    const std::size_t modeIndex = kCleaningTelemetryHeaderCount +
      index * kCleaningTelemetryFieldsPerMotor + 10;
    Require(startupVector[modeIndex] ==
      (index == static_cast<std::size_t>(CleaningActuatorIndex::kLift) ? 1.0 : 0.0),
      "startup heartbeat must preserve the frozen actuator-mode layout");
  }
  telemetryGate.Update(healthyTelemetry, 1.0);
  telemetry = telemetryGate.Snapshot(1.749);
  Require(!telemetry.physics_update_stale && !telemetry.output.fault_active,
    "physics telemetry must remain healthy through 0.749 s");
  Require(telemetry.physics_update_sequence == 1,
    "first physics update must produce sequence one");
  telemetry = telemetryGate.Snapshot(1.751);
  Require(telemetry.physics_update_stale && telemetry.output.fault_active,
    "physics telemetry must fail closed after the 0.75 s simulation window");
  telemetryGate.Update(healthyTelemetry, 1.76);
  telemetry = telemetryGate.Snapshot(1.76);
  Require(!telemetry.physics_update_stale && !telemetry.output.fault_active,
    "fresh physics must recover telemetry after a stale interval");
  Require(telemetry.physics_update_sequence == 2,
    "physics sequence must increase exactly once per accepted update");

  // Deterministically model the old TelemetryLoop interleaving: it sampled
  // wall time before waiting for the telemetry mutex, then a physics update
  // stored a newer time before Snapshot received the old sample.  The gate is
  // intentionally fail-closed for that backwards timestamp.  Sampling again
  // after the update (the fixed system ordering) must stay healthy without
  // changing the 0.75 s stale timeout.
  CleaningActuatorTelemetryGate orderingGate;
  orderingGate.Update(healthyTelemetry, 10.001);
  const auto preLockTimestampSnapshot = orderingGate.Snapshot(10.000);
  Require(
    preLockTimestampSnapshot.physics_update_stale &&
    preLockTimestampSnapshot.output.fault_active,
    "a timestamp captured before a newer locked update must reproduce "
    "the old false stale");
  const auto postLockTimestampSnapshot = orderingGate.Snapshot(10.002);
  Require(
    !postLockTimestampSnapshot.physics_update_stale &&
    !postLockTimestampSnapshot.output.fault_active,
    "a timestamp captured after the locked update must preserve fresh "
    "physics telemetry");

  const auto initialVector = CleaningActuatorTelemetryVector(telemetry);
  Require(initialVector.size() == kCleaningTelemetryValueCount &&
    initialVector[0] == kCleaningTelemetrySchemaVersion &&
    initialVector[1] == 4.0 && initialVector[2] == 2.0 &&
    initialVector[3] == 0.0,
    "typed telemetry header/layout contract missing");
  for (const double value : initialVector) {
    Require(std::isfinite(value), "healthy typed telemetry must be finite");
  }

  const auto parameters = DefaultCleaningActuatorMotorParameters();
  const auto brush = Index(CleaningActuatorIndex::kLeftSideBrush);
  const auto lift = Index(CleaningActuatorIndex::kLift);
  const auto pump = Index(CleaningActuatorIndex::kRecoveryPump);
  Require(std::abs(parameters.motors[brush].no_load_speed - 14.6607657) < 1e-5,
    "Pololu 140 rpm speed boundary missing");
  Require(parameters.motors[brush].rated_current_a == 0.75,
    "Pololu continuous-current recommendation missing");
  Require(parameters.motors[brush].stall_current_a == 3.0,
    "Pololu stall current missing");
  Require(parameters.motors[lift].stall_current_a == 1.0 &&
    parameters.motors[lift].stall_output_load == 300.0,
    "Actuonix current/force boundaries missing");
  Require(parameters.motors[pump].rated_current_a == 6.0 &&
    parameters.motors[pump].stall_current_a == 10.0,
    "Jabsco operating current/fuse boundary missing");

  CleaningActuatorMotorCore core(parameters);
  auto input = ReadyInput();
  input.command[brush] = 10.0;
  input.measured_speed[brush] = 10.0;
  auto output = core.Step(input);
  Require(output.command_fresh && !output.fault_active,
    "healthy tracking must remain permitted");
  Require(output.motors[brush].current_a > 0.0 &&
    output.motors[brush].current_a < parameters.motors[brush].rated_current_a,
    "tracking current must be positive and below rating");
  Require(output.motors[brush].estimated_output_load < 1e-9,
    "zero speed error must not invent output torque");
  Require(output.motors[brush].command == 10.0 &&
    output.motors[brush].measured_speed == 10.0 &&
    !output.motors[brush].position_actuator,
    "atomic motor output must retain signed command and measured speed");
  const std::string statusJson = CleaningActuatorMotorStatusJson(
    "test_vehicle", output, false);
  Require(statusJson.find("\"name\":\"left_side_brush\"") != std::string::npos &&
    statusJson.find("\"control_mode\":\"velocity\"") != std::string::npos &&
    statusJson.find("\"command\":10.000000") != std::string::npos &&
    statusJson.find("\"measured_speed\":10.000000") != std::string::npos &&
    statusJson.find("\"physics_update_stale\":false") != std::string::npos,
    "status_json must publish same-update command and measured speed");
  CleaningActuatorTelemetrySnapshot motorSnapshot;
  motorSnapshot.output = output;
  motorSnapshot.telemetry_sequence = 77;
  motorSnapshot.physics_update_sequence = 123;
  motorSnapshot.physics_update_stale = false;
  const auto motorVector = CleaningActuatorTelemetryVector(motorSnapshot);
  const std::size_t brushBase = sanitation_gazebo_control::kCleaningTelemetryHeaderCount;
  Require(motorVector.size() == 63 && motorVector[1] == 77.0 &&
    motorVector[2] == 123.0 &&
    motorVector[brushBase + 0] == 10.0 &&
    motorVector[brushBase + 2] == 10.0 &&
    motorVector[brushBase + 3] == output.motors[brush].current_a &&
    motorVector[brushBase + 9] == 0.0 && motorVector[brushBase + 10] == 0.0,
    "typed snapshot must preserve same-update command/state/current/fault/mode");

  input.measured_speed[brush] = 0.0;
  output = core.Step(input);
  Require(output.motors[brush].current_a > parameters.motors[brush].rated_current_a,
    "blocked rotor must draw over rated current");
  Require(output.motors[brush].estimated_output_load > 0.0,
    "blocked rotor must estimate torque");
  for (int step = 0; step < 60; ++step) {
    output = core.Step(input);
    if (output.motors[brush].fault == CleaningMotorFault::kStall) {break;}
  }
  Require(output.motors[brush].fault == CleaningMotorFault::kStall,
    "blocked rotor must latch stall protection");
  Require(output.motors[brush].stall_elapsed_s >= 1.0 &&
    output.motors[brush].stall_elapsed_s <= 1.02,
    "blocked side brush must trip at the frozen <=1.0 s stall boundary");
  Require(output.fault_active, "one motor fault must raise aggregate fault");
  Require(!core.ResetFaults(), "active command must block fault reset");

  input.command.fill(0.0);
  input.measured_speed.fill(0.0);
  output = core.Step(input);
  Require(core.ResetFaults(), "idle cool system must accept explicit reset");
  output = core.Step(input);
  Require(!output.fault_active, "reset must clear latched stall");

  input.command_age_s = parameters.command_timeout_s + 0.001;
  output = core.Step(input);
  Require(output.fault_active && !output.command_fresh,
    "stale command must fail closed");
  Require(output.motors[pump].fault == CleaningMotorFault::kCommandTimeout,
    "timeout reason must be explicit");

  input = ReadyInput();
  input.command[lift] = 0.10;
  input.measured_position[lift] = 0.0;
  input.measured_speed[lift] = 0.0048;
  output = core.Step(input);
  Require(output.motors[lift].current_a > 0.0,
    "moving lift must have electrical current");
  Require(output.motors[lift].position_actuator,
    "lift telemetry must declare position control mode");
  Require(output.motors[lift].speed_limit <= 0.0060001,
    "12 V lift speed must not scale to the 24 V bus");

  // Keep the published production thermal constants.  The pure core test can
  // advance simulated time deterministically without shortening them, while
  // the live Gazebo gate remains a separate mechanical-stall acceptance.
  CleaningActuatorMotorCore thermalCore(parameters);
  auto thermalInput = ReadyInput();
  thermalInput.step_s = 0.25;
  thermalInput.command[brush] = parameters.motors[brush].no_load_speed;
  thermalInput.measured_speed[brush] = 0.0;
  CleaningActuatorMotorOutput thermalOutput;
  for (int step = 0; step < 2000; ++step) {
    thermalOutput = thermalCore.Step(thermalInput);
    if (thermalOutput.motors[brush].fault == CleaningMotorFault::kOvertemperature) {
      break;
    }
  }
  Require(thermalOutput.motors[brush].fault == CleaningMotorFault::kOvertemperature,
    "production thermal model must latch overtemperature without parameter overrides");
  thermalInput.command.fill(0.0);
  thermalInput.measured_speed.fill(0.0);
  thermalOutput = thermalCore.Step(thermalInput);
  Require(!thermalCore.ResetFaults(),
    "hot overtemperature latch must reject an early explicit reset");
  for (int step = 0; step < 4000; ++step) {
    thermalOutput = thermalCore.Step(thermalInput);
    if (thermalOutput.motors[brush].temperature_c <=
      parameters.motors[brush].overtemperature_reset_c)
    {
      break;
    }
  }
  Require(thermalOutput.motors[brush].temperature_c <=
    parameters.motors[brush].overtemperature_reset_c,
    "idle motor must cool below the production reset threshold");
  Require(thermalCore.ResetFaults(),
    "cooled idle overtemperature latch must accept explicit reset");
  thermalOutput = thermalCore.Step(thermalInput);
  Require(!thermalOutput.fault_active,
    "explicit reset must clear the cooled overtemperature latch");

  auto invalid = parameters;
  invalid.motors[brush].stall_current_a = 0.5;
  bool rejected = false;
  try {CleaningActuatorMotorCore rejectedCore(invalid);} catch (const std::invalid_argument &) {
    rejected = true;
  }
  Require(rejected, "invalid current ordering must be rejected");
  return 0;
}
