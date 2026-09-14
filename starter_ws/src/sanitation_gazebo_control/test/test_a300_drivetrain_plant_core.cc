// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "sanitation_gazebo_control/A300DrivetrainPlantCore.hh"

namespace
{
using sanitation_gazebo_control::A300DrivetrainPlantCore;
using sanitation_gazebo_control::A300DrivetrainPlantInput;
using sanitation_gazebo_control::A300DrivetrainPlantParameters;
using sanitation_gazebo_control::A300DrivetrainStopReason;

void Require(const bool condition, const std::string & message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool Near(const double lhs, const double rhs, const double tolerance = 1e-9)
{
  return std::abs(lhs - rhs) <= tolerance;
}

A300DrivetrainPlantInput NominalInput()
{
  A300DrivetrainPlantInput input;
  input.step_s = 0.1;
  input.command_age_s = 0.0;
  input.bus_voltage_v = 25.6;
  input.actuator_enable = true;
  input.commanded_speed_rad_s.fill(8.0);
  input.measured_speed_rad_s.fill(0.0);
  return input;
}

void TestOverspeedAndContinuousCurrentLimits()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.25;
  input.commanded_speed_rad_s.fill(100.0);
  const auto output = plant.Step(input);
  const double expected_maximum = 2.0 / 0.1651;
  Require(output.drive_permitted, "nominal drive must be permitted");
  Require(output.current_limited, "four-wheel continuous current must limit torque");
  Require(!output.power_limited, "zero-speed command must not be power limited");
  Require(Near(output.estimated_battery_current_a, 60.0, 1e-8),
    "battery current must remain at the published continuous boundary");
  for (std::size_t index = 0; index < output.wheel_torque_nm.size(); ++index) {
    Require(Near(output.limited_command_rad_s[index], expected_maximum),
      "wheel command must enforce the 2 m/s control-radius boundary");
    Require(std::abs(output.estimated_motor_current_a[index]) <= 17.0 + 1e-9,
      "per-motor continuous current boundary exceeded");
  }
}

void TestTorqueSpeedAndAggregatePowerLimits()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.25;
  input.commanded_speed_rad_s.fill(-100.0);
  input.measured_speed_rad_s.fill(10.0);
  const auto output = plant.Step(input);
  Require(output.power_limited, "high-speed torque must use the power envelope");
  Require(output.total_mechanical_power_w <= 1080.0 + 1e-8,
    "aggregate motor-output power boundary exceeded");
  Require(output.estimated_battery_current_a <= 60.0 + 1e-8,
    "aggregate continuous battery-current boundary exceeded");
}

void TestTorqueSlew()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.01;
  input.commanded_speed_rad_s.fill(100.0);
  const auto output = plant.Step(input);
  for (const double torque : output.wheel_torque_nm) {
    Require(Near(torque, 4.0), "drive torque must obey the engineering slew rate");
  }
}

void TestPersistentSpeedErrorAccumulatesBoundedIntegralTorque()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.1;
  input.commanded_speed_rad_s.fill(1.0);
  input.measured_speed_rad_s.fill(0.0);
  const auto first = plant.Step(input);
  const auto second = plant.Step(input);
  Require(second.wheel_torque_nm[0] > first.wheel_torque_nm[0],
    "persistent speed error must increase PI torque before any limit");
  Require(second.wheel_torque_nm[0] <= 59.5 + 1e-9,
    "integral torque must remain bounded by the low-speed torque envelope");
}

void TestSmallStepSlewDoesNotCreateReverseIntegral()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.001;
  input.commanded_speed_rad_s.fill(1.0);
  input.measured_speed_rad_s.fill(0.0);
  const auto accelerating = plant.Step(input);
  input.measured_speed_rad_s.fill(1.0);
  const auto settled_error = plant.Step(input);
  Require(accelerating.wheel_torque_nm[0] > 0.0,
    "small-step PI setup must produce forward slew torque");
  Require(settled_error.wheel_torque_nm[0] > 0.0,
    "normal slew tracking must not erase or reverse the first I increment");
  Require(settled_error.wheel_torque_nm[0] < accelerating.wheel_torque_nm[0],
    "with zero P error the small retained I torque must pull the slew output down");
}

void TestAccumulatedIntegralUnwindsOnReverseError()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.1;
  input.commanded_speed_rad_s.fill(1.0);
  input.measured_speed_rad_s.fill(0.0);
  plant.Step(input);
  plant.Step(input);
  input.commanded_speed_rad_s.fill(-1.0);
  const auto reversed = plant.Step(input);
  Require(Near(reversed.wheel_torque_nm[0], -11.4, 1e-9),
    "reverse error must reduce existing positive I without retaining propulsion");
}

void TestAggregateSaturationFreezesIntegralAndAllowsReverseUnwind()
{
  A300DrivetrainPlantParameters parameters;
  parameters.wheel_side_torque_constant_nm_per_a = 1.0;
  parameters.continuous_current_per_motor_a = 100.0;
  parameters.continuous_battery_current_a = 1.0;
  parameters.torque_slew_rate_nm_per_s = 1000000.0;
  A300DrivetrainPlantCore plant(parameters);
  auto input = NominalInput();
  input.commanded_speed_rad_s.fill(1.0);
  input.measured_speed_rad_s.fill(0.0);
  const auto first = plant.Step(input);
  const auto second = plant.Step(input);
  Require(first.current_limited && second.current_limited,
    "aggregate battery-current scale setup failed");
  Require(Near(second.wheel_torque_nm[0], first.wheel_torque_nm[0], 1e-9),
    "aggregate saturation must reject an I increment that increases saturation");
  input.commanded_speed_rad_s.fill(-1.0);
  const auto reversed = plant.Step(input);
  Require(reversed.wheel_torque_nm[0] < 0.0,
    "opposite error must be able to unwind through aggregate saturation");
}

void TestNeutralSafetyAndResetClearIntegralWithoutRestartImpulse()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.commanded_speed_rad_s.fill(1.0);
  input.measured_speed_rad_s.fill(0.0);
  for (int index = 0; index < 3; ++index) {
    plant.Step(input);
  }
  input.commanded_speed_rad_s.fill(0.0);
  const auto neutral = plant.Step(input);
  Require(Near(neutral.wheel_torque_nm[0], 0.0),
    "neutral permitted command must clear accumulated I propulsion");

  input.commanded_speed_rad_s.fill(1.0);
  for (int index = 0; index < 3; ++index) {
    plant.Step(input);
  }
  input.emergency_stop = true;
  input.measured_speed_rad_s.fill(0.0);
  const auto stopped = plant.Step(input);
  Require(Near(stopped.wheel_torque_nm[0], 0.0), "estop must clear propulsion immediately");
  input.emergency_stop = false;
  input.commanded_speed_rad_s.fill(0.0);
  const auto released = plant.Step(input);
  Require(Near(released.wheel_torque_nm[0], 0.0), "estop release must not reuse integral propulsion");
  plant.Reset();
  const auto reset = plant.Step(input);
  Require(Near(reset.wheel_torque_nm[0], 0.0), "Reset must clear integral propulsion");
}

void TestEveryNonPermittedPathClearsIntegral()
{
  const auto verify = [](const auto configure, const std::string & description) {
      A300DrivetrainPlantCore plant;
      auto input = NominalInput();
      input.commanded_speed_rad_s.fill(1.0);
      input.measured_speed_rad_s.fill(0.0);
      for (int index = 0; index < 3; ++index) {
        plant.Step(input);
      }
      configure(input);
      const auto inhibited = plant.Step(input);
      Require(!inhibited.drive_permitted, description + " must inhibit propulsion");
      input = NominalInput();
      input.commanded_speed_rad_s.fill(0.0);
      input.measured_speed_rad_s.fill(0.0);
      const auto released = plant.Step(input);
      Require(Near(released.wheel_torque_nm[0], 0.0),
        description + " must clear I before a permitted neutral cycle");
    };
  verify([](auto & input) {input.actuator_enable = false;}, "disabled actuator");
  verify([](auto & input) {input.emergency_stop = true;}, "emergency stop");
  verify([](auto & input) {input.command_age_s = 0.51;}, "command timeout");
  verify([](auto & input) {input.motor_fault[0] = true;}, "motor fault");
  verify([](auto & input) {
    input.bus_voltage_v = std::numeric_limits<double>::quiet_NaN();
  }, "invalid input");
}

void TestTimeoutDropsPropulsionThenRampsBrake()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.step_s = 0.1;
  const auto driving = plant.Step(input);
  Require(driving.wheel_torque_nm[0] > 0.0, "setup drive torque missing");

  input.command_age_s = 0.51;
  input.step_s = 0.04;
  input.measured_speed_rad_s.fill(4.0);
  auto stopped = plant.Step(input);
  Require(stopped.stop_reason == A300DrivetrainStopReason::kCommandTimeout,
    "stale command must fail closed");
  Require(!stopped.resistive_brake_active, "brake activated before response delay");
  Require(Near(stopped.wheel_torque_nm[0], 0.0),
    "propulsion must be removed immediately on timeout");

  stopped = plant.Step(input);
  Require(stopped.resistive_brake_active, "brake must activate at response delay");
  input.step_s = 0.06;
  stopped = plant.Step(input);
  Require(stopped.wheel_torque_nm[0] < 0.0,
    "brake torque must oppose forward wheel motion");
  Require(std::abs(stopped.wheel_torque_nm[0]) <= 32.0 + 1e-9,
    "engineering service-brake torque limit exceeded");
}

void TestEmergencyStopAndMotorFaultAreGlobal()
{
  A300DrivetrainPlantCore estop_plant;
  auto input = NominalInput();
  input.emergency_stop = true;
  const auto estop = estop_plant.Step(input);
  Require(estop.stop_reason == A300DrivetrainStopReason::kEmergencyStop,
    "emergency stop reason missing");
  Require(!estop.drive_permitted, "emergency stop must inhibit drive");

  A300DrivetrainPlantCore fault_plant;
  input.emergency_stop = false;
  input.motor_fault[2] = true;
  const auto fault = fault_plant.Step(input);
  Require(fault.stop_reason == A300DrivetrainStopReason::kMotorFault,
    "any wheel fault must stop the whole drivetrain");
  for (const double torque : fault.wheel_torque_nm) {
    Require(Near(torque, 0.0), "motor fault must remove all propulsion torque");
  }
}

void TestDisabledBrakeCannotInjectEnergyNearZero()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.actuator_enable = false;
  input.step_s = 0.25;

  input.measured_speed_rad_s.fill(0.06);
  auto output = plant.Step(input);
  Require(output.resistive_brake_active, "disabled drivetrain brake must activate");
  Require(output.wheel_torque_nm[0] < 0.0,
    "positive wheel speed must receive negative brake torque");
  Require(output.wheel_torque_nm[0] * input.measured_speed_rad_s[0] <= 0.0,
    "brake must not inject mechanical energy");

  input.measured_speed_rad_s.fill(-0.06);
  output = plant.Step(input);
  Require(output.wheel_torque_nm[0] > 0.0,
    "negative wheel speed must receive positive brake torque");
  Require(output.wheel_torque_nm[0] * input.measured_speed_rad_s[0] <= 0.0,
    "reverse brake must not inject mechanical energy");

  input.measured_speed_rad_s.fill(0.01);
  output = plant.Step(input);
  Require(Near(output.wheel_torque_nm[0], 0.0),
    "near-zero wheel speed must not receive a sign-flipping brake impulse");
}

void TestInvalidInputCannotProduceNanOrDrive()
{
  A300DrivetrainPlantCore plant;
  auto input = NominalInput();
  input.bus_voltage_v = std::numeric_limits<double>::quiet_NaN();
  const auto output = plant.Step(input);
  Require(output.stop_reason == A300DrivetrainStopReason::kInvalidInput,
    "invalid input must report a fail-safe reason");
  Require(!output.drive_permitted, "invalid input must not permit drive");
  Require(std::isfinite(output.estimated_battery_current_a),
    "invalid voltage must not propagate NaN to telemetry");
  for (const double torque : output.wheel_torque_nm) {
    Require(Near(torque, 0.0), "invalid input must produce zero torque");
  }
}
}  // namespace

int main()
{
  try {
    TestOverspeedAndContinuousCurrentLimits();
    TestTorqueSpeedAndAggregatePowerLimits();
    TestTorqueSlew();
    TestPersistentSpeedErrorAccumulatesBoundedIntegralTorque();
    TestSmallStepSlewDoesNotCreateReverseIntegral();
    TestAccumulatedIntegralUnwindsOnReverseError();
    TestAggregateSaturationFreezesIntegralAndAllowsReverseUnwind();
    TestNeutralSafetyAndResetClearIntegralWithoutRestartImpulse();
    TestEveryNonPermittedPathClearsIntegral();
    TestTimeoutDropsPropulsionThenRampsBrake();
    TestEmergencyStopAndMotorFaultAreGlobal();
    TestDisabledBrakeCannotInjectEnergyNearZero();
    TestInvalidInputCannotProduceNanOrDrive();
  } catch (const std::exception & error) {
    std::cerr << "A300 drivetrain plant core test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "A300 drivetrain plant core tests passed\n";
  return EXIT_SUCCESS;
}
