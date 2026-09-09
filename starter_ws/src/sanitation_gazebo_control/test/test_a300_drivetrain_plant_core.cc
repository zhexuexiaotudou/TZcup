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
using sanitation_gazebo_control::A300PlanarTwist;
using sanitation_gazebo_control::A300PlanarTwistFromWheelSpeeds;
using sanitation_gazebo_control::A300DrivetrainStopReason;
using sanitation_gazebo_control::A300WheelSpeedsFromPlanarTwist;

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
  const double expected_maximum = 2.0 / 0.1625;
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

void TestPureSkidSteerSpinKinematics()
{
  constexpr double kControlWheelRadiusM = 0.1625;
  constexpr double kWheelTrackM = 0.562;
  constexpr double kTargetYawRateRadS = 0.35;
  const auto wheelSpeeds = A300WheelSpeedsFromPlanarTwist(
    A300PlanarTwist{0.0, kTargetYawRateRadS},
    kControlWheelRadiusM, kWheelTrackM);
  const double expectedSideSpeed =
    kTargetYawRateRadS * (kWheelTrackM * 0.5) / kControlWheelRadiusM;
  Require(Near(wheelSpeeds[0], -expectedSideSpeed),
    "positive yaw must command the left front wheel negative");
  Require(Near(wheelSpeeds[1], expectedSideSpeed),
    "positive yaw must command the right front wheel positive");
  Require(Near(wheelSpeeds[2], -expectedSideSpeed),
    "positive yaw must command the left rear wheel negative");
  Require(Near(wheelSpeeds[3], expectedSideSpeed),
    "positive yaw must command the right rear wheel positive");

  const auto reconstructed = A300PlanarTwistFromWheelSpeeds(
    wheelSpeeds, kControlWheelRadiusM, kWheelTrackM);
  Require(Near(reconstructed.linear_x_mps, 0.0),
    "symmetric spin wheel speeds must not reconstruct linear motion");
  Require(Near(reconstructed.angular_z_rad_s, kTargetYawRateRadS),
    "spin wheel-speed inverse must reconstruct the requested yaw rate");
}

void TestCounterRotationUsesBreakawayGainOnly()
{
  constexpr double kWheelCommandRadS = 0.4323076923076923;

  A300DrivetrainPlantCore spin_plant;
  auto spin_input = NominalInput();
  spin_input.step_s = 0.25;
  spin_input.commanded_speed_rad_s = {
    -kWheelCommandRadS, kWheelCommandRadS,
    -kWheelCommandRadS, kWheelCommandRadS};
  const auto spin = spin_plant.Step(spin_input);
  const double expected_spin_torque_nm = 120.0 * kWheelCommandRadS;
  Require(Near(spin.wheel_torque_nm[0], -expected_spin_torque_nm, 1e-8),
    "counter-rotation must use the dedicated breakaway gain");
  Require(Near(spin.wheel_torque_nm[1], expected_spin_torque_nm, 1e-8),
    "counter-rotation torque signs must follow wheel commands");
  Require(!spin.current_limited && !spin.power_limited,
    "nominal spin breakaway must stay inside current and power boundaries");

  A300DrivetrainPlantCore high_rate_spin_plant;
  auto high_rate_spin_input = spin_input;
  high_rate_spin_input.commanded_speed_rad_s = {-1.0, 1.0, -1.0, 1.0};
  const auto high_rate_spin = high_rate_spin_plant.Step(high_rate_spin_input);
  for (const double torque_nm : high_rate_spin.wheel_torque_nm) {
    Require(Near(std::abs(torque_nm), 52.5, 1e-8),
      "four-wheel counter-rotation must share the 60 A battery-current limit");
  }
  Require(high_rate_spin.current_limited,
    "counter-rotation above the continuous envelope must report current limiting");
  Require(Near(high_rate_spin.estimated_battery_current_a, 60.0, 1e-8),
    "counter-rotation must not exceed the aggregate continuous battery current");

  A300DrivetrainPlantCore straight_plant;
  auto straight_input = NominalInput();
  straight_input.step_s = 0.25;
  straight_input.commanded_speed_rad_s.fill(kWheelCommandRadS);
  const auto straight = straight_plant.Step(straight_input);
  const double expected_straight_torque_nm = 12.0 * kWheelCommandRadS;
  for (const double torque_nm : straight.wheel_torque_nm) {
    Require(Near(torque_nm, expected_straight_torque_nm, 1e-8),
      "straight drive must retain the general speed-error gain");
  }
}
}  // namespace

int main()
{
  try {
    TestOverspeedAndContinuousCurrentLimits();
    TestTorqueSpeedAndAggregatePowerLimits();
    TestTorqueSlew();
    TestTimeoutDropsPropulsionThenRampsBrake();
    TestEmergencyStopAndMotorFaultAreGlobal();
    TestDisabledBrakeCannotInjectEnergyNearZero();
    TestInvalidInputCannotProduceNanOrDrive();
    TestPureSkidSteerSpinKinematics();
    TestCounterRotationUsesBreakawayGainOnly();
  } catch (const std::exception & error) {
    std::cerr << "A300 drivetrain plant core test failed: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "A300 drivetrain plant core tests passed\n";
  return EXIT_SUCCESS;
}
