// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <cassert>
#include <cmath>
#include <limits>

#include "sanitation_gazebo_control/GripperMimicEffortCore.hh"

using sanitation_gazebo_control::ComputeGripperMimicEffort;
using sanitation_gazebo_control::GripperMimicEffortParameters;

int main()
{
  GripperMimicEffortParameters direct;
  direct.multiplier = 1.0;
  const auto direct_output = ComputeGripperMimicEffort(
    direct, 0.30, 0.20, 0.10, -0.10);
  assert(direct_output.valid);
  assert(std::abs(direct_output.target_position_rad - 0.30) < 1.0e-12);
  assert(std::abs(direct_output.target_velocity_rad_s - 0.20) < 1.0e-12);
  assert(direct_output.effort_nm > 0.0);

  GripperMimicEffortParameters inverse = direct;
  inverse.multiplier = -1.0;
  const auto inverse_output = ComputeGripperMimicEffort(
    inverse, 0.30, 0.20, -0.10, 0.10);
  assert(inverse_output.valid);
  assert(std::abs(inverse_output.target_position_rad + 0.30) < 1.0e-12);
  assert(std::abs(inverse_output.target_velocity_rad_s + 0.20) < 1.0e-12);
  assert(inverse_output.effort_nm < 0.0);

  GripperMimicEffortParameters saturated = direct;
  saturated.position_gain_nm_rad = 1000.0;
  saturated.maximum_effort_nm = 3.5;
  const auto saturated_output = ComputeGripperMimicEffort(
    saturated, 0.8, 0.0, 0.0, 0.0);
  assert(saturated_output.valid);
  assert(std::abs(saturated_output.effort_nm - 3.5) < 1.0e-12);

  const auto invalid_output = ComputeGripperMimicEffort(
    direct, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0, 0.0);
  assert(!invalid_output.valid);
  assert(invalid_output.effort_nm == 0.0);
  return 0;
}
