// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <algorithm>
#include <cmath>

namespace sanitation_gazebo_control
{
struct GripperMimicEffortParameters
{
  double multiplier{1.0};
  double offset_rad{0.0};
  double position_gain_nm_rad{4.0};
  double velocity_gain_nm_s_rad{0.03};
  double maximum_effort_nm{12.0};
};

struct GripperMimicEffortOutput
{
  double target_position_rad{0.0};
  double target_velocity_rad_s{0.0};
  double effort_nm{0.0};
  bool valid{false};
};

/// Compute the bounded compliant effort for one mechanically linked finger
/// joint.  Keeping this arithmetic independent of Gazebo makes the sign,
/// multiplier and saturation contract directly testable.
inline GripperMimicEffortOutput ComputeGripperMimicEffort(
  const GripperMimicEffortParameters & parameters,
  const double master_position_rad,
  const double master_velocity_rad_s,
  const double follower_position_rad,
  const double follower_velocity_rad_s)
{
  GripperMimicEffortOutput output;
  if (!std::isfinite(parameters.multiplier) ||
      !std::isfinite(parameters.offset_rad) ||
      !std::isfinite(parameters.position_gain_nm_rad) ||
      !std::isfinite(parameters.velocity_gain_nm_s_rad) ||
      !std::isfinite(parameters.maximum_effort_nm) ||
      parameters.position_gain_nm_rad < 0.0 ||
      parameters.velocity_gain_nm_s_rad < 0.0 ||
      parameters.maximum_effort_nm <= 0.0 ||
      !std::isfinite(master_position_rad) ||
      !std::isfinite(master_velocity_rad_s) ||
      !std::isfinite(follower_position_rad) ||
      !std::isfinite(follower_velocity_rad_s)) {
    return output;
  }

  output.target_position_rad =
    parameters.multiplier * master_position_rad + parameters.offset_rad;
  output.target_velocity_rad_s = parameters.multiplier * master_velocity_rad_s;
  const double raw_effort =
    parameters.position_gain_nm_rad *
      (output.target_position_rad - follower_position_rad) +
    parameters.velocity_gain_nm_s_rad *
      (output.target_velocity_rad_s - follower_velocity_rad_s);
  output.effort_nm = std::clamp(
    raw_effort, -parameters.maximum_effort_nm, parameters.maximum_effort_nm);
  output.valid = true;
  return output;
}
}  // namespace sanitation_gazebo_control
