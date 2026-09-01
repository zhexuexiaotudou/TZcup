// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include "sanitation_gazebo_auxiliary/SqueegeeComplianceCore.hh"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sanitation_gazebo_auxiliary
{

bool SqueegeeComplianceCore::Valid(
    const ComplianceAxisParameters &_parameters)
{
  return std::isfinite(_parameters.stiffness) &&
      _parameters.stiffness > 0.0 &&
      std::isfinite(_parameters.damping) &&
      _parameters.damping >= 0.0 &&
      std::isfinite(_parameters.reference) &&
      std::isfinite(_parameters.maximumEffort) &&
      _parameters.maximumEffort > 0.0;
}

double SqueegeeComplianceCore::Effort(
    const ComplianceAxisParameters &_parameters,
    const double _position,
    const double _velocity)
{
  if (!Valid(_parameters) || !std::isfinite(_position) ||
      !std::isfinite(_velocity))
  {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::clamp(
      _parameters.stiffness * (_parameters.reference - _position) -
      _parameters.damping * _velocity,
      -_parameters.maximumEffort,
      _parameters.maximumEffort);
}

}  // namespace sanitation_gazebo_auxiliary
