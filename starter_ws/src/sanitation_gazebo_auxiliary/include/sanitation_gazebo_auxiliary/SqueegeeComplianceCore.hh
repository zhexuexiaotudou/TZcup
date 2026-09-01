// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#pragma once

namespace sanitation_gazebo_auxiliary
{

struct ComplianceAxisParameters
{
  double stiffness{0.0};
  double damping{0.0};
  double reference{0.0};
  double maximumEffort{0.0};
};

/// Deterministic spring-damper calculation shared by the Gazebo plugin and
/// unit tests. Positive effort acts in the positive joint-axis direction.
class SqueegeeComplianceCore
{
  public: static bool Valid(const ComplianceAxisParameters &_parameters);

  /// Returns NaN when the measurement or parameters are invalid. The caller
  /// must then withhold the force command rather than writing a fabricated
  /// zero-effort state into the physics engine.
  public: static double Effort(
      const ComplianceAxisParameters &_parameters,
      double _position,
      double _velocity);
};

}  // namespace sanitation_gazebo_auxiliary
