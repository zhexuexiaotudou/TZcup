// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#ifndef SANITATION_GAZEBO_AUXILIARY__LIGHTING_CORE_HH_
#define SANITATION_GAZEBO_AUXILIARY__LIGHTING_CORE_HH_

namespace sanitation_gazebo_auxiliary
{

struct LightingConfig
{
  double warningFrequencyHz{1.0};
  double warningDutyCycle{0.5};
};

struct LightingInputs
{
  bool workRequested{false};
  bool tailRequested{false};
  bool warningRequested{false};
  bool emergencyStopLatched{true};
  bool safetyPowerAvailable{false};
};

struct LightingOutputs
{
  bool workOn{false};
  bool tailOn{false};
  bool warningOn{false};

  bool operator==(const LightingOutputs &_other) const;
  bool operator!=(const LightingOutputs &_other) const;
};

/// Pure, simulation-time driven lighting interlock.
///
/// Work lamps are cut by emergency stop. Tail lamps remain independently
/// controllable so the stopped vehicle stays visible. Warning beacons require
/// safety power and flash deterministically from simulation time; pausing the
/// simulator therefore also pauses the flash phase.
class LightingCore
{
  public: explicit LightingCore(const LightingConfig &_config = {});

  public: LightingOutputs Evaluate(
      double _simulationSeconds,
      const LightingInputs &_inputs) const;

  private: LightingConfig config;
};

}  // namespace sanitation_gazebo_auxiliary

#endif  // SANITATION_GAZEBO_AUXILIARY__LIGHTING_CORE_HH_
