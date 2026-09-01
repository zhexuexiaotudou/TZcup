// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include "sanitation_gazebo_auxiliary/LightingCore.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sanitation_gazebo_auxiliary
{

bool LightingOutputs::operator==(const LightingOutputs &_other) const
{
  return this->workOn == _other.workOn &&
      this->tailOn == _other.tailOn &&
      this->warningOn == _other.warningOn;
}

bool LightingOutputs::operator!=(const LightingOutputs &_other) const
{
  return !(*this == _other);
}

LightingCore::LightingCore(const LightingConfig &_config)
    : config(_config)
{
  if (!std::isfinite(this->config.warningFrequencyHz) ||
      this->config.warningFrequencyHz <= 0.0)
  {
    throw std::invalid_argument("warningFrequencyHz must be finite and positive");
  }
  if (!std::isfinite(this->config.warningDutyCycle) ||
      this->config.warningDutyCycle < 0.0 ||
      this->config.warningDutyCycle > 1.0)
  {
    throw std::invalid_argument("warningDutyCycle must be in [0, 1]");
  }
}

LightingOutputs LightingCore::Evaluate(
    const double _simulationSeconds,
    const LightingInputs &_inputs) const
{
  const double safeTime =
      std::isfinite(_simulationSeconds) ? std::max(0.0, _simulationSeconds) : 0.0;
  const double phase = std::fmod(
      safeTime * this->config.warningFrequencyHz, 1.0);
  const bool flashOn = phase < this->config.warningDutyCycle;
  const bool warningDemand =
      _inputs.warningRequested || _inputs.emergencyStopLatched;

  LightingOutputs outputs;
  outputs.workOn = _inputs.workRequested &&
      _inputs.safetyPowerAvailable && !_inputs.emergencyStopLatched;
  outputs.tailOn = _inputs.tailRequested;
  outputs.warningOn = _inputs.safetyPowerAvailable && warningDemand && flashOn;
  return outputs;
}

}  // namespace sanitation_gazebo_auxiliary
