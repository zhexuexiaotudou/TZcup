// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include "sanitation_gazebo_auxiliary/EstopLatchCore.hh"

namespace sanitation_gazebo_auxiliary
{

EstopLatchCore::EstopLatchCore(const bool _initiallyLatched)
    : latched(_initiallyLatched)
{
}

bool EstopLatchCore::Update(
    const bool _emergencyInputAsserted,
    const bool _resetRequested,
    const bool _resetAllowed)
{
  if (_emergencyInputAsserted)
  {
    this->latched = true;
  }
  else if (_resetRequested && _resetAllowed)
  {
    this->latched = false;
  }
  return this->latched;
}

bool EstopLatchCore::Latched() const
{
  return this->latched;
}

}  // namespace sanitation_gazebo_auxiliary
