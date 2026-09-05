// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.
#ifndef SANITATION_GAZEBO_CONTROL__CONTACT_GATE_CORE_HH_
#define SANITATION_GAZEBO_CONTROL__CONTACT_GATE_CORE_HH_

#include <cstdint>
#include <optional>
#include <set>
#include <string>

namespace sanitation_gazebo_control
{
using CollisionId = std::uint64_t;

inline bool IsExternalCollisionName(
    const std::string &_collisionName,
    const std::string &_vehicleModelName)
{
  return !_collisionName.empty() && !_vehicleModelName.empty() &&
      _collisionName.find(_vehicleModelName) == std::string::npos;
}

inline std::optional<CollisionId> CommonContact(
    const std::set<CollisionId> &_left,
    const std::set<CollisionId> &_right)
{
  for (const auto collision : _left)
  {
    if (_right.count(collision) != 0)
      return collision;
  }
  return std::nullopt;
}

inline bool DynamicBodyEligible(
    const bool _linkResolved,
    const bool _hasStaticAncestor)
{
  return _linkResolved && !_hasStaticAncestor;
}
}  // namespace sanitation_gazebo_control

#endif  // SANITATION_GAZEBO_CONTROL__CONTACT_GATE_CORE_HH_
