// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#pragma once

namespace sanitation_gazebo_control
{
enum class BasePoseSource
{
  kUnavailable,
  kBaseLink,
  kBaseFootprint,
  kModelEntity,
};

constexpr BasePoseSource SelectBasePoseSource(
    const bool _baseLinkAvailable,
    const bool _baseFootprintAvailable,
    const bool _modelEntityAvailable)
{
  if (_baseLinkAvailable)
    return BasePoseSource::kBaseLink;
  if (_baseFootprintAvailable)
    return BasePoseSource::kBaseFootprint;
  if (_modelEntityAvailable)
    return BasePoseSource::kModelEntity;
  return BasePoseSource::kUnavailable;
}

constexpr const char *BasePoseSourceName(const BasePoseSource _source)
{
  switch (_source)
  {
    case BasePoseSource::kBaseLink:
      return "base_link";
    case BasePoseSource::kBaseFootprint:
      return "base_footprint";
    case BasePoseSource::kModelEntity:
      return "model_entity";
    case BasePoseSource::kUnavailable:
    default:
      return "unavailable";
  }
}
}
