// Copyright 2026 Sanitation Vehicle Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

namespace sanitation_scan_refiner
{

inline std::pair<double, double> smoothAnchor(
  const std::pair<double, double> & previous,
  const std::pair<double, double> & measurement,
  const double alpha)
{
  const double bounded = std::clamp(alpha, 0.0, 1.0);
  return {
    previous.first + bounded * (measurement.first - previous.first),
    previous.second + bounded * (measurement.second - previous.second)};
}

inline bool measurementsConsistent(
  const std::pair<double, double> & first,
  const std::pair<double, double> & second,
  const double maximum_disagreement_m)
{
  return std::hypot(first.first - second.first, first.second - second.second) <=
         std::max(0.0, maximum_disagreement_m);
}

inline double propagateHeading(
  const double measured_heading, const double current_local_heading,
  const double measurement_local_heading)
{
  const double angle = measured_heading + current_local_heading - measurement_local_heading;
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline std::pair<double, double> worldToMap(
  const double world_x, const double world_y,
  const double translation_x, const double translation_y,
  const double rotation_yaw)
{
  const double cosine = std::cos(rotation_yaw);
  const double sine = std::sin(rotation_yaw);
  return {
    translation_x + cosine * world_x - sine * world_y,
    translation_y + sine * world_x + cosine * world_y};
}

inline double worldHeadingToMap(const double world_heading, const double rotation_yaw)
{
  const double heading = world_heading + rotation_yaw;
  return std::atan2(std::sin(heading), std::cos(heading));
}

}  // namespace sanitation_scan_refiner
