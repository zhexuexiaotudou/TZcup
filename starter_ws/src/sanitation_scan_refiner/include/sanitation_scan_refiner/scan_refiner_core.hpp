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

#ifndef SANITATION_SCAN_REFINER__SCAN_REFINER_CORE_HPP_
#define SANITATION_SCAN_REFINER__SCAN_REFINER_CORE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sanitation_scan_refiner
{

struct Point2
{
  double x{0.0};
  double y{0.0};
};

struct Pose2
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct SearchStage
{
  double xy_window_m;
  double yaw_window_rad;
  double xy_step_m;
  double yaw_step_rad;
};

struct SearchConfig
{
  std::vector<SearchStage> stages;
  std::size_t minimum_points{60};
  std::size_t maximum_points{240};
  double huber_delta_m{0.08};
  double prior_translation_weight{0.03};
  double prior_yaw_weight{0.02};
  double maximum_score_m{0.12};
  double minimum_improvement_m{0.001};
  double minimum_translation_curvature{0.05};
  double minimum_yaw_curvature{0.01};
};

struct RefinementResult
{
  bool accepted{false};
  bool observable{false};
  std::string reason;
  Pose2 pose;
  double score_m{0.0};
  double prior_score_m{0.0};
  double improvement_m{0.0};
  double covariance_xx{0.0};
  double covariance_yy{0.0};
  double covariance_yaw_yaw{0.0};
  double curvature_x{0.0};
  double curvature_y{0.0};
  double curvature_yaw{0.0};
  std::size_t valid_point_count{0};
  std::size_t evaluated_candidates{0};
};

class OccupancyDistanceField
{
public:
  OccupancyDistanceField() = default;
  OccupancyDistanceField(
    std::size_t width, std::size_t height, double resolution,
    double origin_x, double origin_y, double origin_yaw,
    const std::vector<std::int8_t> & occupancy, std::int8_t occupied_threshold = 65);

  bool valid() const;
  bool contains(double world_x, double world_y) const;
  double distance(double world_x, double world_y) const;
  double resolution() const {return resolution_;}

private:
  std::size_t width_{0};
  std::size_t height_{0};
  double resolution_{0.0};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double origin_yaw_{0.0};
  double cos_origin_{1.0};
  double sin_origin_{0.0};
  double outside_distance_{10.0};
  std::vector<double> distances_;

  Point2 worldToGrid(double world_x, double world_y) const;
};

SearchConfig defaultSearchConfig();
double normalizeAngle(double angle);
double scanMatchScore(
  const OccupancyDistanceField & field, const std::vector<Point2> & points_base,
  const Pose2 & pose, const Pose2 & prior, const SearchConfig & config);
RefinementResult refinePose(
  const OccupancyDistanceField & field, const std::vector<Point2> & points_base,
  const Pose2 & prior, const SearchConfig & config = defaultSearchConfig());

}  // namespace sanitation_scan_refiner

#endif  // SANITATION_SCAN_REFINER__SCAN_REFINER_CORE_HPP_
