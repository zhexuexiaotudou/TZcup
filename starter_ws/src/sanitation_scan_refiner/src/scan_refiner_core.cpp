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

#include "sanitation_scan_refiner/scan_refiner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>

namespace sanitation_scan_refiner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double huberLoss(const double distance, const double delta)
{
  if (distance <= delta) {
    return 0.5 * distance * distance;
  }
  return delta * (distance - 0.5 * delta);
}

std::vector<Point2> evenlySample(
  const std::vector<Point2> & points, const std::size_t maximum_points)
{
  if (points.size() <= maximum_points) {
    return points;
  }
  std::vector<Point2> sampled;
  sampled.reserve(maximum_points);
  const double stride = static_cast<double>(points.size()) /
    static_cast<double>(maximum_points);
  for (std::size_t index = 0; index < maximum_points; ++index) {
    sampled.push_back(points[static_cast<std::size_t>(std::floor(index * stride))]);
  }
  return sampled;
}
}  // namespace

OccupancyDistanceField::OccupancyDistanceField(
  const std::size_t width, const std::size_t height, const double resolution,
  const double origin_x, const double origin_y, const double origin_yaw,
  const std::vector<std::int8_t> & occupancy, const std::int8_t occupied_threshold)
: width_(width), height_(height), resolution_(resolution),
  origin_x_(origin_x), origin_y_(origin_y), origin_yaw_(origin_yaw),
  cos_origin_(std::cos(origin_yaw)), sin_origin_(std::sin(origin_yaw))
{
  if (width_ == 0 || height_ == 0 || resolution_ <= 0.0 ||
    occupancy.size() != width_ * height_)
  {
    return;
  }

  const double infinity = std::numeric_limits<double>::infinity();
  distances_.assign(width_ * height_, infinity);
  using QueueEntry = std::pair<double, std::size_t>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> queue;
  for (std::size_t index = 0; index < occupancy.size(); ++index) {
    if (occupancy[index] >= occupied_threshold) {
      distances_[index] = 0.0;
      queue.emplace(0.0, index);
    }
  }
  if (queue.empty()) {
    distances_.clear();
    return;
  }

  const std::vector<std::tuple<int, int, double>> neighbors{
    {-1, 0, 1.0}, {1, 0, 1.0}, {0, -1, 1.0}, {0, 1, 1.0},
    {-1, -1, std::sqrt(2.0)}, {-1, 1, std::sqrt(2.0)},
    {1, -1, std::sqrt(2.0)}, {1, 1, std::sqrt(2.0)}};
  while (!queue.empty()) {
    const auto [current_distance, index] = queue.top();
    queue.pop();
    if (current_distance > distances_[index]) {
      continue;
    }
    const int x = static_cast<int>(index % width_);
    const int y = static_cast<int>(index / width_);
    for (const auto & [dx, dy, multiplier] : neighbors) {
      const int nx = x + dx;
      const int ny = y + dy;
      if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) ||
        ny >= static_cast<int>(height_))
      {
        continue;
      }
      const std::size_t neighbor_index =
        static_cast<std::size_t>(ny) * width_ + static_cast<std::size_t>(nx);
      const double candidate = current_distance + multiplier * resolution_;
      if (candidate < distances_[neighbor_index]) {
        distances_[neighbor_index] = candidate;
        queue.emplace(candidate, neighbor_index);
      }
    }
  }
  outside_distance_ = std::hypot(
    static_cast<double>(width_) * resolution_,
    static_cast<double>(height_) * resolution_);
}

bool OccupancyDistanceField::valid() const
{
  return width_ > 1 && height_ > 1 && resolution_ > 0.0 &&
         distances_.size() == width_ * height_;
}

Point2 OccupancyDistanceField::worldToGrid(
  const double world_x, const double world_y) const
{
  const double dx = world_x - origin_x_;
  const double dy = world_y - origin_y_;
  return {
    (cos_origin_ * dx + sin_origin_ * dy) / resolution_,
    (-sin_origin_ * dx + cos_origin_ * dy) / resolution_};
}

bool OccupancyDistanceField::contains(const double world_x, const double world_y) const
{
  if (!valid()) {
    return false;
  }
  const Point2 grid = worldToGrid(world_x, world_y);
  return grid.x >= 0.0 && grid.y >= 0.0 &&
         grid.x <= static_cast<double>(width_ - 1) &&
         grid.y <= static_cast<double>(height_ - 1);
}

double OccupancyDistanceField::distance(const double world_x, const double world_y) const
{
  if (!contains(world_x, world_y)) {
    return outside_distance_;
  }
  const Point2 grid = worldToGrid(world_x, world_y);
  const std::size_t x0 = static_cast<std::size_t>(std::floor(grid.x));
  const std::size_t y0 = static_cast<std::size_t>(std::floor(grid.y));
  const std::size_t x1 = std::min(x0 + 1, width_ - 1);
  const std::size_t y1 = std::min(y0 + 1, height_ - 1);
  const double tx = grid.x - static_cast<double>(x0);
  const double ty = grid.y - static_cast<double>(y0);
  const double d00 = distances_[y0 * width_ + x0];
  const double d10 = distances_[y0 * width_ + x1];
  const double d01 = distances_[y1 * width_ + x0];
  const double d11 = distances_[y1 * width_ + x1];
  return (1.0 - ty) * ((1.0 - tx) * d00 + tx * d10) +
         ty * ((1.0 - tx) * d01 + tx * d11);
}

SearchConfig defaultSearchConfig()
{
  SearchConfig config;
  config.stages = {
    {0.20, 3.0 * kPi / 180.0, 0.05, 1.0 * kPi / 180.0},
    {0.06, 0.75 * kPi / 180.0, 0.015, 0.25 * kPi / 180.0},
    {0.015, 0.15 * kPi / 180.0, 0.005, 0.05 * kPi / 180.0}};
  return config;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double scanMatchScore(
  const OccupancyDistanceField & field, const std::vector<Point2> & points_base,
  const Pose2 & pose, const Pose2 & prior, const SearchConfig & config)
{
  if (!field.valid() || points_base.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  const double cosine = std::cos(pose.yaw);
  const double sine = std::sin(pose.yaw);
  double loss_sum = 0.0;
  for (const Point2 & point : points_base) {
    const double world_x = pose.x + cosine * point.x - sine * point.y;
    const double world_y = pose.y + sine * point.x + cosine * point.y;
    loss_sum += huberLoss(field.distance(world_x, world_y), config.huber_delta_m);
  }
  const double robust_distance = std::sqrt(
    2.0 * loss_sum / static_cast<double>(points_base.size()));
  const double translation_delta = std::hypot(pose.x - prior.x, pose.y - prior.y);
  const double yaw_delta = std::abs(normalizeAngle(pose.yaw - prior.yaw));
  return robust_distance +
         config.prior_translation_weight * translation_delta +
         config.prior_yaw_weight * yaw_delta;
}

RefinementResult refinePose(
  const OccupancyDistanceField & field, const std::vector<Point2> & raw_points_base,
  const Pose2 & prior, const SearchConfig & config)
{
  RefinementResult result;
  result.pose = prior;
  if (!field.valid()) {
    result.reason = "invalid_distance_field";
    return result;
  }
  if (!field.contains(prior.x, prior.y)) {
    result.reason = "prior_outside_map";
    return result;
  }
  if (raw_points_base.size() < config.minimum_points) {
    result.reason = "insufficient_valid_points";
    result.valid_point_count = raw_points_base.size();
    return result;
  }
  std::vector<Point2> in_map_points;
  in_map_points.reserve(raw_points_base.size());
  const double prior_cosine = std::cos(prior.yaw);
  const double prior_sine = std::sin(prior.yaw);
  for (const auto & point : raw_points_base) {
    const double world_x = prior.x + prior_cosine * point.x - prior_sine * point.y;
    const double world_y = prior.y + prior_sine * point.x + prior_cosine * point.y;
    if (field.contains(world_x, world_y)) {
      in_map_points.push_back(point);
    }
  }
  if (in_map_points.size() < config.minimum_points) {
    result.reason = "insufficient_in_map_points";
    result.valid_point_count = in_map_points.size();
    return result;
  }
  const std::vector<Point2> points = evenlySample(in_map_points, config.maximum_points);
  result.valid_point_count = points.size();
  result.prior_score_m = scanMatchScore(field, points, prior, prior, config);
  Pose2 best = prior;
  double best_score = result.prior_score_m;

  for (const SearchStage & stage : config.stages) {
    const Pose2 center = best;
    const int xy_count = static_cast<int>(std::ceil(stage.xy_window_m / stage.xy_step_m));
    const int yaw_count = static_cast<int>(
      std::ceil(stage.yaw_window_rad / stage.yaw_step_rad));
    for (int ix = -xy_count; ix <= xy_count; ++ix) {
      for (int iy = -xy_count; iy <= xy_count; ++iy) {
        for (int iyaw = -yaw_count; iyaw <= yaw_count; ++iyaw) {
          Pose2 candidate{
            center.x + static_cast<double>(ix) * stage.xy_step_m,
            center.y + static_cast<double>(iy) * stage.xy_step_m,
            normalizeAngle(center.yaw + static_cast<double>(iyaw) * stage.yaw_step_rad)};
          const double score = scanMatchScore(field, points, candidate, prior, config);
          ++result.evaluated_candidates;
          if (score < best_score) {
            best_score = score;
            best = candidate;
          }
        }
      }
    }
  }

  result.pose = best;
  result.score_m = best_score;
  result.improvement_m = result.prior_score_m - best_score;
  const SearchStage & fine = config.stages.back();
  SearchConfig measurement_config = config;
  measurement_config.prior_translation_weight = 0.0;
  measurement_config.prior_yaw_weight = 0.0;
  const double center_score = scanMatchScore(field, points, best, best, measurement_config);
  auto curvature = [&](const Pose2 & lower, const Pose2 & upper, const double step) {
      const double lower_score = scanMatchScore(
        field, points, lower, best, measurement_config);
      const double upper_score = scanMatchScore(
        field, points, upper, best, measurement_config);
      return std::max(0.0, (lower_score + upper_score - 2.0 * center_score) /
               (step * step));
    };
  result.curvature_x = curvature(
    {best.x - fine.xy_step_m, best.y, best.yaw},
    {best.x + fine.xy_step_m, best.y, best.yaw}, fine.xy_step_m);
  result.curvature_y = curvature(
    {best.x, best.y - fine.xy_step_m, best.yaw},
    {best.x, best.y + fine.xy_step_m, best.yaw}, fine.xy_step_m);
  result.curvature_yaw = curvature(
    {best.x, best.y, normalizeAngle(best.yaw - fine.yaw_step_rad)},
    {best.x, best.y, normalizeAngle(best.yaw + fine.yaw_step_rad)},
    fine.yaw_step_rad);
  result.observable =
    result.curvature_x >= config.minimum_translation_curvature &&
    result.curvature_y >= config.minimum_translation_curvature &&
    result.curvature_yaw >= config.minimum_yaw_curvature;
  const double information_scale = static_cast<double>(points.size());
  result.covariance_xx = 1.0 / std::max(1.0, result.curvature_x * information_scale);
  result.covariance_yy = 1.0 / std::max(1.0, result.curvature_y * information_scale);
  result.covariance_yaw_yaw = 1.0 /
    std::max(1.0, result.curvature_yaw * information_scale);

  if (result.score_m > config.maximum_score_m) {
    result.reason = "score_above_threshold";
  } else if (result.improvement_m < config.minimum_improvement_m) {
    result.reason = "insufficient_improvement";
  } else if (!result.observable) {
    result.reason = "locally_unobservable";
  } else {
    result.accepted = true;
    result.reason = "accepted";
  }
  return result;
}

}  // namespace sanitation_scan_refiner
