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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

#include "sanitation_scan_refiner/scan_refiner_core.hpp"
#include "gtest/gtest.h"

namespace sanitation_scan_refiner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

struct FixtureData
{
  OccupancyDistanceField field;
  std::vector<Point2> scan;
  Pose2 truth;
};

FixtureData makeAsymmetricRoom()
{
  constexpr std::size_t width = 240;
  constexpr std::size_t height = 200;
  constexpr double resolution = 0.05;
  std::vector<std::int8_t> occupancy(width * height, 0);
  auto occupy = [&](const int x, const int y) {
      if (x >= 0 && y >= 0 && x < static_cast<int>(width) && y < static_cast<int>(height)) {
        occupancy[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] = 100;
      }
    };
  for (int x = 10; x <= 225; ++x) {
    occupy(x, 10);
    occupy(x, 185);
  }
  for (int y = 10; y <= 185; ++y) {
    occupy(10, y);
    occupy(225, y);
  }
  for (int y = 25; y <= 130; ++y) {
    occupy(75, y);
  }
  for (int x = 120; x <= 205; ++x) {
    occupy(x, 145);
  }

  const Pose2 truth{5.2, 4.1, 12.0 * kPi / 180.0};
  std::vector<Point2> map_points;
  for (int y = 0; y < static_cast<int>(height); ++y) {
    for (int x = 0; x < static_cast<int>(width); ++x) {
      if (occupancy[static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)] >= 65 &&
        ((x + 3 * y) % 5 == 0))
      {
        map_points.push_back({x * resolution, y * resolution});
      }
    }
  }
  std::vector<Point2> scan;
  const double cosine = std::cos(truth.yaw);
  const double sine = std::sin(truth.yaw);
  for (const auto & map_point : map_points) {
    const double dx = map_point.x - truth.x;
    const double dy = map_point.y - truth.y;
    const Point2 base{cosine * dx + sine * dy, -sine * dx + cosine * dy};
    const double range = std::hypot(base.x, base.y);
    if (range > 0.5 && range < 5.5) {
      scan.push_back(base);
    }
  }
  return {
    OccupancyDistanceField(width, height, resolution, 0.0, 0.0, 0.0, occupancy),
    scan, truth};
}
}  // namespace

TEST(ScanRefinerCore, RecoversKnownOffsetCoarseToFine)
{
  const auto data = makeAsymmetricRoom();
  const Pose2 prior{data.truth.x + 0.14, data.truth.y - 0.11, data.truth.yaw + 2.2 * kPi / 180.0};
  const auto result = refinePose(data.field, data.scan, prior);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_LT(std::hypot(result.pose.x - data.truth.x, result.pose.y - data.truth.y), 0.025);
  EXPECT_LT(std::abs(normalizeAngle(result.pose.yaw - data.truth.yaw)), 0.3 * kPi / 180.0);
  EXPECT_TRUE(result.observable);
  EXPECT_GE(result.valid_point_count, 60u);
}

TEST(ScanRefinerCore, RemainsAccurateWithThirtyPercentDynamicPoints)
{
  auto data = makeAsymmetricRoom();
  std::mt19937 generator(41);
  std::uniform_real_distribution<double> dynamic_xy(-4.0, 4.0);
  const std::size_t dynamic_count = data.scan.size() * 3 / 7;
  for (std::size_t index = 0; index < dynamic_count; ++index) {
    data.scan.push_back({dynamic_xy(generator), dynamic_xy(generator)});
  }
  std::shuffle(data.scan.begin(), data.scan.end(), generator);
  SearchConfig config = defaultSearchConfig();
  config.maximum_score_m = 0.30;
  const Pose2 prior{data.truth.x - 0.10, data.truth.y + 0.08, data.truth.yaw - 1.5 * kPi / 180.0};
  const auto result = refinePose(data.field, data.scan, prior, config);
  ASSERT_TRUE(result.accepted) << result.reason << " score=" << result.score_m;
  EXPECT_LT(std::hypot(result.pose.x - data.truth.x, result.pose.y - data.truth.y), 0.04);
  EXPECT_LT(std::abs(normalizeAngle(result.pose.yaw - data.truth.yaw)), 0.5 * kPi / 180.0);
}

TEST(ScanRefinerCore, FailsClosedOnInsufficientPoints)
{
  const auto data = makeAsymmetricRoom();
  std::vector<Point2> sparse(data.scan.begin(), data.scan.begin() + 20);
  const auto result = refinePose(data.field, sparse, data.truth);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "insufficient_valid_points");
}

TEST(ScanRefinerCore, FailsClosedWhenPriorIsOutsideMap)
{
  const auto data = makeAsymmetricRoom();
  const auto result = refinePose(data.field, data.scan, {-1.0, -1.0, 0.0});
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "prior_outside_map");
}

TEST(ScanRefinerCore, RecoversWithHalfTheScanOccluded)
{
  auto data = makeAsymmetricRoom();
  std::vector<Point2> occluded;
  for (std::size_t index = 0; index < data.scan.size(); index += 2) {
    occluded.push_back(data.scan[index]);
  }
  const Pose2 prior{
    data.truth.x + 0.09, data.truth.y - 0.07,
    data.truth.yaw + 1.0 * kPi / 180.0};
  SearchConfig config = defaultSearchConfig();
  config.minimum_points = 30;
  const auto result = refinePose(data.field, occluded, prior, config);
  ASSERT_TRUE(result.accepted) << result.reason;
  EXPECT_LT(std::hypot(result.pose.x - data.truth.x, result.pose.y - data.truth.y), 0.04);
}

TEST(ScanRefinerCore, FailsClosedWhenObservabilityRequirementIsNotMet)
{
  const auto data = makeAsymmetricRoom();
  SearchConfig config = defaultSearchConfig();
  config.minimum_translation_curvature = 1e9;
  config.minimum_yaw_curvature = 1e9;
  const Pose2 prior{
    data.truth.x + 0.08, data.truth.y - 0.06,
    data.truth.yaw + 1.0 * kPi / 180.0};
  const auto result = refinePose(data.field, data.scan, prior, config);
  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "locally_unobservable");
}

TEST(ScanRefinerCore, DoesNotHidePriorOutsideSearchWindow)
{
  const auto data = makeAsymmetricRoom();
  const Pose2 prior{data.truth.x + 0.28, data.truth.y, data.truth.yaw};
  const auto result = refinePose(data.field, data.scan, prior);
  EXPECT_TRUE(
    !result.accepted ||
    std::hypot(result.pose.x - data.truth.x, result.pose.y - data.truth.y) > 0.05);
}

TEST(ScanRefinerCore, MeetsCpuBudgetForThreeHundredSixtyAndSevenHundredTwentyBeams)
{
  const auto data = makeAsymmetricRoom();
  for (const std::size_t beam_count : {360u, 720u}) {
    std::vector<Point2> beams;
    beams.reserve(beam_count);
    for (std::size_t index = 0; index < beam_count; ++index) {
      beams.push_back(data.scan[index % data.scan.size()]);
    }
    const Pose2 prior{
      data.truth.x + 0.08, data.truth.y - 0.06,
      data.truth.yaw + 1.0 * kPi / 180.0};
    const auto started = std::chrono::steady_clock::now();
    const auto result = refinePose(data.field, beams, prior);
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    ASSERT_TRUE(result.accepted) << result.reason;
    EXPECT_LT(elapsed_ms, 500.0);
  }
}
}  // namespace sanitation_scan_refiner
