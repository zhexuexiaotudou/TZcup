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

#include <gtest/gtest.h>

#include "sanitation_scan_refiner/fusion_filter.hpp"

TEST(FusionFilter, SmoothsGlobalAnchorWithoutPositionLag)
{
  const auto filtered = sanitation_scan_refiner::smoothAnchor(
    {10.0, -2.0}, {10.2, -1.8}, 0.1);
  EXPECT_NEAR(filtered.first, 10.02, 1e-12);
  EXPECT_NEAR(filtered.second, -1.98, 1e-12);
}

TEST(FusionFilter, BoundsAlphaAndRejectsInconsistentAbsoluteFixes)
{
  const auto filtered = sanitation_scan_refiner::smoothAnchor(
    {1.0, 2.0}, {3.0, 4.0}, 2.0);
  EXPECT_DOUBLE_EQ(filtered.first, 3.0);
  EXPECT_DOUBLE_EQ(filtered.second, 4.0);
  EXPECT_TRUE(sanitation_scan_refiner::measurementsConsistent(
      {0.0, 0.0}, {0.06, 0.06}, 0.10));
  EXPECT_FALSE(sanitation_scan_refiner::measurementsConsistent(
      {0.0, 0.0}, {0.08, 0.08}, 0.10));
}

TEST(FusionFilter, PropagatesDelayedHeadingWithLocalYawAndWraps)
{
  const double propagated = sanitation_scan_refiner::propagateHeading(
    3.13, -3.10, 3.08);
  EXPECT_NEAR(propagated, -3.05, 1e-12);
}
