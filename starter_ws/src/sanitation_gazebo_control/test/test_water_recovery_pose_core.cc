#include <gtest/gtest.h>

#include "sanitation_gazebo_control/WaterRecoveryPoseCore.hh"

namespace
{
using sanitation_gazebo_control::BasePoseSource;
using sanitation_gazebo_control::BasePoseSourceName;
using sanitation_gazebo_control::SelectBasePoseSource;

TEST(WaterRecoveryPoseCore, UsesDeterministicPriority)
{
  EXPECT_EQ(
      SelectBasePoseSource(true, true, true), BasePoseSource::kBaseLink);
  EXPECT_EQ(
      SelectBasePoseSource(false, true, true), BasePoseSource::kBaseFootprint);
  EXPECT_EQ(
      SelectBasePoseSource(false, false, true), BasePoseSource::kModelEntity);
}

TEST(WaterRecoveryPoseCore, MissingAllSourcesIsExplicitlyUnavailable)
{
  const auto source = SelectBasePoseSource(false, false, false);
  EXPECT_EQ(source, BasePoseSource::kUnavailable);
  EXPECT_STREQ(BasePoseSourceName(source), "unavailable");
}
}
