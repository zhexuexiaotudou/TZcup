// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "sanitation_gazebo_auxiliary/SqueegeeComplianceCore.hh"

namespace sanitation_gazebo_auxiliary
{
namespace
{

const ComplianceAxisParameters kFloat{1800.0, 45.0, -0.0066, 120.0};

TEST(SqueegeeComplianceCore, AppliesSpecifiedGroundPreloadAtZeroDatum)
{
  EXPECT_NEAR(SqueegeeComplianceCore::Effort(kFloat, 0.0, 0.0), -11.88, 1e-12);
}

TEST(SqueegeeComplianceCore, RelaxesAtFreeSpringReference)
{
  EXPECT_NEAR(SqueegeeComplianceCore::Effort(kFloat, -0.0066, 0.0), 0.0, 1e-12);
}

TEST(SqueegeeComplianceCore, DampingOpposesJointVelocity)
{
  EXPECT_NEAR(SqueegeeComplianceCore::Effort(kFloat, -0.0066, 0.1), -4.5, 1e-12);
  EXPECT_NEAR(SqueegeeComplianceCore::Effort(kFloat, -0.0066, -0.1), 4.5, 1e-12);
}

TEST(SqueegeeComplianceCore, ClampsForceAtBothPhysicalLimits)
{
  EXPECT_DOUBLE_EQ(SqueegeeComplianceCore::Effort(kFloat, 1.0, 0.0), -120.0);
  EXPECT_DOUBLE_EQ(SqueegeeComplianceCore::Effort(kFloat, -1.0, 0.0), 120.0);
}

TEST(SqueegeeComplianceCore, RejectsInvalidInputsFailClosed)
{
  auto invalid = kFloat;
  invalid.maximumEffort = 0.0;
  EXPECT_FALSE(SqueegeeComplianceCore::Valid(invalid));
  EXPECT_TRUE(std::isnan(SqueegeeComplianceCore::Effort(invalid, 0.0, 0.0)));
  EXPECT_TRUE(std::isnan(SqueegeeComplianceCore::Effort(
      kFloat, std::numeric_limits<double>::quiet_NaN(), 0.0)));
}

}  // namespace
}  // namespace sanitation_gazebo_auxiliary
