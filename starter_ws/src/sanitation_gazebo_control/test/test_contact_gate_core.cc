// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include "sanitation_gazebo_control/ContactGateCore.hh"

namespace sanitation_gazebo_control
{
TEST(ContactGateCore, RejectsNoContactAndOneSidedContact)
{
  EXPECT_FALSE(CommonContact({}, {}).has_value());
  EXPECT_FALSE(CommonContact({17}, {}).has_value());
  EXPECT_FALSE(CommonContact({}, {17}).has_value());
}

TEST(ContactGateCore, RejectsDifferentBodiesAcrossFingers)
{
  EXPECT_FALSE(CommonContact({17}, {23}).has_value());
  EXPECT_FALSE(CommonContact({17, 19}, {23, 29}).has_value());
}

TEST(ContactGateCore, AcceptsOnlyTheSameBodyAcrossBothFingers)
{
  const auto selected = CommonContact({17, 19}, {19, 23});
  ASSERT_TRUE(selected.has_value());
  EXPECT_EQ(19u, selected.value());
}

TEST(ContactGateCore, RejectsVehicleNamesAndStaticWorldBodies)
{
  EXPECT_FALSE(IsExternalCollisionName(
      "tzcup_formal_sanitation_vehicle::base_link::collision",
      "tzcup_formal_sanitation_vehicle"));
  EXPECT_FALSE(IsExternalCollisionName("", "tzcup_formal_sanitation_vehicle"));
  EXPECT_TRUE(IsExternalCollisionName(
      "object_runtime::cube_link::collision",
      "tzcup_formal_sanitation_vehicle"));
  EXPECT_TRUE(DynamicBodyEligible(true, false));
  EXPECT_FALSE(DynamicBodyEligible(true, true));
  EXPECT_FALSE(DynamicBodyEligible(false, false));
}
}  // namespace sanitation_gazebo_control
