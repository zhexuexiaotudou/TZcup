// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <gtest/gtest.h>

#include "sanitation_gazebo_auxiliary/EstopLatchCore.hh"

namespace sanitation_gazebo_auxiliary
{

TEST(EstopLatchCore, StartsFailClosedByDefault)
{
  const EstopLatchCore latch;
  EXPECT_TRUE(latch.Latched());
}

TEST(EstopLatchCore, ReleaseAloneNeverClearsLatch)
{
  EstopLatchCore latch(false);
  EXPECT_TRUE(latch.Update(true, false, true));
  EXPECT_TRUE(latch.Update(false, false, true));
}

TEST(EstopLatchCore, ResetRequiresReleasedInputAndSafetyPermission)
{
  EstopLatchCore latch;
  EXPECT_TRUE(latch.Update(false, true, false));
  EXPECT_TRUE(latch.Update(true, true, true));
  EXPECT_FALSE(latch.Update(false, true, true));
}

TEST(EstopLatchCore, NewEmergencyAlwaysRelatchesAfterReset)
{
  EstopLatchCore latch;
  EXPECT_FALSE(latch.Update(false, true, true));
  EXPECT_TRUE(latch.Update(true, false, true));
}

}  // namespace sanitation_gazebo_auxiliary
