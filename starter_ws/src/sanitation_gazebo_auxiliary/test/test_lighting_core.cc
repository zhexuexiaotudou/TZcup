// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "sanitation_gazebo_auxiliary/LightingCore.hh"

namespace sanitation_gazebo_auxiliary
{

TEST(LightingCore, RejectsInvalidFlashConfiguration)
{
  EXPECT_THROW(LightingCore({0.0, 0.5}), std::invalid_argument);
  EXPECT_THROW(LightingCore({1.0, -0.01}), std::invalid_argument);
  EXPECT_THROW(LightingCore({1.0, 1.01}), std::invalid_argument);
}

TEST(LightingCore, EmergencyStopCutsWorkAndForcesWarningDemand)
{
  const LightingCore core({2.0, 0.5});
  LightingInputs inputs;
  inputs.workRequested = true;
  inputs.safetyPowerAvailable = true;
  inputs.emergencyStopLatched = true;

  const auto onPhase = core.Evaluate(0.10, inputs);
  EXPECT_FALSE(onPhase.workOn);
  EXPECT_TRUE(onPhase.warningOn);

  const auto offPhase = core.Evaluate(0.30, inputs);
  EXPECT_FALSE(offPhase.workOn);
  EXPECT_FALSE(offPhase.warningOn);
}

TEST(LightingCore, WorkRequiresSafetyPowerButTailRemainsVisible)
{
  const LightingCore core;
  LightingInputs inputs;
  inputs.workRequested = true;
  inputs.tailRequested = true;
  inputs.emergencyStopLatched = false;
  inputs.safetyPowerAvailable = false;

  const auto outputs = core.Evaluate(0.0, inputs);
  EXPECT_FALSE(outputs.workOn);
  EXPECT_TRUE(outputs.tailOn);
  EXPECT_FALSE(outputs.warningOn);
}

TEST(LightingCore, FlashPhaseUsesSimulationTimeDeterministically)
{
  const LightingCore core({1.0, 0.25});
  LightingInputs inputs;
  inputs.warningRequested = true;
  inputs.emergencyStopLatched = false;
  inputs.safetyPowerAvailable = true;

  EXPECT_TRUE(core.Evaluate(2.10, inputs).warningOn);
  EXPECT_FALSE(core.Evaluate(2.30, inputs).warningOn);
  EXPECT_TRUE(core.Evaluate(3.10, inputs).warningOn);
  EXPECT_TRUE(core.Evaluate(
      std::numeric_limits<double>::quiet_NaN(), inputs).warningOn);
}

TEST(LightingCore, RepeatedSimulationTimeFreezesWarningPhase)
{
  const LightingCore core({1.0, 0.5});
  LightingInputs inputs;
  inputs.warningRequested = true;
  inputs.emergencyStopLatched = false;
  inputs.safetyPowerAvailable = true;

  const auto beforePause = core.Evaluate(4.75, inputs);
  const auto duringPause = core.Evaluate(4.75, inputs);
  EXPECT_EQ(beforePause, duringPause);
  EXPECT_FALSE(duringPause.warningOn);
}

}  // namespace sanitation_gazebo_auxiliary
