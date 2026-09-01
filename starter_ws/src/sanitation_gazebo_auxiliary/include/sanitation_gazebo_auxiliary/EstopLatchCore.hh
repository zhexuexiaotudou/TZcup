// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#ifndef SANITATION_GAZEBO_AUXILIARY__ESTOP_LATCH_CORE_HH_
#define SANITATION_GAZEBO_AUXILIARY__ESTOP_LATCH_CORE_HH_

namespace sanitation_gazebo_auxiliary
{

/// Pure emergency-stop latch matching a normally-closed safety relay.
///
/// Any asserted emergency input latches immediately. Releasing the physical
/// button does not clear the latch: a separate reset edge is accepted only
/// when the emergency input is released and the external safety chain allows
/// reset. The default startup state is fail-closed.
class EstopLatchCore
{
  public: explicit EstopLatchCore(bool _initiallyLatched = true);

  public: bool Update(
      bool _emergencyInputAsserted,
      bool _resetRequested,
      bool _resetAllowed);

  public: bool Latched() const;

  private: bool latched{true};
};

}  // namespace sanitation_gazebo_auxiliary

#endif  // SANITATION_GAZEBO_AUXILIARY__ESTOP_LATCH_CORE_HH_
