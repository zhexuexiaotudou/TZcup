#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace sanitation_gazebo_control
{

constexpr std::size_t kA300WheelCount = 4;

enum class A300DrivetrainStopReason : std::uint8_t
{
  kNone = 0,
  kActuatorDisabled,
  kEmergencyStop,
  kCommandTimeout,
  kMotorFault,
  kInvalidInput,
};

struct A300DrivetrainPlantParameters
{
  // Published A300 40 Ah boundaries. See the companion contract for exact
  // provenance and the distinction between physical and odometry radii.
  double physical_wheel_radius_m{0.1651};
  double control_wheel_radius_m{0.1625};
  double maximum_vehicle_speed_mps{2.0};
  double nominal_bus_voltage_v{25.6};
  double continuous_current_per_motor_a{17.0};
  double continuous_battery_current_a{60.0};
  double total_motor_output_power_w{1080.0};
  double command_timeout_s{0.5};

  // Engineering calibration parameters. Clearpath does not publish the
  // wheel-side torque constant, low-speed torque cap, response delay or
  // torque slew rate. These values must never be reported as official data.
  double wheel_side_torque_constant_nm_per_a{3.5};
  double low_speed_torque_limit_nm{59.5};
  double speed_error_gain_nm_per_rad_s{12.0};
  double torque_slew_rate_nm_per_s{400.0};
  double service_brake_torque_limit_nm{32.0};
  double brake_response_delay_s{0.08};
  double brake_ramp_time_s{0.12};
  double drivetrain_efficiency{0.80};
  double stopped_speed_rad_s{0.05};
};

struct A300DrivetrainPlantInput
{
  double step_s{0.0};
  double command_age_s{0.0};
  double bus_voltage_v{25.6};
  bool actuator_enable{false};
  bool emergency_stop{false};
  std::array<bool, kA300WheelCount> motor_fault{};
  std::array<double, kA300WheelCount> commanded_speed_rad_s{};
  std::array<double, kA300WheelCount> measured_speed_rad_s{};
};

struct A300DrivetrainPlantOutput
{
  A300DrivetrainStopReason stop_reason{A300DrivetrainStopReason::kActuatorDisabled};
  bool drive_permitted{false};
  bool resistive_brake_active{false};
  bool current_limited{false};
  bool power_limited{false};
  std::array<double, kA300WheelCount> limited_command_rad_s{};
  std::array<double, kA300WheelCount> wheel_torque_nm{};
  std::array<double, kA300WheelCount> estimated_motor_current_a{};
  double total_mechanical_power_w{0.0};
  double estimated_battery_current_a{0.0};
};

class A300DrivetrainPlantCore
{
public:
  explicit A300DrivetrainPlantCore(
    A300DrivetrainPlantParameters parameters = A300DrivetrainPlantParameters{});

  const A300DrivetrainPlantParameters & Parameters() const;
  A300DrivetrainPlantOutput Step(const A300DrivetrainPlantInput & input);
  void Reset();

private:
  A300DrivetrainPlantParameters parameters_;
  std::array<double, kA300WheelCount> applied_torque_nm_{};
  double stopped_elapsed_s_{0.0};
};

}  // namespace sanitation_gazebo_control
