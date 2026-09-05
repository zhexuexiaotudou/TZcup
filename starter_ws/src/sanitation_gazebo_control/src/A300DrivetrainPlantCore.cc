#include "sanitation_gazebo_control/A300DrivetrainPlantCore.hh"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sanitation_gazebo_control
{
namespace
{

double ClampMagnitude(const double value, const double magnitude)
{
  return std::clamp(value, -magnitude, magnitude);
}

double MoveToward(const double current, const double target, const double maximum_delta)
{
  return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

bool FiniteArray(const std::array<double, kA300WheelCount> & values)
{
  return std::all_of(values.begin(), values.end(), [](const double value) {
    return std::isfinite(value);
  });
}

double MechanicalPower(
  const std::array<double, kA300WheelCount> & torque_nm,
  const std::array<double, kA300WheelCount> & speed_rad_s)
{
  double power_w = 0.0;
  for (std::size_t index = 0; index < kA300WheelCount; ++index) {
    power_w += std::abs(torque_nm[index] * speed_rad_s[index]);
  }
  return power_w;
}

}  // namespace

A300DrivetrainPlantCore::A300DrivetrainPlantCore(
  A300DrivetrainPlantParameters parameters)
: parameters_(parameters)
{
  const std::array<double, 17> finite_parameters{
    parameters_.physical_wheel_radius_m,
    parameters_.control_wheel_radius_m,
    parameters_.maximum_vehicle_speed_mps,
    parameters_.nominal_bus_voltage_v,
    parameters_.continuous_current_per_motor_a,
    parameters_.continuous_battery_current_a,
    parameters_.total_motor_output_power_w,
    parameters_.command_timeout_s,
    parameters_.wheel_side_torque_constant_nm_per_a,
    parameters_.low_speed_torque_limit_nm,
    parameters_.speed_error_gain_nm_per_rad_s,
    parameters_.torque_slew_rate_nm_per_s,
    parameters_.service_brake_torque_limit_nm,
    parameters_.brake_response_delay_s,
    parameters_.brake_ramp_time_s,
    parameters_.drivetrain_efficiency,
    parameters_.stopped_speed_rad_s};
  if (!std::all_of(
      finite_parameters.begin(), finite_parameters.end(),
      [](const double value) {return std::isfinite(value);}) ||
    !(parameters_.physical_wheel_radius_m > 0.0) ||
    !(parameters_.control_wheel_radius_m > 0.0) ||
    !(parameters_.maximum_vehicle_speed_mps > 0.0) ||
    !(parameters_.nominal_bus_voltage_v > 0.0) ||
    !(parameters_.continuous_current_per_motor_a > 0.0) ||
    !(parameters_.continuous_battery_current_a > 0.0) ||
    !(parameters_.total_motor_output_power_w > 0.0) ||
    !(parameters_.command_timeout_s > 0.0) ||
    !(parameters_.wheel_side_torque_constant_nm_per_a > 0.0) ||
    !(parameters_.low_speed_torque_limit_nm > 0.0) ||
    !(parameters_.speed_error_gain_nm_per_rad_s > 0.0) ||
    !(parameters_.torque_slew_rate_nm_per_s > 0.0) ||
    !(parameters_.service_brake_torque_limit_nm > 0.0) ||
    parameters_.brake_response_delay_s < 0.0 ||
    !(parameters_.brake_ramp_time_s > 0.0) ||
    !(parameters_.drivetrain_efficiency > 0.0) ||
    parameters_.drivetrain_efficiency > 1.0 ||
    parameters_.stopped_speed_rad_s < 0.0)
  {
    throw std::invalid_argument("invalid A300 drivetrain plant parameters");
  }
}

const A300DrivetrainPlantParameters & A300DrivetrainPlantCore::Parameters() const
{
  return parameters_;
}

void A300DrivetrainPlantCore::Reset()
{
  applied_torque_nm_.fill(0.0);
  stopped_elapsed_s_ = 0.0;
}

A300DrivetrainPlantOutput A300DrivetrainPlantCore::Step(
  const A300DrivetrainPlantInput & input)
{
  A300DrivetrainPlantOutput output;
  const bool valid = std::isfinite(input.step_s) && input.step_s > 0.0 &&
    input.step_s <= 0.25 && std::isfinite(input.command_age_s) &&
    input.command_age_s >= 0.0 && std::isfinite(input.bus_voltage_v) &&
    input.bus_voltage_v > 0.0 && FiniteArray(input.commanded_speed_rad_s) &&
    FiniteArray(input.measured_speed_rad_s);
  const bool any_fault = std::any_of(
    input.motor_fault.begin(), input.motor_fault.end(), [](const bool value) {return value;});

  if (!valid) {
    output.stop_reason = A300DrivetrainStopReason::kInvalidInput;
  } else if (input.emergency_stop) {
    output.stop_reason = A300DrivetrainStopReason::kEmergencyStop;
  } else if (!input.actuator_enable) {
    output.stop_reason = A300DrivetrainStopReason::kActuatorDisabled;
  } else if (any_fault) {
    output.stop_reason = A300DrivetrainStopReason::kMotorFault;
  } else if (input.command_age_s > parameters_.command_timeout_s) {
    output.stop_reason = A300DrivetrainStopReason::kCommandTimeout;
  } else {
    output.stop_reason = A300DrivetrainStopReason::kNone;
    output.drive_permitted = true;
  }

  const double step_s = valid ? input.step_s : 0.01;
  std::array<double, kA300WheelCount> target_torque{};
  if (output.drive_permitted) {
    stopped_elapsed_s_ = 0.0;
    const double maximum_speed_rad_s =
      parameters_.maximum_vehicle_speed_mps / parameters_.control_wheel_radius_m;
    const double per_motor_power_w =
      parameters_.total_motor_output_power_w / static_cast<double>(kA300WheelCount);
    const double current_torque_limit_nm =
      parameters_.continuous_current_per_motor_a *
      parameters_.wheel_side_torque_constant_nm_per_a;
    for (std::size_t index = 0; index < kA300WheelCount; ++index) {
      output.limited_command_rad_s[index] = ClampMagnitude(
        input.commanded_speed_rad_s[index], maximum_speed_rad_s);
      const double speed_error =
        output.limited_command_rad_s[index] - input.measured_speed_rad_s[index];
      const double absolute_speed = std::max(
        std::abs(input.measured_speed_rad_s[index]), 0.25);
      const double power_torque_limit_nm = per_motor_power_w / absolute_speed;
      const double raw_torque_nm =
        parameters_.speed_error_gain_nm_per_rad_s * speed_error;
      const double torque_limit_nm = std::min(
        {parameters_.low_speed_torque_limit_nm, current_torque_limit_nm,
          power_torque_limit_nm});
      target_torque[index] = ClampMagnitude(raw_torque_nm, torque_limit_nm);
      output.current_limited = output.current_limited ||
        (std::abs(raw_torque_nm) > current_torque_limit_nm &&
        current_torque_limit_nm <= parameters_.low_speed_torque_limit_nm &&
        current_torque_limit_nm <= power_torque_limit_nm);
      output.power_limited = output.power_limited ||
        (std::abs(raw_torque_nm) > power_torque_limit_nm &&
        power_torque_limit_nm <= parameters_.low_speed_torque_limit_nm &&
        power_torque_limit_nm <= current_torque_limit_nm);
    }
  } else {
    stopped_elapsed_s_ += step_s;
    if (stopped_elapsed_s_ >= parameters_.brake_response_delay_s) {
      output.resistive_brake_active = true;
      const double ramp = std::min(
        1.0,
        (stopped_elapsed_s_ - parameters_.brake_response_delay_s) /
        parameters_.brake_ramp_time_s);
      for (std::size_t index = 0; index < kA300WheelCount; ++index) {
        const double speed = valid ? input.measured_speed_rad_s[index] : 0.0;
        if (std::abs(speed) > parameters_.stopped_speed_rad_s) {
          // A fixed Coulomb torque can cross zero within one physics step,
          // reverse the wheel, and then inject energy while the sign flips on
          // the next step.  The resulting chatter was sufficient to propel
          // the uncommanded 160 kg vehicle several metres in Gazebo.  Model
          // the controller's closed-loop regenerative/service braking as a
          // proportional zero-speed target, capped by the physical brake
          // envelope and its response ramp.
          const double proportional_brake =
            -parameters_.speed_error_gain_nm_per_rad_s * speed;
          target_torque[index] = ClampMagnitude(
            proportional_brake,
            parameters_.service_brake_torque_limit_nm * ramp);
        }
      }
    }
  }

  if (output.drive_permitted) {
    const double maximum_torque_delta = parameters_.torque_slew_rate_nm_per_s * step_s;
    for (std::size_t index = 0; index < kA300WheelCount; ++index) {
      applied_torque_nm_[index] = MoveToward(
        applied_torque_nm_[index], target_torque[index], maximum_torque_delta);
    }
  } else {
    // Removing propulsion is fail-safe and immediate. The separately modelled
    // resistive brake still observes its response delay and ramp.
    applied_torque_nm_ = target_torque;
  }

  // Enforce the published 1080 W aggregate motor-output boundary after the
  // per-wheel torque-speed and current limits.
  const std::array<double, kA300WheelCount> zero_speed{};
  const auto & measured_speed = valid ? input.measured_speed_rad_s : zero_speed;
  double mechanical_power_w = MechanicalPower(applied_torque_nm_, measured_speed);
  if (mechanical_power_w > parameters_.total_motor_output_power_w) {
    const double scale = parameters_.total_motor_output_power_w / mechanical_power_w;
    for (double & torque : applied_torque_nm_) {
      torque *= scale;
    }
    output.power_limited = true;
    mechanical_power_w = MechanicalPower(applied_torque_nm_, measured_speed);
  }

  double total_current_a = 0.0;
  for (const double torque : applied_torque_nm_) {
    total_current_a += std::abs(torque) /
      parameters_.wheel_side_torque_constant_nm_per_a;
  }
  const double effective_bus_voltage_v = valid ? input.bus_voltage_v :
    parameters_.nominal_bus_voltage_v;
  const double electrical_power_current_a = mechanical_power_w /
    (effective_bus_voltage_v * parameters_.drivetrain_efficiency);
  total_current_a = std::max(total_current_a, electrical_power_current_a);
  if (total_current_a > parameters_.continuous_battery_current_a) {
    const double scale = parameters_.continuous_battery_current_a / total_current_a;
    for (double & torque : applied_torque_nm_) {
      torque *= scale;
    }
    output.current_limited = true;
  }

  output.total_mechanical_power_w = 0.0;
  output.estimated_battery_current_a = 0.0;
  for (std::size_t index = 0; index < kA300WheelCount; ++index) {
    output.wheel_torque_nm[index] = applied_torque_nm_[index];
    output.estimated_motor_current_a[index] = std::min(
      parameters_.continuous_current_per_motor_a,
      std::abs(applied_torque_nm_[index]) /
      parameters_.wheel_side_torque_constant_nm_per_a);
    output.estimated_battery_current_a += output.estimated_motor_current_a[index];
    output.total_mechanical_power_w += std::abs(
      applied_torque_nm_[index] * measured_speed[index]);
  }
  const double final_electrical_power_current_a = output.total_mechanical_power_w /
    (effective_bus_voltage_v * parameters_.drivetrain_efficiency);
  output.estimated_battery_current_a = std::min(
    std::max(output.estimated_battery_current_a, final_electrical_power_current_a),
    parameters_.continuous_battery_current_a);
  return output;
}

}  // namespace sanitation_gazebo_control
