// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace sanitation_gazebo_control
{

/// Typed, authority-preserving boundary between the unique safety manager's
/// TwistStamped output and ros_gz_bridge's geometry_msgs/Twist transport.
/// This node never creates a command independently: stale, disabled or invalid
/// input produces zero Twist and a false enable at 50 Hz.
class A300DrivetrainCommandAdapter final : public rclcpp::Node
{
public:
  A300DrivetrainCommandAdapter()
  : Node("a300_drivetrain_command_adapter")
  {
    this->declare_parameter<std::string>(
      "safe_command_input_topic", "/base_controller/cmd_vel");
    this->declare_parameter<std::string>(
      "safety_enable_input_topic", "/safety/actuators_enabled");
    this->declare_parameter<std::string>(
      "gazebo_command_output_topic",
      "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/cmd_vel");
    this->declare_parameter<std::string>(
      "gazebo_enable_output_topic",
      "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/actuator_enable");
    this->declare_parameter<double>("input_timeout_s", 0.15);
    this->declare_parameter<double>("publish_period_s", 0.02);

    this->commandPublisher = this->create_publisher<geometry_msgs::msg::Twist>(
      this->get_parameter("gazebo_command_output_topic").as_string(), 10);
    this->enablePublisher = this->create_publisher<std_msgs::msg::Bool>(
      this->get_parameter("gazebo_enable_output_topic").as_string(), 10);
    this->commandSubscription =
      this->create_subscription<geometry_msgs::msg::TwistStamped>(
      this->get_parameter("safe_command_input_topic").as_string(), 10,
      [this](const geometry_msgs::msg::TwistStamped & message) {
        this->latestCommand = message.twist;
        this->commandValid = Finite(message.twist);
        this->lastCommand = std::chrono::steady_clock::now();
        this->commandSeen = true;
      });
    this->enableSubscription = this->create_subscription<std_msgs::msg::Bool>(
      this->get_parameter("safety_enable_input_topic").as_string(), 10,
      [this](const std_msgs::msg::Bool & message) {
        this->safetyEnable = message.data;
        this->lastEnable = std::chrono::steady_clock::now();
        this->enableSeen = true;
      });
    this->timer = this->create_wall_timer(
      std::chrono::duration<double>(this->get_parameter("publish_period_s").as_double()),
      [this]() {this->Publish();});
  }

private:
  static bool Finite(const geometry_msgs::msg::Twist & command)
  {
    return std::isfinite(command.linear.x) && std::isfinite(command.linear.y) &&
      std::isfinite(command.linear.z) && std::isfinite(command.angular.x) &&
      std::isfinite(command.angular.y) && std::isfinite(command.angular.z);
  }

  void Publish()
  {
    const auto now = std::chrono::steady_clock::now();
    const double timeoutS = this->get_parameter("input_timeout_s").as_double();
    const bool commandFresh = this->commandSeen &&
      std::chrono::duration<double>(now - this->lastCommand).count() <= timeoutS;
    const bool enableFresh = this->enableSeen &&
      std::chrono::duration<double>(now - this->lastEnable).count() <= timeoutS;
    const bool permitted = commandFresh && enableFresh &&
      this->commandValid && this->safetyEnable;

    geometry_msgs::msg::Twist command;
    if (permitted) {
      command = this->latestCommand;
    }
    this->commandPublisher->publish(command);
    std_msgs::msg::Bool enable;
    enable.data = permitted;
    this->enablePublisher->publish(enable);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr commandPublisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enablePublisher;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr commandSubscription;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enableSubscription;
  rclcpp::TimerBase::SharedPtr timer;
  geometry_msgs::msg::Twist latestCommand;
  std::chrono::steady_clock::time_point lastCommand{};
  std::chrono::steady_clock::time_point lastEnable{};
  bool commandSeen{false};
  bool enableSeen{false};
  bool commandValid{false};
  bool safetyEnable{false};
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<sanitation_gazebo_control::A300DrivetrainCommandAdapter>());
  rclcpp::shutdown();
  return 0;
}
