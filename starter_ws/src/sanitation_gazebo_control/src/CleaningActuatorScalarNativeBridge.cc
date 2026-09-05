#include <memory>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "sanitation_gazebo_control/NativeBridgeSupport.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr RosToGazeboEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kLiftCommand{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/lift_position"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kEnableCommand{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/enable"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kResetFaultsCommand{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/command/reset_faults"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kFaultActive{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/fault_active"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kTotalCurrent{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/total_current_a"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kTotalPower{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/total_power_w"};
}  // namespace

class CleaningActuatorScalarNativeBridge final : public NativeBridgeSupport
{
public:
  CleaningActuatorScalarNativeBridge()
  : NativeBridgeSupport("cleaning_actuator_scalar_native_bridge")
  {
    lift_gz_pub_ = gz_node_.Advertise<gz::msgs::Double>(kLiftCommand.topic);
    enable_gz_pub_ = gz_node_.Advertise<gz::msgs::Boolean>(kEnableCommand.topic);
    reset_faults_gz_pub_ = gz_node_.Advertise<gz::msgs::Boolean>(kResetFaultsCommand.topic);

    lift_ros_sub_ = create_subscription<std_msgs::msg::Float64>(
      kLiftCommand.topic, 10, [this](const std_msgs::msg::Float64::SharedPtr message) {
        PublishRosToGazebo<std_msgs::msg::Float64, gz::msgs::Double>(*message, lift_gz_pub_);
      });
    enable_ros_sub_ = create_subscription<std_msgs::msg::Bool>(
      kEnableCommand.topic, 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
        PublishRosToGazebo<std_msgs::msg::Bool, gz::msgs::Boolean>(*message, enable_gz_pub_);
      });
    reset_faults_ros_sub_ = create_subscription<std_msgs::msg::Bool>(
      kResetFaultsCommand.topic, 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
        PublishRosToGazebo<std_msgs::msg::Bool, gz::msgs::Boolean>(
          *message, reset_faults_gz_pub_);
      });

    fault_active_ros_pub_ = create_publisher<std_msgs::msg::Bool>(kFaultActive.topic, 10);
    total_current_ros_pub_ = create_publisher<std_msgs::msg::Float64>(kTotalCurrent.topic, 10);
    total_power_ros_pub_ = create_publisher<std_msgs::msg::Float64>(kTotalPower.topic, 10);

    bool subscribed = true;
    subscribed = Subscribe(kFaultActive.topic, &CleaningActuatorScalarNativeBridge::OnFaultActive, this) && subscribed;
    subscribed = Subscribe(kTotalCurrent.topic, &CleaningActuatorScalarNativeBridge::OnTotalCurrent, this) && subscribed;
    subscribed = Subscribe(kTotalPower.topic, &CleaningActuatorScalarNativeBridge::OnTotalPower, this) && subscribed;
    if (!subscribed) {
      StopAndThrow("failed to subscribe to cleaning actuator scalar telemetry");
    }
  }

private:
  void OnFaultActive(const gz::msgs::Boolean & message)
  {
    PublishGazeboToRos<std_msgs::msg::Bool, gz::msgs::Boolean>(message, fault_active_ros_pub_);
  }

  void OnTotalCurrent(const gz::msgs::Double & message)
  {
    PublishGazeboToRos<std_msgs::msg::Float64, gz::msgs::Double>(message, total_current_ros_pub_);
  }

  void OnTotalPower(const gz::msgs::Double & message)
  {
    PublishGazeboToRos<std_msgs::msg::Float64, gz::msgs::Double>(message, total_power_ros_pub_);
  }

  gz::transport::Node::Publisher lift_gz_pub_;
  gz::transport::Node::Publisher enable_gz_pub_;
  gz::transport::Node::Publisher reset_faults_gz_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr lift_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_faults_ros_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_active_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_current_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr total_power_ros_pub_;
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::CleaningActuatorScalarNativeBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::CleaningActuatorScalarNativeBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cleaning_actuator_scalar_native_bridge"), "%s", error.what());
    if (bridge) {
      bridge->Stop();
      bridge.reset();
    }
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
