#include <array>
#include <memory>

#include <gz/msgs/boolean.pb.h>
#include <std_msgs/msg/bool.hpp>

#include "sanitation_gazebo_control/NativeBridgeSupport.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kWorkLightsCommand{
  "/formal_vehicle/lighting/work_lights_on"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kTailLightsCommand{
  "/formal_vehicle/lighting/tail_lights_on"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kWarningLightsCommand{
  "/formal_vehicle/lighting/warning_lights_on"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kEmergencyStopCommand{
  "/formal_vehicle/simulation/command/emergency_stop"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kEmergencyStopPlungerCommand{
  "/formal_vehicle/simulation/command/emergency_stop_plunger_pressed"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kEmergencyStopResetCommand{
  "/formal_vehicle/simulation/command/emergency_stop_reset"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kSafetyBranchCommand{
  "/formal_vehicle/power/branches/safety/enabled"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kMainPowerCommand{
  "/formal_vehicle/simulation/command/main_power"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kMainContactorCommand{
  "/formal_vehicle/power/main_contactor_command"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kEmergencyStop{
  "/emergency_stop"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kMainIsolatorClosed{
  "/formal_vehicle/power/main_isolator_closed"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kMainContactorClosed{
  "/formal_vehicle/power/main_contactor_closed"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kWorkLightsApplied{
  "/formal_vehicle/lighting/work_lights_applied"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kTailLightsApplied{
  "/formal_vehicle/lighting/tail_lights_applied"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kWarningLightsApplied{
  "/formal_vehicle/lighting/warning_lights_applied"};

constexpr std::array<const char *, 9> kRosToGazeboTopics{{
  kWorkLightsCommand.topic, kTailLightsCommand.topic, kWarningLightsCommand.topic,
  kEmergencyStopCommand.topic, kEmergencyStopPlungerCommand.topic, kEmergencyStopResetCommand.topic,
  kSafetyBranchCommand.topic, kMainPowerCommand.topic, kMainContactorCommand.topic,
}};
constexpr std::array<const char *, 6> kGazeboToRosTopics{{
  kEmergencyStop.topic, kMainIsolatorClosed.topic, kMainContactorClosed.topic,
  kWorkLightsApplied.topic, kTailLightsApplied.topic, kWarningLightsApplied.topic,
}};
}  // namespace

class FormalAuxiliaryNativeBridge final : public NativeBridgeSupport
{
public:
  FormalAuxiliaryNativeBridge()
  : NativeBridgeSupport("formal_auxiliary_native_bridge")
  {
    for (std::size_t index = 0; index < kRosToGazeboTopics.size(); ++index) {
      gazebo_publishers_[index] = gz_node_.Advertise<gz::msgs::Boolean>(
        kRosToGazeboTopics[index]);
      ros_subscriptions_[index] = create_subscription<std_msgs::msg::Bool>(
        kRosToGazeboTopics[index], 10,
        [this, index](const std_msgs::msg::Bool::SharedPtr message) {
          PublishRosToGazebo<std_msgs::msg::Bool, gz::msgs::Boolean>(
            *message, gazebo_publishers_[index]);
        });
    }
    for (std::size_t index = 0; index < kGazeboToRosTopics.size(); ++index) {
      ros_publishers_[index] = create_publisher<std_msgs::msg::Bool>(kGazeboToRosTopics[index], 10);
    }

    bool subscribed = true;
    subscribed = Subscribe(kGazeboToRosTopics[0], &FormalAuxiliaryNativeBridge::OnEmergencyStop, this) && subscribed;
    subscribed = Subscribe(kGazeboToRosTopics[1], &FormalAuxiliaryNativeBridge::OnMainIsolator, this) && subscribed;
    subscribed = Subscribe(kGazeboToRosTopics[2], &FormalAuxiliaryNativeBridge::OnMainContactor, this) && subscribed;
    subscribed = Subscribe(kGazeboToRosTopics[3], &FormalAuxiliaryNativeBridge::OnWorkLights, this) && subscribed;
    subscribed = Subscribe(kGazeboToRosTopics[4], &FormalAuxiliaryNativeBridge::OnTailLights, this) && subscribed;
    subscribed = Subscribe(kGazeboToRosTopics[5], &FormalAuxiliaryNativeBridge::OnWarningLights, this) && subscribed;
    if (!subscribed) {
      StopAndThrow("failed to subscribe to formal auxiliary telemetry");
    }
  }

private:
  void Publish(const gz::msgs::Boolean & message, std::size_t index)
  {
    PublishGazeboToRos<std_msgs::msg::Bool, gz::msgs::Boolean>(message, ros_publishers_[index]);
  }
  void OnEmergencyStop(const gz::msgs::Boolean & message) { Publish(message, 0); }
  void OnMainIsolator(const gz::msgs::Boolean & message) { Publish(message, 1); }
  void OnMainContactor(const gz::msgs::Boolean & message) { Publish(message, 2); }
  void OnWorkLights(const gz::msgs::Boolean & message) { Publish(message, 3); }
  void OnTailLights(const gz::msgs::Boolean & message) { Publish(message, 4); }
  void OnWarningLights(const gz::msgs::Boolean & message) { Publish(message, 5); }

  std::array<gz::transport::Node::Publisher, kRosToGazeboTopics.size()> gazebo_publishers_;
  std::array<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr, kRosToGazeboTopics.size()> ros_subscriptions_;
  std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, kGazeboToRosTopics.size()> ros_publishers_;
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::FormalAuxiliaryNativeBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::FormalAuxiliaryNativeBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formal_auxiliary_native_bridge"), "%s", error.what());
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
