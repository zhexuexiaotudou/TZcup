#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

namespace sanitation_gazebo_control
{
namespace
{
template<typename RosMessageT, typename GazeboMessageT>
struct GazeboToRosEndpoint
{
  const char * topic;
};

constexpr char kRoot[] = "/model/tzcup_formal_sanitation_vehicle/water_recovery";
constexpr char kResetGroundTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/reset_ground_volume_l";
constexpr char kResetTankTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/reset_tank_mass_kg";
constexpr char kCommandFilterBlockageTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/filter_blockage_fraction";
constexpr char kGroundVolumeTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/ground_volume_l";
constexpr char kMassBalanceTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/mass_balance_error_fraction";
// The component register consumes this typed declaration, so the formal
// evaluator's sole GZ->ROS writer cannot drift from the compiled endpoint.
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double>
  kFilterBlockageTelemetryEndpoint{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_blockage_fraction"
  };
constexpr char kStatusTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/status_json";
}  // namespace

class WaterEvaluationBridge final : public rclcpp::Node
{
public:
  WaterEvaluationBridge()
  : Node("water_evaluation_bridge")
  {
    reset_ground_gz_pub_ = gz_node_.Advertise<gz::msgs::Double>(kResetGroundTopic);
    reset_tank_gz_pub_ = gz_node_.Advertise<gz::msgs::Double>(kResetTankTopic);
    command_filter_blockage_gz_pub_ =
      gz_node_.Advertise<gz::msgs::Double>(kCommandFilterBlockageTopic);

    reset_ground_ros_sub_ = create_subscription<std_msgs::msg::Float64>(
      kResetGroundTopic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        PublishGazeboDouble(*message, reset_ground_gz_pub_);
      });
    reset_tank_ros_sub_ = create_subscription<std_msgs::msg::Float64>(
      kResetTankTopic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        PublishGazeboDouble(*message, reset_tank_gz_pub_);
      });
    command_filter_blockage_ros_sub_ = create_subscription<std_msgs::msg::Float64>(
      kCommandFilterBlockageTopic, 10,
      [this](const std_msgs::msg::Float64::SharedPtr message) {
        PublishGazeboDouble(*message, command_filter_blockage_gz_pub_);
      });

    ground_volume_ros_pub_ = create_publisher<std_msgs::msg::Float64>(kGroundVolumeTopic, 10);
    mass_balance_ros_pub_ = create_publisher<std_msgs::msg::Float64>(kMassBalanceTopic, 10);
    filter_blockage_ros_pub_ = create_publisher<std_msgs::msg::Float64>(
      kFilterBlockageTelemetryEndpoint.topic, 10);
    status_ros_pub_ = create_publisher<std_msgs::msg::String>(kStatusTopic, 10);

    if (!gz_node_.Subscribe(kGroundVolumeTopic, &WaterEvaluationBridge::OnGroundVolume, this) ||
      !gz_node_.Subscribe(kMassBalanceTopic, &WaterEvaluationBridge::OnMassBalance, this) ||
      !gz_node_.Subscribe(
        kFilterBlockageTelemetryEndpoint.topic, &WaterEvaluationBridge::OnFilterBlockage, this) ||
      !gz_node_.Subscribe(kStatusTopic, &WaterEvaluationBridge::OnStatus, this))
    {
      Stop();
      throw std::runtime_error("failed to subscribe to formal water evaluator telemetry");
    }
  }

  ~WaterEvaluationBridge() override
  {
    Stop();
  }

  void Stop()
  {
    if (stopping_.exchange(true)) {
      return;
    }
    gz_node_.Unsubscribe(kGroundVolumeTopic);
    gz_node_.Unsubscribe(kMassBalanceTopic);
    gz_node_.Unsubscribe(kFilterBlockageTelemetryEndpoint.topic);
    gz_node_.Unsubscribe(kStatusTopic);
    const std::lock_guard<std::mutex> drain(callback_mutex_);
  }

private:
  void PublishGazeboDouble(
    const std_msgs::msg::Float64 & source,
    gz::transport::Node::Publisher & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    gz::msgs::Double target;
    target.set_data(source.data);
    publisher.Publish(target);
  }

  void PublishRosDouble(
    const gz::msgs::Double & source,
    const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    std_msgs::msg::Float64 target;
    target.data = source.data();
    publisher->publish(target);
  }

  void OnGroundVolume(const gz::msgs::Double & message)
  {
    PublishRosDouble(message, ground_volume_ros_pub_);
  }

  void OnMassBalance(const gz::msgs::Double & message)
  {
    PublishRosDouble(message, mass_balance_ros_pub_);
  }

  void OnFilterBlockage(const gz::msgs::Double & message)
  {
    PublishRosDouble(message, filter_blockage_ros_pub_);
  }

  void OnStatus(const gz::msgs::StringMsg & source)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    std_msgs::msg::String target;
    target.data = source.data();
    status_ros_pub_->publish(target);
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher reset_ground_gz_pub_;
  gz::transport::Node::Publisher reset_tank_gz_pub_;
  gz::transport::Node::Publisher command_filter_blockage_gz_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reset_ground_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr reset_tank_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_filter_blockage_ros_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ground_volume_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mass_balance_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr filter_blockage_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_ros_pub_;
  std::mutex callback_mutex_;
  std::atomic<bool> stopping_{false};
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::WaterEvaluationBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::WaterEvaluationBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("water_evaluation_bridge"), "%s", error.what());
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
