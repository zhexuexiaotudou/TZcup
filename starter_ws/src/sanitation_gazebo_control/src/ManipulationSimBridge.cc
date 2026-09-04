#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

namespace sanitation_gazebo_control
{
namespace
{
constexpr char kAttachTopic[] = "/manipulation/grasp/attach";
constexpr char kDetachTopic[] = "/manipulation/grasp/detach";
constexpr char kStateTopic[] = "/manipulation/grasp/state";
constexpr char kDualContactTopic[] = "/manipulation/gripper/dual_contact";
constexpr char kDryBinStatusTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/observed_status_json";
}  // namespace

class ManipulationSimBridge final : public rclcpp::Node
{
public:
  ManipulationSimBridge()
  : Node("manipulation_sim_bridge")
  {
    attach_gz_pub_ = gz_node_.Advertise<gz::msgs::Empty>(kAttachTopic);
    detach_gz_pub_ = gz_node_.Advertise<gz::msgs::Empty>(kDetachTopic);
    attach_ros_sub_ = create_subscription<std_msgs::msg::Empty>(
      kAttachTopic, 10,
      [this](const std_msgs::msg::Empty::SharedPtr message) {
        PublishEmpty(*message, attach_gz_pub_);
      });
    detach_ros_sub_ = create_subscription<std_msgs::msg::Empty>(
      kDetachTopic, 10,
      [this](const std_msgs::msg::Empty::SharedPtr message) {
        PublishEmpty(*message, detach_gz_pub_);
      });
    state_ros_pub_ = create_publisher<std_msgs::msg::Bool>(kStateTopic, 10);
    dual_contact_ros_pub_ =
      create_publisher<std_msgs::msg::Bool>(kDualContactTopic, 10);
    dry_bin_status_ros_pub_ =
      create_publisher<std_msgs::msg::String>(kDryBinStatusTopic, 10);

    if (!gz_node_.Subscribe(kStateTopic, &ManipulationSimBridge::OnState, this) ||
      !gz_node_.Subscribe(
        kDualContactTopic, &ManipulationSimBridge::OnDualContact, this) ||
      !gz_node_.Subscribe(
        kDryBinStatusTopic, &ManipulationSimBridge::OnDryBinStatus, this))
    {
      Stop();
      throw std::runtime_error("failed to subscribe to formal manipulation Gazebo state");
    }
  }

  ~ManipulationSimBridge() override
  {
    Stop();
  }

  void Stop()
  {
    if (stopping_.exchange(true)) {
      return;
    }
    gz_node_.Unsubscribe(kStateTopic);
    gz_node_.Unsubscribe(kDualContactTopic);
    gz_node_.Unsubscribe(kDryBinStatusTopic);
    const std::lock_guard<std::mutex> drain(callback_mutex_);
  }

private:
  void PublishEmpty(
    const std_msgs::msg::Empty &,
    gz::transport::Node::Publisher & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!stopping_.load()) {
      publisher.Publish(gz::msgs::Empty());
    }
  }

  void OnState(const gz::msgs::Boolean & message)
  {
    PublishBoolean(message, state_ros_pub_);
  }

  void OnDualContact(const gz::msgs::Boolean & message)
  {
    PublishBoolean(message, dual_contact_ros_pub_);
  }

  void PublishBoolean(
    const gz::msgs::Boolean & source,
    const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    std_msgs::msg::Bool target;
    target.data = source.data();
    publisher->publish(target);
  }

  void OnDryBinStatus(const gz::msgs::StringMsg & source)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    std_msgs::msg::String target;
    target.data = source.data();
    dry_bin_status_ros_pub_->publish(target);
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher attach_gz_pub_;
  gz::transport::Node::Publisher detach_gz_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr attach_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr detach_ros_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dual_contact_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr dry_bin_status_ros_pub_;
  std::mutex callback_mutex_;
  std::atomic<bool> stopping_{false};
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::ManipulationSimBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::ManipulationSimBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("manipulation_sim_bridge"), "%s", error.what());
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
