#include <atomic>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/header.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/transport/Node.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
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

constexpr char kCommandVelocityTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/cmd_vel";
constexpr char kActuatorEnableTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/actuator_enable";
constexpr char kEmergencyStopTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/emergency_stop";
constexpr GazeboToRosEndpoint<nav_msgs::msg::Odometry, gz::msgs::Odometry>
  kOdometryEndpoint{
  "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/odom"
  };
constexpr char kStatusTopic[] =
  "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status";

std::string HeaderValue(const gz::msgs::Header & header, const std::string & key)
{
  for (const auto & item : header.data()) {
    if (item.key() == key && item.value_size() > 0) {
      return item.value(0);
    }
  }
  return {};
}
}  // namespace

class A300DrivetrainNativeBridge final : public rclcpp::Node
{
public:
  A300DrivetrainNativeBridge()
  : Node("a300_drivetrain_bridge")
  {
    command_velocity_gz_pub_ = gz_node_.Advertise<gz::msgs::Twist>(kCommandVelocityTopic);
    actuator_enable_gz_pub_ = gz_node_.Advertise<gz::msgs::Boolean>(kActuatorEnableTopic);
    emergency_stop_gz_pub_ = gz_node_.Advertise<gz::msgs::Boolean>(kEmergencyStopTopic);

    command_velocity_ros_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      kCommandVelocityTopic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        PublishGazeboTwist(*message);
      });
    actuator_enable_ros_sub_ = create_subscription<std_msgs::msg::Bool>(
      kActuatorEnableTopic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        PublishGazeboBool(*message, actuator_enable_gz_pub_);
      });
    emergency_stop_ros_sub_ = create_subscription<std_msgs::msg::Bool>(
      kEmergencyStopTopic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        PublishGazeboBool(*message, emergency_stop_gz_pub_);
      });

    odometry_ros_pub_ = create_publisher<nav_msgs::msg::Odometry>(kOdometryEndpoint.topic, 10);
    status_ros_pub_ = create_publisher<std_msgs::msg::String>(kStatusTopic, 10);

    if (!gz_node_.Subscribe(
        kOdometryEndpoint.topic, &A300DrivetrainNativeBridge::OnOdometry, this) ||
      !gz_node_.Subscribe(kStatusTopic, &A300DrivetrainNativeBridge::OnStatus, this))
    {
      Stop();
      throw std::runtime_error("failed to subscribe to A300 drivetrain telemetry");
    }
  }

  ~A300DrivetrainNativeBridge() override
  {
    Stop();
  }

  void Stop()
  {
    if (stopping_.exchange(true)) {
      return;
    }
    gz_node_.Unsubscribe(kOdometryEndpoint.topic);
    gz_node_.Unsubscribe(kStatusTopic);
    const std::lock_guard<std::mutex> drain(callback_mutex_);
  }

private:
  void PublishGazeboTwist(const geometry_msgs::msg::Twist & source)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    gz::msgs::Twist target;
    target.mutable_linear()->set_x(source.linear.x);
    target.mutable_linear()->set_y(source.linear.y);
    target.mutable_linear()->set_z(source.linear.z);
    target.mutable_angular()->set_x(source.angular.x);
    target.mutable_angular()->set_y(source.angular.y);
    target.mutable_angular()->set_z(source.angular.z);
    command_velocity_gz_pub_.Publish(target);
  }

  void PublishGazeboBool(
    const std_msgs::msg::Bool & source,
    gz::transport::Node::Publisher & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    gz::msgs::Boolean target;
    target.set_data(source.data);
    publisher.Publish(target);
  }

  void OnOdometry(const gz::msgs::Odometry & source)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    nav_msgs::msg::Odometry target;
    target.header.stamp.sec = source.header().stamp().sec();
    target.header.stamp.nanosec = source.header().stamp().nsec();
    target.header.frame_id = HeaderValue(source.header(), "frame_id");
    target.child_frame_id = HeaderValue(source.header(), "child_frame_id");
    target.pose.pose.position.x = source.pose().position().x();
    target.pose.pose.position.y = source.pose().position().y();
    target.pose.pose.position.z = source.pose().position().z();
    target.pose.pose.orientation.x = source.pose().orientation().x();
    target.pose.pose.orientation.y = source.pose().orientation().y();
    target.pose.pose.orientation.z = source.pose().orientation().z();
    target.pose.pose.orientation.w = source.pose().orientation().w();
    target.twist.twist.linear.x = source.twist().linear().x();
    target.twist.twist.linear.y = source.twist().linear().y();
    target.twist.twist.linear.z = source.twist().linear().z();
    target.twist.twist.angular.x = source.twist().angular().x();
    target.twist.twist.angular.y = source.twist().angular().y();
    target.twist.twist.angular.z = source.twist().angular().z();
    odometry_ros_pub_->publish(target);
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
  gz::transport::Node::Publisher command_velocity_gz_pub_;
  gz::transport::Node::Publisher actuator_enable_gz_pub_;
  gz::transport::Node::Publisher emergency_stop_gz_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_velocity_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr actuator_enable_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_ros_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_ros_pub_;
  std::mutex callback_mutex_;
  std::atomic<bool> stopping_{false};
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::A300DrivetrainNativeBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::A300DrivetrainNativeBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("a300_drivetrain_bridge"), "%s", error.what());
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
