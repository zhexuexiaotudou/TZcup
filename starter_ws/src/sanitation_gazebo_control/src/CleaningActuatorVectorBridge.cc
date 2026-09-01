#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <gz/msgs/double_v.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "sanitation_gazebo_control/CleaningActuatorMotorCore.hh"

namespace sanitation_gazebo_control
{
template<typename RosMessageT, typename GazeboMessageT>
struct GazeboToRosEndpoint
{
  const char * topic;
};

// These typed endpoint declarations are both consumed by the running bridge
// and audited by the formal component-register validator. Consequently the
// registered ROS / Gazebo types cannot drift away from the compiled endpoint.
constexpr GazeboToRosEndpoint<
  std_msgs::msg::Float64MultiArray, gz::msgs::Double_V> kMotorCurrentEndpoint{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/motor_current_a"};
constexpr GazeboToRosEndpoint<
  std_msgs::msg::Float64MultiArray, gz::msgs::Double_V> kMotorTemperatureEndpoint{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/motor_temperature_c"};
constexpr GazeboToRosEndpoint<
  std_msgs::msg::Float64MultiArray, gz::msgs::Double_V> kMotorOutputLoadEndpoint{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/estimated_output_load"};
constexpr GazeboToRosEndpoint<
  std_msgs::msg::Float64MultiArray, gz::msgs::Double_V> kTelemetrySnapshotEndpoint{
  "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot"};
constexpr std::size_t kExpectedMotorCount = 5;

class CleaningActuatorVectorBridge final : public rclcpp::Node
{
public:
  CleaningActuatorVectorBridge()
  : Node("cleaning_actuator_motor_bridge")
  {
    const std::string root =
      "/model/tzcup_formal_sanitation_vehicle/cleaning_motors";
    brush_gz_pub_ = gz_node_.Advertise<gz::msgs::Double_V>(
      root + "/command/brush");
    pump_gz_pub_ = gz_node_.Advertise<gz::msgs::Double_V>(
      root + "/command/pump");
    brush_ros_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      root + "/command/brush", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        PublishGazeboVector(*message, brush_gz_pub_);
      });
    pump_ros_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      root + "/command/pump", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr message) {
        PublishGazeboVector(*message, pump_gz_pub_);
      });
    current_ros_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      kMotorCurrentEndpoint.topic, 10);
    temperature_ros_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      kMotorTemperatureEndpoint.topic, 10);
    load_ros_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      kMotorOutputLoadEndpoint.topic, 10);
    telemetry_snapshot_ros_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      kTelemetrySnapshotEndpoint.topic, 50);
    if (!gz_node_.Subscribe(
        kMotorCurrentEndpoint.topic,
        &CleaningActuatorVectorBridge::OnCurrent, this) ||
      !gz_node_.Subscribe(
        kMotorTemperatureEndpoint.topic,
        &CleaningActuatorVectorBridge::OnTemperature, this) ||
      !gz_node_.Subscribe(
        kMotorOutputLoadEndpoint.topic,
        &CleaningActuatorVectorBridge::OnLoad, this) ||
      !gz_node_.Subscribe(
        kTelemetrySnapshotEndpoint.topic,
        &CleaningActuatorVectorBridge::OnTelemetrySnapshot, this))
    {
      throw std::runtime_error("failed to subscribe to cleaning actuator Gazebo vectors");
    }
    observability_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this]() {LogBridgeHealth();});
    RCLCPP_INFO(
      get_logger(),
      "active GZ->ROS Double_V bridge for %s, %s, %s and %s (motor length=%zu, snapshot length=%zu)",
      kMotorCurrentEndpoint.topic, kMotorTemperatureEndpoint.topic,
      kMotorOutputLoadEndpoint.topic, kTelemetrySnapshotEndpoint.topic,
      kExpectedMotorCount, kCleaningTelemetryValueCount);
  }

private:
  static void PublishGazeboVector(
    const std_msgs::msg::Float64MultiArray & source,
    gz::transport::Node::Publisher & publisher)
  {
    gz::msgs::Double_V target;
    for (const double value : source.data) {
      target.add_data(value);
    }
    publisher.Publish(target);
  }

  bool PublishTelemetry(
    const gz::msgs::Double_V & source,
    const char * topic,
    const rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr & publisher,
    std::atomic<std::uint64_t> & received_count,
    const std::size_t expected_length)
  {
    ++received_count;
    if (source.data_size() != static_cast<int>(expected_length)) {
      ++invalid_vector_count_;
      RCLCPP_ERROR(
        get_logger(), "dropping malformed Gazebo Double_V on %s: got %d values, expected %zu",
        topic, source.data_size(), expected_length);
      return false;
    }
    std_msgs::msg::Float64MultiArray target;
    target.data.reserve(static_cast<std::size_t>(source.data_size()));
    for (const double value : source.data()) {
      target.data.push_back(value);
    }
    publisher->publish(target);
    return true;
  }

  void OnCurrent(const gz::msgs::Double_V & message)
  {
    PublishTelemetry(
      message, kMotorCurrentEndpoint.topic, current_ros_pub_, current_received_count_,
      kExpectedMotorCount);
  }

  void OnTemperature(const gz::msgs::Double_V & message)
  {
    PublishTelemetry(
      message, kMotorTemperatureEndpoint.topic, temperature_ros_pub_,
      temperature_received_count_, kExpectedMotorCount);
  }

  void OnLoad(const gz::msgs::Double_V & message)
  {
    PublishTelemetry(
      message, kMotorOutputLoadEndpoint.topic, load_ros_pub_, load_received_count_,
      kExpectedMotorCount);
  }

  void OnTelemetrySnapshot(const gz::msgs::Double_V & message)
  {
    PublishTelemetry(
      message, kTelemetrySnapshotEndpoint.topic, telemetry_snapshot_ros_pub_,
      telemetry_snapshot_received_count_, kCleaningTelemetryValueCount);
  }

  void LogBridgeHealth()
  {
    RCLCPP_INFO(
      get_logger(),
      "GZ->ROS vector bridge health: current=%llu temperature=%llu load=%llu snapshot=%llu malformed=%llu; ROS subscriptions=%zu/%zu/%zu/%zu",
      static_cast<unsigned long long>(current_received_count_.load()),
      static_cast<unsigned long long>(temperature_received_count_.load()),
      static_cast<unsigned long long>(load_received_count_.load()),
      static_cast<unsigned long long>(telemetry_snapshot_received_count_.load()),
      static_cast<unsigned long long>(invalid_vector_count_.load()),
      current_ros_pub_->get_subscription_count(),
      temperature_ros_pub_->get_subscription_count(),
      load_ros_pub_->get_subscription_count(),
      telemetry_snapshot_ros_pub_->get_subscription_count());
  }

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher brush_gz_pub_;
  gz::transport::Node::Publisher pump_gz_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr brush_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pump_ros_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr current_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr temperature_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr load_ros_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    telemetry_snapshot_ros_pub_;
  rclcpp::TimerBase::SharedPtr observability_timer_;
  std::atomic<std::uint64_t> current_received_count_{0};
  std::atomic<std::uint64_t> temperature_received_count_{0};
  std::atomic<std::uint64_t> load_received_count_{0};
  std::atomic<std::uint64_t> telemetry_snapshot_received_count_{0};
  std::atomic<std::uint64_t> invalid_vector_count_{0};
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<sanitation_gazebo_control::CleaningActuatorVectorBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("cleaning_actuator_motor_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
