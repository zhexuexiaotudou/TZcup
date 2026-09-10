#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>

#include <gz/msgs/laserscan.pb.h>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "sanitation_gazebo_control/NativeBridgeSupport.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr GazeboToRosEndpoint<sensor_msgs::msg::LaserScan, gz::msgs::LaserScan>
  kFormalLidarScan{"/sensors/lidar_2d/scan"};
constexpr char kRosScanTopic[] = "/scan";
constexpr auto kPublishPeriod = std::chrono::milliseconds(20);
}  // namespace

// This bridge owns exactly one direction and one raw ROS topic.  Keeping it
// separate from the product telemetry bridge prevents a high-rate scan callback
// from delaying GNSS/IMU or actuator transport work.
class FormalLidarNativeBridge final : public NativeBridgeSupport
{
public:
  FormalLidarNativeBridge()
  : NativeBridgeSupport("formal_vehicle_lidar_bridge")
  {
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(1));
    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      // /scan is an internal one-hop handoff to the self-filter.  Reliable
      // depth-one delivery prevents a matched late-starting filter from
      // observing zero samples, while the latest-only GZ slot still bounds
      // memory and prevents historical replay.
      kRosScanTopic, sensor_qos.reliable().durability_volatile());

    if (!Subscribe(
        kFormalLidarScan.topic, &FormalLidarNativeBridge::OnScan, this))
    {
      StopAndThrow("failed to subscribe to formal Gazebo lidar");
    }

    publish_timer_ = create_wall_timer(
      kPublishPeriod, [this]() { PublishLatestScan(); });
    health_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this]() {
        RCLCPP_INFO(
          get_logger(),
          "health: gazebo_scans=%llu ros_scans=%llu coalesced_scans=%llu ros_subscriptions=%zu",
          static_cast<unsigned long long>(gazebo_scan_count_.load()),
          static_cast<unsigned long long>(ros_scan_count_.load()),
          static_cast<unsigned long long>(coalesced_scan_count_.load()),
          scan_publisher_->get_subscription_count());
      });
  }

private:
  void OnScan(const gz::msgs::LaserScan & message)
  {
    // Keep Gazebo Transport's callback short: conversion of 1081-ray scans
    // belongs on the ROS executor, not its receive thread.  A pending slot is
    // deliberately overwritten so no historical scan can be replayed.
    const std::lock_guard<std::mutex> callback_lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    {
      const std::lock_guard<std::mutex> slot_lock(latest_scan_mutex_);
      if (latest_scan_ready_) {
        coalesced_scan_count_.fetch_add(1, std::memory_order_relaxed);
      }
      latest_scan_.CopyFrom(message);
      latest_scan_ready_ = true;
    }
    gazebo_scan_count_.fetch_add(1, std::memory_order_relaxed);
  }

  void PublishLatestScan()
  {
    gz::msgs::LaserScan latest;
    {
      const std::lock_guard<std::mutex> slot_lock(latest_scan_mutex_);
      if (!latest_scan_ready_) {
        return;
      }
      latest_scan_.Swap(&latest);
      latest_scan_ready_ = false;
    }
    // Preserve Gazebo's header exactly.  If transport is still behind /clock,
    // collision_monitor must see that age and fail closed rather than receive
    // an invented current timestamp.
    PublishGazeboToRos<sensor_msgs::msg::LaserScan, gz::msgs::LaserScan>(
      latest, scan_publisher_);
    ros_scan_count_.fetch_add(1, std::memory_order_relaxed);
  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  gz::msgs::LaserScan latest_scan_;
  std::mutex latest_scan_mutex_;
  bool latest_scan_ready_{false};
  std::atomic<std::uint64_t> gazebo_scan_count_{0};
  std::atomic<std::uint64_t> ros_scan_count_{0};
  std::atomic<std::uint64_t> coalesced_scan_count_{0};
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::FormalLidarNativeBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::FormalLidarNativeBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formal_vehicle_lidar_bridge"), "%s", error.what());
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
