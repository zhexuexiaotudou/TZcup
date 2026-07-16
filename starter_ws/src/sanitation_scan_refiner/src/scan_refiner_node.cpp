// Copyright 2026 Sanitation Vehicle Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sanitation_scan_refiner/scan_refiner_core.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sanitation_scan_refiner
{
namespace
{
geometry_msgs::msg::Quaternion yawQuaternion(const double yaw)
{
  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(quaternion);
}

template<typename ValueT>
diagnostic_msgs::msg::KeyValue keyValue(const std::string & key, const ValueT & value)
{
  std::ostringstream stream;
  stream << value;
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = stream.str();
  return item;
}
}  // namespace

class ScanRefinerNode : public rclcpp::Node
{
public:
  ScanRefinerNode()
  : Node("scan_refiner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    maximum_scan_range_m_ = declare_parameter<double>("maximum_scan_range_m", 20.0);
    const auto map_topic = declare_parameter<std::string>("map_topic", "/map");
    const auto scan_topic = declare_parameter<std::string>("scan_topic", "/scan");
    const auto prior_topic = declare_parameter<std::string>("prior_topic", "/amcl_pose");
    const auto pose_topic = declare_parameter<std::string>(
      "refined_pose_topic", "/localization/refined_pose");
    const auto odom_topic = declare_parameter<std::string>(
      "refined_odom_topic", "/localization/refined_odom");
    const auto diagnostics_topic = declare_parameter<std::string>(
      "diagnostics_topic", "/localization/refiner_diagnostics");

    config_ = defaultSearchConfig();
    config_.minimum_points = static_cast<std::size_t>(
      declare_parameter<int>("minimum_points", static_cast<int>(config_.minimum_points)));
    config_.maximum_points = static_cast<std::size_t>(
      declare_parameter<int>("maximum_points", static_cast<int>(config_.maximum_points)));
    config_.maximum_score_m = declare_parameter<double>(
      "maximum_score_m", config_.maximum_score_m);
    config_.minimum_improvement_m = declare_parameter<double>(
      "minimum_improvement_m", config_.minimum_improvement_m);

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic,
        10);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic, 10);
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ScanRefinerNode::onMap, this, std::placeholders::_1));
    prior_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      prior_topic, 20, std::bind(&ScanRefinerNode::onPrior, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::SensorDataQoS(),
      std::bind(&ScanRefinerNode::onScan, this, std::placeholders::_1));
  }

private:
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr message)
  {
    const auto & orientation = message->info.origin.orientation;
    field_ = std::make_unique<OccupancyDistanceField>(
      message->info.width, message->info.height, message->info.resolution,
      message->info.origin.position.x, message->info.origin.position.y,
      tf2::getYaw(orientation), message->data);
    map_frame_ = message->header.frame_id.empty() ? map_frame_ : message->header.frame_id;
    RCLCPP_INFO(
      get_logger(), "Distance field ready: %ux%u at %.3f m",
      message->info.width, message->info.height, message->info.resolution);
  }

  void onPrior(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      publishDiagnostic("prior_frame_mismatch", nullptr, 0.0);
      return;
    }
    prior_ = Pose2{
      message->pose.pose.position.x, message->pose.pose.position.y,
      tf2::getYaw(message->pose.pose.orientation)};
    prior_stamp_ = message->header.stamp;
    have_prior_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!field_ || !field_->valid()) {
      publishDiagnostic("map_unavailable", nullptr, 0.0);
      return;
    }
    if (!have_prior_) {
      publishDiagnostic("prior_unavailable", nullptr, 0.0);
      return;
    }

    geometry_msgs::msg::TransformStamped sensor_to_base;
    try {
      sensor_to_base = tf_buffer_.lookupTransform(
        base_frame_, scan->header.frame_id, scan->header.stamp,
        rclcpp::Duration::from_seconds(0.05));
    } catch (const tf2::TransformException & error) {
      publishDiagnostic(std::string("scan_tf_unavailable:") + error.what(), nullptr, 0.0);
      return;
    }

    std::vector<Point2> points;
    points.reserve(scan->ranges.size());
    double angle = scan->angle_min;
    for (const float range : scan->ranges) {
      const double usable_maximum = std::min(
        maximum_scan_range_m_, static_cast<double>(scan->range_max) - 1e-3);
      if (std::isfinite(range) && range >= scan->range_min && range < usable_maximum) {
        geometry_msgs::msg::PointStamped sensor_point;
        geometry_msgs::msg::PointStamped base_point;
        sensor_point.header = scan->header;
        sensor_point.point.x = static_cast<double>(range) * std::cos(angle);
        sensor_point.point.y = static_cast<double>(range) * std::sin(angle);
        tf2::doTransform(sensor_point, base_point, sensor_to_base);
        points.push_back({base_point.point.x, base_point.point.y});
      }
      angle += scan->angle_increment;
    }

    const auto started = std::chrono::steady_clock::now();
    const RefinementResult result = refinePose(*field_, points, prior_, config_);
    const double latency_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    ++attempt_count_;
    if (result.accepted) {
      ++accepted_count_;
    } else {
      ++rejected_count_;
    }
    publishDiagnostic(result.reason, &result, latency_ms);
    if (!result.accepted) {
      return;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = scan->header.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = result.pose.x;
    pose.pose.pose.position.y = result.pose.y;
    pose.pose.pose.orientation = yawQuaternion(result.pose.yaw);
    pose.pose.covariance[0] = result.covariance_xx;
    pose.pose.covariance[7] = result.covariance_yy;
    pose.pose.covariance[35] = result.covariance_yaw_yaw;
    pose_publisher_->publish(pose);

    nav_msgs::msg::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = base_frame_;
    odometry.pose = pose.pose;
    odom_publisher_->publish(odometry);
  }

  void publishDiagnostic(
    const std::string & reason, const RefinementResult * result, const double latency_ms)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sanitation_scan_refiner";
    status.hardware_id = "cpu_scan_matcher";
    status.level = result && result->accepted ?
      diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = reason;
    status.values.push_back(keyValue("accepted", result && result->accepted));
    status.values.push_back(keyValue("reason", reason));
    status.values.push_back(keyValue("ground_truth_dependency", false));
    status.values.push_back(keyValue("latency_ms", latency_ms));
    status.values.push_back(keyValue("attempt_count", attempt_count_));
    status.values.push_back(keyValue("accepted_count", accepted_count_));
    status.values.push_back(keyValue("rejected_count", rejected_count_));
    status.values.push_back(keyValue("accepted_ever", accepted_count_ > 0));
    if (result) {
      status.values.push_back(keyValue("score_m", result->score_m));
      status.values.push_back(keyValue("prior_score_m", result->prior_score_m));
      status.values.push_back(keyValue("improvement_m", result->improvement_m));
      status.values.push_back(keyValue("valid_points", result->valid_point_count));
      status.values.push_back(keyValue("evaluated_candidates", result->evaluated_candidates));
      status.values.push_back(keyValue("observable", result->observable));
      status.values.push_back(keyValue("curvature_x", result->curvature_x));
      status.values.push_back(keyValue("curvature_y", result->curvature_y));
      status.values.push_back(keyValue("curvature_yaw", result->curvature_yaw));
      status.values.push_back(keyValue("covariance_xx", result->covariance_xx));
      status.values.push_back(keyValue("covariance_yy", result->covariance_yy));
      status.values.push_back(keyValue("covariance_yaw_yaw", result->covariance_yaw_yaw));
    }
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::string map_frame_;
  std::string base_frame_;
  double maximum_scan_range_m_{20.0};
  SearchConfig config_;
  std::unique_ptr<OccupancyDistanceField> field_;
  Pose2 prior_;
  builtin_interfaces::msg::Time prior_stamp_;
  bool have_prior_{false};
  std::uint64_t attempt_count_{0};
  std::uint64_t accepted_count_{0};
  std::uint64_t rejected_count_{0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    prior_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};
}  // namespace sanitation_scan_refiner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sanitation_scan_refiner::ScanRefinerNode>());
  rclcpp::shutdown();
  return 0;
}
