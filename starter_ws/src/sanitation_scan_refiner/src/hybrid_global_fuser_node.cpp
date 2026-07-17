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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace sanitation_scan_refiner
{
namespace
{
constexpr double kEarthRadiusM = 6378137.0;
constexpr double kPi = 3.14159265358979323846;

double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

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

struct OdomSample
{
  double stamp{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};
}  // namespace

class HybridGlobalFuserNode : public rclcpp::Node
{
public:
  HybridGlobalFuserNode()
  : Node("hybrid_global_fuser"), tf_broadcaster_(*this)
  {
    mode_ = declare_parameter<std::string>("mode", "hybrid_rtk_scan_imu_wheel");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    origin_latitude_deg_ = declare_parameter<double>("origin_latitude_deg", 31.2304);
    origin_longitude_deg_ = declare_parameter<double>("origin_longitude_deg", 121.4737);
    maximum_gnss_age_s_ = declare_parameter<double>("maximum_gnss_age_s", 0.5);
    maximum_refined_age_s_ = declare_parameter<double>("maximum_refined_age_s", 0.5);
    gnss_variance_scale_ = declare_parameter<double>("gnss_variance_scale", 1.0);
    gnss_outlier_threshold_m_ = declare_parameter<double>("gnss_outlier_threshold_m", 0.75);
    minimum_refined_variance_ = declare_parameter<double>(
      "minimum_refined_variance", 0.0025);
    maximum_refined_variance_ = declare_parameter<double>(
      "maximum_refined_variance", 1.0);
    publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);
    initial_pose_x_ = declare_parameter<double>("initial_pose_x", 0.0);
    initial_pose_y_ = declare_parameter<double>("initial_pose_y", 0.0);
    initial_pose_yaw_ = declare_parameter<double>("initial_pose_yaw", 0.0);
    const auto local_topic = declare_parameter<std::string>("local_odom_topic", "/odom");
    const auto gnss_topic = declare_parameter<std::string>("gnss_topic", "/gnss/fix");
    const auto refined_topic = declare_parameter<std::string>(
      "refined_pose_topic", "/localization/refined_pose");

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/localization/fused_pose", 20);
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/localization/fused_odom", 20);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/localization/fusion_diagnostics", 10);
    local_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      local_topic, 50, std::bind(&HybridGlobalFuserNode::onLocal, this, std::placeholders::_1));
    gnss_subscription_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      gnss_topic, 20, std::bind(&HybridGlobalFuserNode::onGnss, this, std::placeholders::_1));
    refined_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      refined_topic, 20,
      std::bind(&HybridGlobalFuserNode::onRefined, this, std::placeholders::_1));
  }

private:
  void onLocal(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    local_ = OdomSample{
      stampSeconds(message->header.stamp), message->pose.pose.position.x,
      message->pose.pose.position.y, tf2::getYaw(message->pose.pose.orientation)};
    if (!have_local_) {
      local_origin_ = local_;
    }
    local_twist_ = message->twist;
    local_history_.push_back(local_);
    while (local_history_.size() > 500 ||
      (!local_history_.empty() && local_.stamp - local_history_.front().stamp > 5.0))
    {
      local_history_.pop_front();
    }
    have_local_ = true;
    publishFusion(message->header.stamp);
  }

  void onGnss(const sensor_msgs::msg::NavSatFix::SharedPtr message)
  {
    if (message->status.status < sensor_msgs::msg::NavSatStatus::STATUS_FIX ||
      !std::isfinite(message->latitude) || !std::isfinite(message->longitude))
    {
      return;
    }
    const double latitude_rad = message->latitude * kPi / 180.0;
    const double origin_latitude_rad = origin_latitude_deg_ * kPi / 180.0;
    const double raw_x = (message->longitude - origin_longitude_deg_) * kPi / 180.0 *
      kEarthRadiusM * std::cos(origin_latitude_rad);
    const double raw_y = (latitude_rad - origin_latitude_rad) * kEarthRadiusM;
    const double variance = gnss_variance_scale_ * std::max(
      1e-6, std::max(message->position_covariance[0], message->position_covariance[4]));

    double projected_x = raw_x;
    double projected_y = raw_y;
    const double measurement_stamp = stampSeconds(message->header.stamp);
    const auto sample = closestLocal(measurement_stamp);
    if (sample && have_local_) {
      const auto projected = propagateGlobal(raw_x, raw_y, *sample);
      projected_x = projected.first;
      projected_y = projected.second;
    }

    if (have_global_ && std::hypot(projected_x - global_x_, projected_y - global_y_) >
      gnss_outlier_threshold_m_)
    {
      ++rejected_gnss_count_;
      publishDiagnostic("gnss_outlier_rejected", false, true);
      return;
    }
    gnss_x_ = projected_x;
    gnss_y_ = projected_y;
    gnss_variance_ = variance;
    gnss_local_anchor_ = local_;
    gnss_receive_stamp_ = now().seconds();
    have_gnss_ = true;
  }

  void onRefined(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (message->header.frame_id != map_frame_) {
      return;
    }
    refined_x_ = message->pose.pose.position.x;
    refined_y_ = message->pose.pose.position.y;
    refined_yaw_ = tf2::getYaw(message->pose.pose.orientation);
    const auto sample = closestLocal(stampSeconds(message->header.stamp));
    if (sample && have_local_) {
      const auto projected = propagateGlobal(refined_x_, refined_y_, *sample);
      refined_x_ = projected.first;
      refined_y_ = projected.second;
      refined_yaw_ += local_.yaw - sample->yaw;
    }
    refined_variance_ = std::clamp(
      std::max(message->pose.covariance[0], message->pose.covariance[7]),
      minimum_refined_variance_, maximum_refined_variance_);
    refined_yaw_variance_ = std::max(1e-6, message->pose.covariance[35]);
    refined_local_anchor_ = local_;
    refined_receive_stamp_ = now().seconds();
    have_refined_ = true;
  }

  std::optional<OdomSample> closestLocal(const double stamp) const
  {
    if (local_history_.empty()) {
      return std::nullopt;
    }
    auto best = local_history_.front();
    double best_delta = std::abs(best.stamp - stamp);
    for (const auto & sample : local_history_) {
      const double delta = std::abs(sample.stamp - stamp);
      if (delta < best_delta) {
        best = sample;
        best_delta = delta;
      }
    }
    return best;
  }

  std::pair<double, double> propagateGlobal(
    const double global_x, const double global_y, const OdomSample & local_anchor) const
  {
    const double cosine = std::cos(initial_pose_yaw_ - local_origin_.yaw);
    const double sine = std::sin(initial_pose_yaw_ - local_origin_.yaw);
    const double delta_x = local_.x - local_anchor.x;
    const double delta_y = local_.y - local_anchor.y;
    return {
      global_x + cosine * delta_x - sine * delta_y,
      global_y + sine * delta_x + cosine * delta_y};
  }

  void publishFusion(const builtin_interfaces::msg::Time & stamp)
  {
    const double current = now().seconds();
    const bool gnss_fresh = have_gnss_ && current - gnss_receive_stamp_ <= maximum_gnss_age_s_;
    const bool refined_fresh = have_refined_ &&
      current - refined_receive_stamp_ <= maximum_refined_age_s_;
    const auto gnss_position = propagateGlobal(gnss_x_, gnss_y_, gnss_local_anchor_);
    const auto refined_position = propagateGlobal(
      refined_x_, refined_y_, refined_local_anchor_);
    const double refined_yaw = refined_yaw_ + local_.yaw - refined_local_anchor_.yaw;

    double x = 0.0;
    double y = 0.0;
    double yaw = local_.yaw;
    double xy_variance = 0.25;
    double yaw_variance = 0.05;
    std::string source;
    if (mode_ == "rtk_imu_wheel" && gnss_fresh) {
      x = gnss_position.first;
      y = gnss_position.second;
      xy_variance = gnss_variance_;
      source = "rtk_plus_local_yaw";
    } else if (mode_ == "gnss_denied_scan_fallback" && refined_fresh) {
      x = refined_position.first;
      y = refined_position.second;
      yaw = refined_yaw;
      xy_variance = refined_variance_;
      yaw_variance = refined_yaw_variance_;
      source = "scan_fallback";
    } else if (mode_ == "hybrid_rtk_scan_imu_wheel" && (gnss_fresh || refined_fresh)) {
      if (gnss_fresh && refined_fresh) {
        const double gnss_weight = 1.0 / gnss_variance_;
        const double scan_weight = 1.0 / refined_variance_;
        x = (gnss_weight * gnss_position.first + scan_weight * refined_position.first) /
          (gnss_weight + scan_weight);
        y = (gnss_weight * gnss_position.second + scan_weight * refined_position.second) /
          (gnss_weight + scan_weight);
        xy_variance = 1.0 / (gnss_weight + scan_weight);
        yaw = local_.yaw;
        yaw_variance = 0.01;
        source = "rtk_scan_local";
      } else if (gnss_fresh) {
        x = gnss_position.first;
        y = gnss_position.second;
        xy_variance = gnss_variance_;
        source = "rtk_local_fallback";
      } else {
        x = refined_position.first;
        y = refined_position.second;
        yaw = refined_yaw;
        xy_variance = refined_variance_;
        yaw_variance = refined_yaw_variance_;
        source = "scan_local_fallback";
      }
    } else {
      const double delta_yaw = local_.yaw - local_origin_.yaw;
      const double cosine = std::cos(initial_pose_yaw_ - local_origin_.yaw);
      const double sine = std::sin(initial_pose_yaw_ - local_origin_.yaw);
      const double delta_x = local_.x - local_origin_.x;
      const double delta_y = local_.y - local_origin_.y;
      x = initial_pose_x_ + cosine * delta_x - sine * delta_y;
      y = initial_pose_y_ + sine * delta_x + cosine * delta_y;
      yaw = initial_pose_yaw_ + delta_yaw;
      xy_variance = 0.25;
      yaw_variance = 0.10;
      source = "local_prior_only";
    }

    global_x_ = x;
    global_y_ = y;
    have_global_ = true;
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose.position.x = x;
    pose.pose.pose.position.y = y;
    pose.pose.pose.orientation = yawQuaternion(yaw);
    pose.pose.covariance[0] = xy_variance;
    pose.pose.covariance[7] = xy_variance;
    pose.pose.covariance[35] = yaw_variance;
    pose_publisher_->publish(pose);

    nav_msgs::msg::Odometry odometry;
    odometry.header = pose.header;
    odometry.child_frame_id = base_frame_;
    odometry.pose = pose.pose;
    odometry.twist = local_twist_;
    odom_publisher_->publish(odometry);

    if (publish_map_to_odom_) {
      tf2::Transform map_to_base;
      map_to_base.setOrigin(tf2::Vector3(x, y, 0.0));
      tf2::Quaternion map_rotation;
      map_rotation.setRPY(0.0, 0.0, yaw);
      map_to_base.setRotation(map_rotation);
      tf2::Transform odom_to_base;
      odom_to_base.setOrigin(tf2::Vector3(local_.x, local_.y, 0.0));
      tf2::Quaternion odom_rotation;
      odom_rotation.setRPY(0.0, 0.0, local_.yaw);
      odom_to_base.setRotation(odom_rotation);
      const tf2::Transform map_to_odom = map_to_base * odom_to_base.inverse();
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = odom_frame_;
      transform.transform = tf2::toMsg(map_to_odom);
      tf_broadcaster_.sendTransform(transform);
    }
    publishDiagnostic(source, true, false);
  }

  void publishDiagnostic(const std::string & source, const bool available, const bool outlier)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "sanitation_hybrid_global_fuser";
    status.hardware_id = "standard_ros_interfaces";
    status.level = available ? diagnostic_msgs::msg::DiagnosticStatus::OK :
      diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = source;
    status.values.push_back(keyValue("mode", mode_));
    status.values.push_back(keyValue("source", source));
    status.values.push_back(keyValue("global_available", available));
    status.values.push_back(keyValue("gnss_outlier_rejected", outlier));
    status.values.push_back(keyValue("rejected_gnss_count", rejected_gnss_count_));
    status.values.push_back(keyValue("map_to_odom_owner", publish_map_to_odom_));
    status.values.push_back(keyValue("ground_truth_direct_fusion", false));
    array.status.push_back(status);
    diagnostics_publisher_->publish(array);
  }

  std::string mode_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  double origin_latitude_deg_{0.0};
  double origin_longitude_deg_{0.0};
  double maximum_gnss_age_s_{0.5};
  double maximum_refined_age_s_{0.5};
  double gnss_variance_scale_{1.0};
  double gnss_outlier_threshold_m_{0.75};
  double minimum_refined_variance_{0.0025};
  double maximum_refined_variance_{1.0};
  bool publish_map_to_odom_{true};
  double initial_pose_x_{0.0};
  double initial_pose_y_{0.0};
  double initial_pose_yaw_{0.0};
  bool have_local_{false};
  bool have_gnss_{false};
  bool have_refined_{false};
  bool have_global_{false};
  OdomSample local_;
  OdomSample local_origin_;
  std::deque<OdomSample> local_history_;
  geometry_msgs::msg::TwistWithCovariance local_twist_;
  double gnss_x_{0.0};
  double gnss_y_{0.0};
  double gnss_variance_{0.04};
  double gnss_receive_stamp_{0.0};
  OdomSample gnss_local_anchor_;
  double refined_x_{0.0};
  double refined_y_{0.0};
  double refined_yaw_{0.0};
  double refined_variance_{0.04};
  double refined_yaw_variance_{0.05};
  double refined_receive_stamp_{0.0};
  OdomSample refined_local_anchor_;
  double global_x_{0.0};
  double global_y_{0.0};
  std::uint64_t rejected_gnss_count_{0};
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    refined_subscription_;
};
}  // namespace sanitation_scan_refiner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sanitation_scan_refiner::HybridGlobalFuserNode>());
  rclcpp::shutdown();
  return 0;
}
