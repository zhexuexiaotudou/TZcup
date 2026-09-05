#include <array>
#include <memory>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/imu.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/msgs/navsat.pb.h>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include "sanitation_gazebo_control/NativeBridgeSupport.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr GazeboToRosEndpoint<sensor_msgs::msg::LaserScan, gz::msgs::LaserScan> kLidarScan{
  "/sensors/lidar_2d/scan"};
constexpr GazeboToRosEndpoint<sensor_msgs::msg::NavSatFix, gz::msgs::NavSat> kGnssFix{
  "/sensors/gnss/fix"};
constexpr GazeboToRosEndpoint<sensor_msgs::msg::Imu, gz::msgs::IMU> kImuData{
  "/sensors/imu/data"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kEnableCommand{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/enable"};
constexpr RosToGazeboEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kServiceDrainCommand{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/command/service_drain_open"};

constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kWastewaterMass{
  "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg/applied"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kTankMass{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_mass_kg"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kTankLevel{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_level_fraction"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kFlow{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/flow_l_min"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kRecoveredVolume{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/recovered_volume_l"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSensedFlow{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_flow_l_min"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSensedTankLevel{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/sensed_tank_level_fraction"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kFilterPressure{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_differential_pressure_kpa"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kPumpCurrent{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/pump_current_a"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kDrainedVolume{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drained_volume_l"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kDryBinFill{
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/fill_level_fraction"};

constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kTankFull{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_full"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kLowProbe{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_low_probe_wet"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kHighProbe{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/tank_high_probe_wet"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kFilterProtection{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/filter_protection_active"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kDrainOpen{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_open"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kDrainPermitted{
  "/model/tzcup_formal_sanitation_vehicle/water_recovery/service_drain_permitted"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kDryBinFull{
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/full"};
constexpr GazeboToRosEndpoint<std_msgs::msg::Bool, gz::msgs::Boolean> kDryBinSensorReady{
  "/model/tzcup_formal_sanitation_vehicle/dry_bin/sensor_ready"};

constexpr GazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kSuctionContact{"/cleaning/suction_nozzle/contact"};
constexpr GazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kDryDepositContact{"/storage/dry_deposit/contact"};

constexpr std::array<const char *, 11> kFloatTopics{{
  kWastewaterMass.topic, kTankMass.topic, kTankLevel.topic, kFlow.topic, kRecoveredVolume.topic,
  kSensedFlow.topic, kSensedTankLevel.topic, kFilterPressure.topic, kPumpCurrent.topic,
  kDrainedVolume.topic, kDryBinFill.topic,
}};
constexpr std::array<const char *, 8> kBoolTopics{{
  kTankFull.topic, kLowProbe.topic, kHighProbe.topic, kFilterProtection.topic, kDrainOpen.topic,
  kDrainPermitted.topic, kDryBinFull.topic, kDryBinSensorReady.topic,
}};
constexpr std::array<const char *, 2> kContactTopics{{kSuctionContact.topic, kDryDepositContact.topic}};
}  // namespace

class FormalVehicleProductNativeBridge final : public NativeBridgeSupport
{
public:
  FormalVehicleProductNativeBridge()
  : NativeBridgeSupport("formal_vehicle_product_native_bridge")
  {
    enable_gz_pub_ = gz_node_.Advertise<gz::msgs::Boolean>(kEnableCommand.topic);
    service_drain_gz_pub_ = gz_node_.Advertise<gz::msgs::Boolean>(kServiceDrainCommand.topic);
    enable_ros_sub_ = create_subscription<std_msgs::msg::Bool>(
      kEnableCommand.topic, 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
        PublishRosToGazebo<std_msgs::msg::Bool, gz::msgs::Boolean>(*message, enable_gz_pub_);
      });
    service_drain_ros_sub_ = create_subscription<std_msgs::msg::Bool>(
      kServiceDrainCommand.topic, 10, [this](const std_msgs::msg::Bool::SharedPtr message) {
        PublishRosToGazebo<std_msgs::msg::Bool, gz::msgs::Boolean>(
          *message, service_drain_gz_pub_);
      });

    lidar_ros_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(kLidarScan.topic, 10);
    gnss_ros_pub_ = create_publisher<sensor_msgs::msg::NavSatFix>(kGnssFix.topic, 10);
    imu_ros_pub_ = create_publisher<sensor_msgs::msg::Imu>(kImuData.topic, 10);
    for (std::size_t index = 0; index < kFloatTopics.size(); ++index) {
      float_ros_pubs_[index] = create_publisher<std_msgs::msg::Float64>(kFloatTopics[index], 10);
    }
    for (std::size_t index = 0; index < kBoolTopics.size(); ++index) {
      bool_ros_pubs_[index] = create_publisher<std_msgs::msg::Bool>(kBoolTopics[index], 10);
    }
    for (std::size_t index = 0; index < kContactTopics.size(); ++index) {
      contact_ros_pubs_[index] = create_publisher<ros_gz_interfaces::msg::Contacts>(
        kContactTopics[index], 10);
    }

    bool subscribed = true;
    subscribed = Subscribe(kLidarScan.topic, &FormalVehicleProductNativeBridge::OnLidarScan, this) && subscribed;
    subscribed = Subscribe(kGnssFix.topic, &FormalVehicleProductNativeBridge::OnGnssFix, this) && subscribed;
    subscribed = Subscribe(kImuData.topic, &FormalVehicleProductNativeBridge::OnImuData, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[0], &FormalVehicleProductNativeBridge::OnWastewaterMass, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[1], &FormalVehicleProductNativeBridge::OnTankMass, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[2], &FormalVehicleProductNativeBridge::OnTankLevel, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[3], &FormalVehicleProductNativeBridge::OnFlow, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[4], &FormalVehicleProductNativeBridge::OnRecoveredVolume, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[5], &FormalVehicleProductNativeBridge::OnSensedFlow, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[6], &FormalVehicleProductNativeBridge::OnSensedTankLevel, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[7], &FormalVehicleProductNativeBridge::OnFilterPressure, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[8], &FormalVehicleProductNativeBridge::OnPumpCurrent, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[9], &FormalVehicleProductNativeBridge::OnDrainedVolume, this) && subscribed;
    subscribed = Subscribe(kFloatTopics[10], &FormalVehicleProductNativeBridge::OnDryBinFill, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[0], &FormalVehicleProductNativeBridge::OnTankFull, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[1], &FormalVehicleProductNativeBridge::OnLowProbe, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[2], &FormalVehicleProductNativeBridge::OnHighProbe, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[3], &FormalVehicleProductNativeBridge::OnFilterProtection, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[4], &FormalVehicleProductNativeBridge::OnDrainOpen, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[5], &FormalVehicleProductNativeBridge::OnDrainPermitted, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[6], &FormalVehicleProductNativeBridge::OnDryBinFull, this) && subscribed;
    subscribed = Subscribe(kBoolTopics[7], &FormalVehicleProductNativeBridge::OnDryBinSensorReady, this) && subscribed;
    subscribed = Subscribe(kContactTopics[0], &FormalVehicleProductNativeBridge::OnSuctionContact, this) && subscribed;
    subscribed = Subscribe(kContactTopics[1], &FormalVehicleProductNativeBridge::OnDryDepositContact, this) && subscribed;
    if (!subscribed) {
      StopAndThrow("failed to subscribe to formal vehicle product telemetry");
    }
  }

private:
  void OnLidarScan(const gz::msgs::LaserScan & message)
  {
    PublishGazeboToRos<sensor_msgs::msg::LaserScan, gz::msgs::LaserScan>(message, lidar_ros_pub_);
  }
  void OnGnssFix(const gz::msgs::NavSat & message)
  {
    PublishGazeboToRos<sensor_msgs::msg::NavSatFix, gz::msgs::NavSat>(message, gnss_ros_pub_);
  }
  void OnImuData(const gz::msgs::IMU & message)
  {
    PublishGazeboToRos<sensor_msgs::msg::Imu, gz::msgs::IMU>(message, imu_ros_pub_);
  }
  void PublishFloat(const gz::msgs::Double & message, std::size_t index)
  {
    PublishGazeboToRos<std_msgs::msg::Float64, gz::msgs::Double>(message, float_ros_pubs_[index]);
  }
  void OnWastewaterMass(const gz::msgs::Double & message) { PublishFloat(message, 0); }
  void OnTankMass(const gz::msgs::Double & message) { PublishFloat(message, 1); }
  void OnTankLevel(const gz::msgs::Double & message) { PublishFloat(message, 2); }
  void OnFlow(const gz::msgs::Double & message) { PublishFloat(message, 3); }
  void OnRecoveredVolume(const gz::msgs::Double & message) { PublishFloat(message, 4); }
  void OnSensedFlow(const gz::msgs::Double & message) { PublishFloat(message, 5); }
  void OnSensedTankLevel(const gz::msgs::Double & message) { PublishFloat(message, 6); }
  void OnFilterPressure(const gz::msgs::Double & message) { PublishFloat(message, 7); }
  void OnPumpCurrent(const gz::msgs::Double & message) { PublishFloat(message, 8); }
  void OnDrainedVolume(const gz::msgs::Double & message) { PublishFloat(message, 9); }
  void OnDryBinFill(const gz::msgs::Double & message) { PublishFloat(message, 10); }
  void PublishBool(const gz::msgs::Boolean & message, std::size_t index)
  {
    PublishGazeboToRos<std_msgs::msg::Bool, gz::msgs::Boolean>(message, bool_ros_pubs_[index]);
  }
  void OnTankFull(const gz::msgs::Boolean & message) { PublishBool(message, 0); }
  void OnLowProbe(const gz::msgs::Boolean & message) { PublishBool(message, 1); }
  void OnHighProbe(const gz::msgs::Boolean & message) { PublishBool(message, 2); }
  void OnFilterProtection(const gz::msgs::Boolean & message) { PublishBool(message, 3); }
  void OnDrainOpen(const gz::msgs::Boolean & message) { PublishBool(message, 4); }
  void OnDrainPermitted(const gz::msgs::Boolean & message) { PublishBool(message, 5); }
  void OnDryBinFull(const gz::msgs::Boolean & message) { PublishBool(message, 6); }
  void OnDryBinSensorReady(const gz::msgs::Boolean & message) { PublishBool(message, 7); }
  void PublishContacts(const gz::msgs::Contacts & message, std::size_t index)
  {
    PublishGazeboToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(
      message, contact_ros_pubs_[index]);
  }
  void OnSuctionContact(const gz::msgs::Contacts & message) { PublishContacts(message, 0); }
  void OnDryDepositContact(const gz::msgs::Contacts & message) { PublishContacts(message, 1); }

  gz::transport::Node::Publisher enable_gz_pub_;
  gz::transport::Node::Publisher service_drain_gz_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_ros_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr service_drain_ros_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_ros_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_ros_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_ros_pub_;
  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kFloatTopics.size()> float_ros_pubs_;
  std::array<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr, kBoolTopics.size()> bool_ros_pubs_;
  std::array<rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr, kContactTopics.size()> contact_ros_pubs_;
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::FormalVehicleProductNativeBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::FormalVehicleProductNativeBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formal_vehicle_product_native_bridge"), "%s", error.what());
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
