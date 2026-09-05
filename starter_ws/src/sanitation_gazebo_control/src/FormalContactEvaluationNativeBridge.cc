#include <array>
#include <memory>
#include <string>

#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/double.pb.h>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_msgs/msg/float64.hpp>

#include "sanitation_gazebo_control/NativeBridgeSupport.hh"

namespace sanitation_gazebo_control
{
namespace
{
constexpr GroupedGazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSqueegeeFloatPosition{
  "squeegee", "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_position_m"};
constexpr GroupedGazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSqueegeeFloatVelocity{
  "squeegee", "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_velocity_m_s"};
constexpr GroupedGazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSqueegeeFloatForce{
  "squeegee", "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/float_force_n"};
constexpr GroupedGazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSqueegeePitchPosition{
  "squeegee", "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_position_rad"};
constexpr GroupedGazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSqueegeePitchVelocity{
  "squeegee", "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_velocity_rad_s"};
constexpr GroupedGazeboToRosEndpoint<std_msgs::msg::Float64, gz::msgs::Double> kSqueegeePitchTorque{
  "squeegee", "/model/tzcup_formal_sanitation_vehicle/squeegee_compliance/pitch_torque_nm"};
constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kSqueegeeContact{
  "squeegee", "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/squeegee_link/sensor/squeegee_blade_ground_contact/contact"};

constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kLeftSideBrushContact{
  "brushes", "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/left_side_brush_link/sensor/left_side_brush_ground_contact/contact"};
constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kRightSideBrushContact{
  "brushes", "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/right_side_brush_link/sensor/right_side_brush_ground_contact/contact"};
constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kCentralRollerContact{
  "brushes", "/world/formal_vehicle_validation/model/tzcup_formal_sanitation_vehicle/link/central_roller_link/sensor/central_roller_ground_contact/contact"};

constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kChargeReceptacleContact{"charge_receptacle", "/formal_vehicle/gazebo/charge_receptacle/contact"};
constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kWastewaterDrainContact{"wastewater_drain", "/formal_vehicle/gazebo/wastewater_drain_coupling/contact"};
constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kFrontBumperContact{"front_bumper", "/safety/front_bumper/contact"};
constexpr GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>
  kRearBumperContact{"rear_bumper", "/safety/rear_bumper/contact"};

constexpr std::array<const char *, 6> kSqueegeeFloatTopics{{
  kSqueegeeFloatPosition.topic, kSqueegeeFloatVelocity.topic, kSqueegeeFloatForce.topic,
  kSqueegeePitchPosition.topic, kSqueegeePitchVelocity.topic, kSqueegeePitchTorque.topic,
}};
}  // namespace

class FormalContactEvaluationNativeBridge final : public NativeBridgeSupport
{
public:
  FormalContactEvaluationNativeBridge()
  : NativeBridgeSupport("formal_contact_evaluation_native_bridge")
  {
    declare_parameter<std::string>("endpoint_group", "");
    const auto group = get_parameter("endpoint_group").as_string();
    if (group == "squeegee") {
      ConfigureSqueegee();
    } else if (group == "brushes") {
      ConfigureBrushes();
    } else if (group == "charge_receptacle") {
      ConfigureSingleContact(kChargeReceptacleContact);
    } else if (group == "wastewater_drain") {
      ConfigureSingleContact(kWastewaterDrainContact);
    } else if (group == "front_bumper") {
      ConfigureSingleContact(kFrontBumperContact);
    } else if (group == "rear_bumper") {
      ConfigureSingleContact(kRearBumperContact);
    } else {
      StopAndThrow("endpoint_group must name exactly one formal contact bridge group");
    }
  }

private:
  void ConfigureSqueegee()
  {
    for (std::size_t index = 0; index < kSqueegeeFloatTopics.size(); ++index) {
      squeegee_float_pubs_[index] = create_publisher<std_msgs::msg::Float64>(
        kSqueegeeFloatTopics[index], 10);
    }
    squeegee_contact_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      kSqueegeeContact.topic, 10);
    bool subscribed = true;
    subscribed = Subscribe(kSqueegeeFloatPosition.topic, &FormalContactEvaluationNativeBridge::OnSqueegeeFloatPosition, this) && subscribed;
    subscribed = Subscribe(kSqueegeeFloatVelocity.topic, &FormalContactEvaluationNativeBridge::OnSqueegeeFloatVelocity, this) && subscribed;
    subscribed = Subscribe(kSqueegeeFloatForce.topic, &FormalContactEvaluationNativeBridge::OnSqueegeeFloatForce, this) && subscribed;
    subscribed = Subscribe(kSqueegeePitchPosition.topic, &FormalContactEvaluationNativeBridge::OnSqueegeePitchPosition, this) && subscribed;
    subscribed = Subscribe(kSqueegeePitchVelocity.topic, &FormalContactEvaluationNativeBridge::OnSqueegeePitchVelocity, this) && subscribed;
    subscribed = Subscribe(kSqueegeePitchTorque.topic, &FormalContactEvaluationNativeBridge::OnSqueegeePitchTorque, this) && subscribed;
    subscribed = Subscribe(kSqueegeeContact.topic, &FormalContactEvaluationNativeBridge::OnSqueegeeContact, this) && subscribed;
    if (!subscribed) {
      StopAndThrow("failed to subscribe to squeegee evaluation endpoints");
    }
  }

  void ConfigureBrushes()
  {
    left_brush_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      kLeftSideBrushContact.topic, 10);
    right_brush_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      kRightSideBrushContact.topic, 10);
    roller_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(
      kCentralRollerContact.topic, 10);
    bool subscribed = true;
    subscribed = Subscribe(
      kLeftSideBrushContact.topic, &FormalContactEvaluationNativeBridge::OnLeftBrush, this) && subscribed;
    subscribed = Subscribe(
      kRightSideBrushContact.topic, &FormalContactEvaluationNativeBridge::OnRightBrush, this) && subscribed;
    subscribed = Subscribe(
      kCentralRollerContact.topic, &FormalContactEvaluationNativeBridge::OnCentralRoller, this) && subscribed;
    if (!subscribed) {
      StopAndThrow("failed to subscribe to brush evaluation endpoints");
    }
  }

  void ConfigureSingleContact(
    const GroupedGazeboToRosEndpoint<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts> & endpoint)
  {
    single_contact_pub_ = create_publisher<ros_gz_interfaces::msg::Contacts>(endpoint.topic, 10);
    if (!Subscribe(endpoint.topic, &FormalContactEvaluationNativeBridge::OnSingleContact, this)) {
      StopAndThrow("failed to subscribe to formal service or safety contact endpoint");
    }
  }

  void PublishSqueegeeFloat(const gz::msgs::Double & message, std::size_t index)
  {
    PublishGazeboToRos<std_msgs::msg::Float64, gz::msgs::Double>(message, squeegee_float_pubs_[index]);
  }
  void OnSqueegeeFloatPosition(const gz::msgs::Double & message) { PublishSqueegeeFloat(message, 0); }
  void OnSqueegeeFloatVelocity(const gz::msgs::Double & message) { PublishSqueegeeFloat(message, 1); }
  void OnSqueegeeFloatForce(const gz::msgs::Double & message) { PublishSqueegeeFloat(message, 2); }
  void OnSqueegeePitchPosition(const gz::msgs::Double & message) { PublishSqueegeeFloat(message, 3); }
  void OnSqueegeePitchVelocity(const gz::msgs::Double & message) { PublishSqueegeeFloat(message, 4); }
  void OnSqueegeePitchTorque(const gz::msgs::Double & message) { PublishSqueegeeFloat(message, 5); }
  void OnSqueegeeContact(const gz::msgs::Contacts & message)
  {
    PublishGazeboToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(
      message, squeegee_contact_pub_);
  }
  void OnLeftBrush(const gz::msgs::Contacts & message)
  {
    PublishGazeboToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(message, left_brush_pub_);
  }
  void OnRightBrush(const gz::msgs::Contacts & message)
  {
    PublishGazeboToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(message, right_brush_pub_);
  }
  void OnCentralRoller(const gz::msgs::Contacts & message)
  {
    PublishGazeboToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(message, roller_pub_);
  }
  void OnSingleContact(const gz::msgs::Contacts & message)
  {
    PublishGazeboToRos<ros_gz_interfaces::msg::Contacts, gz::msgs::Contacts>(message, single_contact_pub_);
  }

  std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kSqueegeeFloatTopics.size()> squeegee_float_pubs_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr squeegee_contact_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr left_brush_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr right_brush_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr roller_pub_;
  rclcpp::Publisher<ros_gz_interfaces::msg::Contacts>::SharedPtr single_contact_pub_;
};
}  // namespace sanitation_gazebo_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<sanitation_gazebo_control::FormalContactEvaluationNativeBridge> bridge;
  try {
    bridge = std::make_shared<sanitation_gazebo_control::FormalContactEvaluationNativeBridge>();
    rclcpp::spin(bridge);
    bridge->Stop();
    bridge.reset();
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("formal_contact_evaluation_native_bridge"), "%s", error.what());
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
