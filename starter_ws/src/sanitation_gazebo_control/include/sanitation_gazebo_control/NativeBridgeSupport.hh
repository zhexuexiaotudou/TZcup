#ifndef SANITATION_GAZEBO_CONTROL__NATIVE_BRIDGE_SUPPORT_HH_
#define SANITATION_GAZEBO_CONTROL__NATIVE_BRIDGE_SUPPORT_HH_

#include <atomic>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/convert.hpp>

namespace sanitation_gazebo_control
{

template<typename RosMessageT, typename GazeboMessageT>
struct GazeboToRosEndpoint
{
  const char * topic;
};

template<typename RosMessageT, typename GazeboMessageT>
struct GroupedGazeboToRosEndpoint
{
  const char * group;
  const char * topic;
};

template<typename RosMessageT, typename GazeboMessageT>
struct RosToGazeboEndpoint
{
  const char * topic;
};

// The formal launch owns topic selection and single-writer policy.  Native
// bridge implementations share this small lifecycle primitive so each GZ
// callback is drained only after its transport subscription is removed.
class NativeBridgeSupport : public rclcpp::Node
{
public:
  explicit NativeBridgeSupport(const std::string & name)
  : Node(name)
  {
  }

  ~NativeBridgeSupport() override
  {
    Stop();
  }

  void Stop()
  {
    if (stopping_.exchange(true)) {
      return;
    }
    for (const auto & topic : gazebo_subscription_topics_) {
      gz_node_.Unsubscribe(topic);
    }
    const std::lock_guard<std::mutex> drain(callback_mutex_);
  }

protected:
  template<typename CallbackT, typename ObjectT>
  bool Subscribe(
    const char * topic,
    CallbackT callback,
    ObjectT * object)
  {
    if (!gz_node_.Subscribe(topic, callback, object)) {
      return false;
    }
    gazebo_subscription_topics_.emplace_back(topic);
    return true;
  }

  template<typename RosMessageT, typename GazeboMessageT>
  void PublishGazeboToRos(
    const GazeboMessageT & source,
    const typename rclcpp::Publisher<RosMessageT>::SharedPtr & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    RosMessageT target;
    ros_gz_bridge::convert_gz_to_ros(source, target);
    publisher->publish(target);
  }

  template<typename RosMessageT, typename GazeboMessageT>
  void PublishRosToGazebo(
    const RosMessageT & source,
    gz::transport::Node::Publisher & publisher)
  {
    const std::lock_guard<std::mutex> lock(callback_mutex_);
    if (stopping_.load()) {
      return;
    }
    GazeboMessageT target;
    ros_gz_bridge::convert_ros_to_gz(source, target);
    publisher.Publish(target);
  }

  [[noreturn]] void StopAndThrow(const char * message)
  {
    Stop();
    throw std::runtime_error(message);
  }

  gz::transport::Node gz_node_;
  std::atomic<bool> stopping_{false};
  std::mutex callback_mutex_;

private:
  std::vector<std::string> gazebo_subscription_topics_;
};

}  // namespace sanitation_gazebo_control

#endif  // SANITATION_GAZEBO_CONTROL__NATIVE_BRIDGE_SUPPORT_HH_
