#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace
{
using json = nlohmann::json;
using namespace std::chrono_literals;

constexpr int kSchemaVersion = 1;
const std::vector<std::string> kTopics = {
  "/tf", "/tf_static", "/odom", "/odom/unfiltered", "/imu/data",
  "/amcl_pose", "/gnss/fix", "/odometry/gps",
  "/localization/fused_odom"};

std::string gid_hex(const uint8_t * data)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < RMW_GID_STORAGE_SIZE; ++index) {
    stream << std::setw(2) << static_cast<unsigned int>(data[index]);
  }
  return stream.str();
}

std::string gid_hex(
  const std::array<uint8_t, RMW_GID_STORAGE_SIZE> & data)
{
  return gid_hex(data.data());
}

std::string node_path(const std::string & node_namespace, const std::string & name)
{
  if (node_namespace.empty() || node_namespace == "/") {
    return "/" + name;
  }
  std::string cleaned = node_namespace;
  while (!cleaned.empty() && cleaned.back() == '/') {
    cleaned.pop_back();
  }
  return cleaned + "/" + name;
}

struct Options
{
  std::string mode;
  std::string output;
  double duration_seconds{20.0};
};

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--mode" && index + 1 < argc) {
      options.mode = argv[++index];
    } else if (argument == "--output" && index + 1 < argc) {
      options.output = argv[++index];
    } else if (argument == "--duration-seconds" && index + 1 < argc) {
      options.duration_seconds = std::stod(argv[++index]);
    }
  }
  if ((options.mode != "mapping" && options.mode != "cleaning") ||
    options.output.empty() || options.duration_seconds <= 0.0)
  {
    throw std::invalid_argument(
            "required: --mode mapping|cleaning --output PATH "
            "[--duration-seconds POSITIVE]");
  }
  return options;
}

class Collector : public rclcpp::Node
{
public:
  explicit Collector(const std::string & mode)
  : Node("formal_localization_runtime_collector"), mode_(mode)
  {
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);
    const auto static_qos = rclcpp::QoS(1).reliable().transient_local();
    add_tf_subscription("/tf", sensor_qos);
    add_tf_subscription("/tf_static", static_qos);
    add_subscription<nav_msgs::msg::Odometry>("/odom", sensor_qos);
    add_subscription<nav_msgs::msg::Odometry>("/odom/unfiltered", sensor_qos);
    add_subscription<sensor_msgs::msg::Imu>("/imu/data", sensor_qos);
    add_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/amcl_pose", sensor_qos);
    add_subscription<sensor_msgs::msg::NavSatFix>("/gnss/fix", sensor_qos);
    add_subscription<nav_msgs::msg::Odometry>("/odometry/gps", sensor_qos);
    add_subscription<nav_msgs::msg::Odometry>(
      "/localization/fused_odom", sensor_qos);
  }

  void snapshot_graph()
  {
    for (const auto & name : get_node_names()) {
      graph_nodes_.insert(name);
    }
    for (const auto & topic : kTopics) {
      record_endpoints(topic, true, get_publishers_info_by_topic(topic));
      record_endpoints(topic, false, get_subscriptions_info_by_topic(topic));
    }
  }

  json report(double duration_seconds)
  {
    snapshot_graph();
    json topics = json::object();
    for (const auto & topic : kTopics) {
      topics[topic] = {
        {"message_count", message_counts_[topic]},
        {"messages_by_gid", messages_by_gid_[topic]},
        {"publishers", endpoint_values(observed_publishers_[topic])},
        {"subscriptions", endpoint_values(observed_subscriptions_[topic])}};
    }
    json edges = json::object();
    for (const auto & [edge, counts] : tf_edges_) {
      std::uint64_t total = 0;
      for (const auto & [gid, count] : counts) {
        (void)gid;
        total += count;
      }
      edges[edge] = {{"message_count", total}, {"messages_by_gid", counts}};
    }
    return {
      {"schema_version", kSchemaVersion},
      {"mode", mode_},
      {"duration_seconds", duration_seconds},
      {"collector_contract", {
          {"world_truth_used", false}, {"subscribed_topics", kTopics}}},
      {"graph_nodes", graph_nodes_},
      {"endpoint_registry", endpoint_registry_},
      {"topics", topics},
      {"tf_edges", edges}};
  }

private:
  template<typename MessageT>
  void add_subscription(const std::string & topic, const rclcpp::QoS & qos)
  {
    auto subscription = create_subscription<MessageT>(
      topic, qos,
      [this, topic](const std::shared_ptr<MessageT>, const rclcpp::MessageInfo & info) {
        record_message(topic, info);
      });
    subscriptions_.push_back(subscription);
  }

  void add_tf_subscription(const std::string & topic, const rclcpp::QoS & qos)
  {
    auto subscription = create_subscription<tf2_msgs::msg::TFMessage>(
      topic, qos,
      [this, topic](
        const tf2_msgs::msg::TFMessage::SharedPtr message,
        const rclcpp::MessageInfo & info)
      {
        const auto gid = record_message(topic, info);
        for (const auto & transform : message->transforms) {
          auto parent = transform.header.frame_id;
          auto child = transform.child_frame_id;
          while (!parent.empty() && parent.front() == '/') {
            parent.erase(parent.begin());
          }
          while (!child.empty() && child.front() == '/') {
            child.erase(child.begin());
          }
          if (!parent.empty() && !child.empty()) {
            ++tf_edges_[parent + "->" + child][gid];
          }
        }
      });
    subscriptions_.push_back(subscription);
  }

  std::string record_message(
    const std::string & topic, const rclcpp::MessageInfo & info)
  {
    const auto & gid = info.get_rmw_message_info().publisher_gid;
    const auto value = gid_hex(gid.data);
    ++message_counts_[topic];
    ++messages_by_gid_[topic][value];
    return value;
  }

  static json endpoint_values(const std::map<std::string, json> & endpoints)
  {
    json values = json::array();
    for (const auto & [gid, endpoint] : endpoints) {
      (void)gid;
      values.push_back(endpoint);
    }
    return values;
  }

  void record_endpoints(
    const std::string & topic, bool publishers,
    const std::vector<rclcpp::TopicEndpointInfo> & values)
  {
    for (const auto & value : values) {
      const auto node = node_path(value.node_namespace(), value.node_name());
      if (!publishers && node == get_fully_qualified_name()) {
        continue;
      }
      const auto gid = gid_hex(value.endpoint_gid());
      const json endpoint = {
        {"gid", gid}, {"node", node}, {"topic_type", value.topic_type()}};
      endpoint_registry_[gid] = endpoint;
      if (publishers) {
        observed_publishers_[topic][gid] = endpoint;
      } else {
        observed_subscriptions_[topic][gid] = endpoint;
      }
    }
  }

  std::string mode_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  std::map<std::string, std::uint64_t> message_counts_;
  std::map<std::string, std::map<std::string, std::uint64_t>> messages_by_gid_;
  std::map<std::string, std::map<std::string, std::uint64_t>> tf_edges_;
  std::map<std::string, json> endpoint_registry_;
  std::map<std::string, std::map<std::string, json>> observed_publishers_;
  std::map<std::string, std::map<std::string, json>> observed_subscriptions_;
  std::set<std::string> graph_nodes_;
};
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const auto options = parse_options(argc, argv);
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<Collector>(options.mode);
    const auto started = std::chrono::steady_clock::now();
    const auto deadline = started + std::chrono::duration<double>(options.duration_seconds);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      rclcpp::spin_some(node);
      node->snapshot_graph();
      std::this_thread::sleep_for(20ms);
    }
    const auto elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started).count();
    std::ofstream output(options.output);
    if (!output) {
      throw std::runtime_error("cannot open output: " + options.output);
    }
    output << node->report(elapsed).dump(2) << '\n';
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception & error) {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    std::cerr << error.what() << '\n';
    return 2;
  }
}
