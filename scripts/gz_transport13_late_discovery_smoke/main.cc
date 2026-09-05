// Minimal transport-only late-discovery endpoint.  It deliberately has no
// Gazebo, ROS 2, or Docker dependency; the shell runner provides isolation and
// captures the two process maps while these endpoints are still alive.
#include <atomic>
#include <algorithm>
#include <chrono>
#include <climits>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

namespace {

struct Options {
  std::string mode;
  std::string topic;
  std::string report;
  int count{0};
  int periodMs{0};
  int holdMs{0};
};

bool ParsePositive(const char *raw, int *value) {
  try {
    const int parsed = std::stoi(raw);
    if (parsed <= 0) {
      return false;
    }
    *value = parsed;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

bool ParseArgs(int argc, char **argv, Options *options) {
  for (int index = 1; index < argc; index += 2) {
    if (index + 1 >= argc) {
      return false;
    }
    const std::string key(argv[index]);
    const char *value = argv[index + 1];
    if (key == "--mode") {
      options->mode = value;
    } else if (key == "--topic") {
      options->topic = value;
    } else if (key == "--report") {
      options->report = value;
    } else if (key == "--count") {
      if (!ParsePositive(value, &options->count)) {
        return false;
      }
    } else if (key == "--period-ms") {
      if (!ParsePositive(value, &options->periodMs)) {
        return false;
      }
    } else if (key == "--hold-ms") {
      if (!ParsePositive(value, &options->holdMs)) {
        return false;
      }
    } else {
      return false;
    }
  }
  return (options->mode == "publisher" || options->mode == "subscriber") &&
         !options->topic.empty() && !options->report.empty() &&
         options->count > 0 && options->periodMs > 0 && options->holdMs > 0;
}

bool WriteReport(const Options &options, bool endpointOk, int publishAttempts,
                 int publishedCount, int receivedCount, int uniqueCount,
                 int numericSequenceCount, int maxConsecutiveSequenceCount,
                 bool topicInfoOk, std::size_t topicInfoPublishers) {
  std::ofstream stream(options.report, std::ios::out | std::ios::trunc);
  if (!stream) {
    return false;
  }
  stream << "{\n"
         << "  \"mode\": \"" << options.mode << "\",\n"
         << "  \"topic\": \"" << options.topic << "\",\n"
         << "  \"endpoint_ok\": " << (endpointOk ? "true" : "false")
         << ",\n"
         << "  \"expected_count\": " << options.count << ",\n"
         << "  \"publish_attempt_count\": " << publishAttempts << ",\n"
         << "  \"published_count\": " << publishedCount << ",\n"
         << "  \"received_count\": " << receivedCount << ",\n"
         << "  \"unique_received_count\": " << uniqueCount << ",\n"
         << "  \"numeric_sequence_count\": " << numericSequenceCount << ",\n"
         << "  \"max_consecutive_sequence_count\": "
         << maxConsecutiveSequenceCount << ",\n"
         << "  \"topic_info_ok\": " << (topicInfoOk ? "true" : "false")
         << ",\n"
         << "  \"topic_info_publisher_count\": " << topicInfoPublishers
         << "\n}\n";
  return static_cast<bool>(stream);
}

bool ParseSequence(const std::string &raw, int *sequence) {
  try {
    std::size_t parsed = 0;
    const long long value = std::stoll(raw, &parsed, 10);
    if (parsed != raw.size() || value < 0 || value > INT_MAX) {
      return false;
    }
    *sequence = static_cast<int>(value);
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

int MaxConsecutiveSequenceCount(const std::vector<int> &sequences) {
  int longest = 0;
  int current = 0;
  int previous = -2;
  for (const int value : sequences) {
    current = value == previous + 1 ? current + 1 : 1;
    longest = std::max(longest, current);
    previous = value;
  }
  return longest;
}

void QueryTopicInfo(gz::transport::Node &node, const std::string &topic,
                    bool *topicInfoOk, std::size_t *publisherCount) {
  std::vector<gz::transport::MessagePublisher> publishers;
  const bool ok = node.TopicInfo(topic, publishers);
  if (ok) {
    *topicInfoOk = true;
    *publisherCount = publishers.size();
  }
}

int RunPublisher(const Options &options) {
  gz::transport::Node node;
  auto publisher = node.Advertise<gz::msgs::StringMsg>(options.topic);
  const bool endpointOk = static_cast<bool>(publisher);
  int publishedCount = 0;
  bool topicInfoOk = false;
  std::size_t topicInfoPublishers = 0;
  for (int sequence = 0; endpointOk && sequence < options.count; ++sequence) {
    gz::msgs::StringMsg message;
    message.set_data(std::to_string(sequence));
    if (publisher.Publish(message)) {
      ++publishedCount;
    }
    QueryTopicInfo(node, options.topic, &topicInfoOk, &topicInfoPublishers);
    std::this_thread::sleep_for(std::chrono::milliseconds(options.periodMs));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(options.holdMs));
  QueryTopicInfo(node, options.topic, &topicInfoOk, &topicInfoPublishers);
  const bool passed = endpointOk && publishedCount == options.count &&
                      topicInfoOk && topicInfoPublishers >= 1u;
  if (!WriteReport(options, endpointOk, options.count, publishedCount, 0, 0,
                   0, 0, topicInfoOk, topicInfoPublishers)) {
    return 125;
  }
  return passed ? 0 : 1;
}

int RunSubscriber(const Options &options) {
  gz::transport::Node node;
  std::atomic<int> receivedCount{0};
  std::mutex receivedMutex;
  std::set<std::string> uniqueMessages;
  std::set<int> numericSequences;
  std::vector<int> numericSequenceReceiveOrder;
  const bool endpointOk = node.Subscribe<gz::msgs::StringMsg>(
      options.topic, [&receivedCount, &receivedMutex, &uniqueMessages,
                      &numericSequences, &numericSequenceReceiveOrder](
                         const gz::msgs::StringMsg &message) {
        receivedCount.fetch_add(1);
        std::lock_guard<std::mutex> lock(receivedMutex);
        uniqueMessages.insert(message.data());
        int sequence = 0;
        if (ParseSequence(message.data(), &sequence)) {
          numericSequences.insert(sequence);
          numericSequenceReceiveOrder.push_back(sequence);
        }
      });
  bool topicInfoOk = false;
  std::size_t topicInfoPublishers = 0;
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(options.holdMs);
  while (std::chrono::steady_clock::now() < deadline) {
    QueryTopicInfo(node, options.topic, &topicInfoOk, &topicInfoPublishers);
    std::this_thread::sleep_for(std::chrono::milliseconds(options.periodMs));
  }
  int uniqueCount = 0;
  int numericSequenceCount = 0;
  int maxConsecutiveSequenceCount = 0;
  {
    std::lock_guard<std::mutex> lock(receivedMutex);
    uniqueCount = static_cast<int>(uniqueMessages.size());
    numericSequenceCount = static_cast<int>(numericSequences.size());
    maxConsecutiveSequenceCount =
        MaxConsecutiveSequenceCount(numericSequenceReceiveOrder);
  }
  const bool passed = endpointOk && receivedCount.load() >= options.count &&
                      uniqueCount >= options.count &&
                      maxConsecutiveSequenceCount >= options.count && topicInfoOk &&
                      topicInfoPublishers >= 1u;
  if (!WriteReport(options, endpointOk, 0, 0, receivedCount.load(), uniqueCount,
                   numericSequenceCount, maxConsecutiveSequenceCount,
                   topicInfoOk, topicInfoPublishers)) {
    return 125;
  }
  return passed ? 0 : 1;
}

}  // namespace

int main(int argc, char **argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) {
    std::cerr << "usage: --mode publisher|subscriber --topic TOPIC --report FILE "
                 "--count N --period-ms N --hold-ms N\n";
    return 2;
  }
  return options.mode == "publisher" ? RunPublisher(options)
                                     : RunSubscriber(options);
}
