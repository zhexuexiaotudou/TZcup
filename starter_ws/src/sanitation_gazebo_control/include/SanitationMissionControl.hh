#ifndef SANITATION_GAZEBO_CONTROL__SANITATION_MISSION_CONTROL_HH_
#define SANITATION_GAZEBO_CONTROL__SANITATION_MISSION_CONTROL_HH_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gz/gui/Plugin.hh>
#include <gz/gui/qt.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

class SanitationMissionControl : public gz::gui::Plugin
{
  Q_OBJECT
  Q_PROPERTY(QString missionState READ MissionState NOTIFY MissionStateChanged)
  Q_PROPERTY(QString operatorMessage READ OperatorMessage NOTIFY OperatorMessageChanged)
  Q_PROPERTY(QString sceneLabel READ SceneLabel NOTIFY SceneLabelChanged)
  Q_PROPERTY(QString telemetryJson READ TelemetryJson NOTIFY TelemetryJsonChanged)

public:
  SanitationMissionControl();
  ~SanitationMissionControl() override;
  void LoadConfig(const tinyxml2::XMLElement *_pluginElem) override;

  QString MissionState() const;
  QString OperatorMessage() const;
  QString SceneLabel() const;
  QString TelemetryJson() const;

signals:
  void MissionStateChanged();
  void OperatorMessageChanged();
  void SceneLabelChanged();
  void TelemetryJsonChanged();

protected slots:
  void StartMission();
  void PauseMission();
  void ResumeMission();
  void StopMission();
  void CloseGazebo();

private:
  void Call(const std::string &_service, const std::string &_pendingMessage);
  void SetMissionState(const std::string &_state);
  void SetOperatorMessage(const std::string &_message);
  void SetTelemetryJson(const std::string &_telemetry);

  std::shared_ptr<rclcpp::Context> context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stateSubscription;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr telemetrySubscription;
  std::thread spinThread;
  std::atomic_bool spinning{false};
  mutable std::mutex textMutex;
  std::string missionState{"WAITING_FOR_NODE"};
  std::string operatorMessage{"等待清扫任务节点就绪"};
  std::string sceneLabel{"小型演示 · 30 m × 20 m"};
  std::string telemetryJson{"{\"state\":\"WAITING_FOR_DATA\"}"};
};

#endif
