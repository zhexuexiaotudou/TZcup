#include "SanitationMissionControl.hh"

#include <chrono>

#include <QCoreApplication>
#include <QMetaObject>
#include <QTimer>
#include <gz/plugin/Register.hh>

using namespace std::chrono_literals;

SanitationMissionControl::SanitationMissionControl()
: gz::gui::Plugin()
{
  Q_INIT_RESOURCE(SanitationMissionControl);
  this->context = std::make_shared<rclcpp::Context>();
  this->context->init(0, nullptr);
  rclcpp::NodeOptions nodeOptions;
  nodeOptions.context(this->context);
  this->node = std::make_shared<rclcpp::Node>(
    "sanitation_gazebo_mission_control", nodeOptions);
  this->stateSubscription = this->node->create_subscription<std_msgs::msg::String>(
    "/coverage/state", rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr _message) {
      this->SetMissionState(_message->data);
    });
  this->telemetrySubscription = this->node->create_subscription<std_msgs::msg::String>(
    "/coverage/gazebo_telemetry", rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr _message) {
      this->SetTelemetryJson(_message->data);
    });
  rclcpp::ExecutorOptions executorOptions;
  executorOptions.context = this->context;
  this->executor =
    std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
  this->executor->add_node(this->node);
  this->spinning = true;
  this->spinThread = std::thread([this]() {
    while (this->spinning && this->context->is_valid()) {
      this->executor->spin_some(100ms);
    }
  });
}

SanitationMissionControl::~SanitationMissionControl()
{
  this->spinning = false;
  if (this->executor) {
    this->executor->cancel();
  }
  if (this->spinThread.joinable()) {
    this->spinThread.join();
  }
  if (this->context && this->context->is_valid()) {
    this->context->shutdown("Gazebo mission control closed");
  }
}

void SanitationMissionControl::LoadConfig(
  const tinyxml2::XMLElement *_pluginElem)
{
  this->title = "清扫任务控制";
  if (!_pluginElem) {
    return;
  }
  const auto *scene = _pluginElem->FirstChildElement("scene_label");
  if (scene && scene->GetText()) {
    this->sceneLabel = scene->GetText();
    emit SceneLabelChanged();
  }
}

QString SanitationMissionControl::MissionState() const
{
  std::lock_guard<std::mutex> lock(this->textMutex);
  return QString::fromStdString(this->missionState);
}

QString SanitationMissionControl::OperatorMessage() const
{
  std::lock_guard<std::mutex> lock(this->textMutex);
  return QString::fromStdString(this->operatorMessage);
}

QString SanitationMissionControl::SceneLabel() const
{
  return QString::fromStdString(this->sceneLabel);
}

QString SanitationMissionControl::TelemetryJson() const
{
  std::lock_guard<std::mutex> lock(this->textMutex);
  return QString::fromStdString(this->telemetryJson);
}

void SanitationMissionControl::SetMissionState(const std::string &_state)
{
  {
    std::lock_guard<std::mutex> lock(this->textMutex);
    this->missionState = _state;
  }
  QMetaObject::invokeMethod(this, "MissionStateChanged", Qt::QueuedConnection);
}

void SanitationMissionControl::SetOperatorMessage(const std::string &_message)
{
  {
    std::lock_guard<std::mutex> lock(this->textMutex);
    this->operatorMessage = _message;
  }
  QMetaObject::invokeMethod(this, "OperatorMessageChanged", Qt::QueuedConnection);
}

void SanitationMissionControl::SetTelemetryJson(const std::string &_telemetry)
{
  {
    std::lock_guard<std::mutex> lock(this->textMutex);
    this->telemetryJson = _telemetry;
  }
  QMetaObject::invokeMethod(this, "TelemetryJsonChanged", Qt::QueuedConnection);
}

void SanitationMissionControl::Call(
  const std::string &_service, const std::string &_pendingMessage)
{
  auto client = this->node->create_client<std_srvs::srv::Trigger>(_service);
  if (!client->wait_for_service(500ms)) {
    this->SetOperatorMessage("任务服务尚未就绪，请稍后再试");
    return;
  }
  this->SetOperatorMessage(_pendingMessage);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  client->async_send_request(
    request,
    [this, client](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture _future) {
      const auto response = _future.get();
      this->SetOperatorMessage(response->message);
    });
}

void SanitationMissionControl::StartMission()
{
  this->Call("/coverage/control/start", "正在启动清扫任务…");
}

void SanitationMissionControl::PauseMission()
{
  this->Call("/coverage/control/pause", "正在安全暂停并关闭刷盘…");
}

void SanitationMissionControl::ResumeMission()
{
  this->Call("/coverage/control/resume", "正在续接当前清扫段…");
}

void SanitationMissionControl::StopMission()
{
  this->Call("/coverage/control/stop", "正在停止任务并关闭刷盘…");
}

void SanitationMissionControl::CloseGazebo()
{
  this->StopMission();
  this->SetOperatorMessage("正在安全停止任务并关闭 Gazebo…");
  QTimer::singleShot(800, QCoreApplication::instance(), &QCoreApplication::quit);
}

GZ_ADD_PLUGIN(SanitationMissionControl, gz::gui::Plugin)
