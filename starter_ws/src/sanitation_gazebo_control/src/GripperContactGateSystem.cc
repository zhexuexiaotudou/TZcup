// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/empty.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Static.hh>
#include <gz/transport/Node.hh>

#include "sanitation_gazebo_control/ContactGateCore.hh"

namespace sanitation_gazebo_control
{
/// Collapse simulator collision identities into an identity-free dual-contact
/// signal.  Collision names are deliberately confined to this Gazebo system;
/// product ROS nodes receive only a Boolean physical-contact observation.
class GripperContactGateSystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    if (const auto *name =
        _ecm.Component<gz::sim::components::Name>(_entity))
      this->vehicleModelName = name->Data();
    if (_sdf->HasElement("left_contact_topic"))
      this->leftTopic = _sdf->Get<std::string>("left_contact_topic");
    if (_sdf->HasElement("right_contact_topic"))
      this->rightTopic = _sdf->Get<std::string>("right_contact_topic");
    if (_sdf->HasElement("output_topic"))
      this->outputTopic = _sdf->Get<std::string>("output_topic");
    if (_sdf->HasElement("attach_topic"))
      this->attachTopic = _sdf->Get<std::string>("attach_topic");
    if (_sdf->HasElement("detach_topic"))
      this->detachTopic = _sdf->Get<std::string>("detach_topic");
    if (_sdf->HasElement("state_topic"))
      this->stateTopic = _sdf->Get<std::string>("state_topic");
    if (_sdf->HasElement("parent_link"))
      this->parentLinkName = _sdf->Get<std::string>("parent_link");
    if (_sdf->HasElement("maximum_contact_age_sec"))
      this->maximumContactAgeSec =
          _sdf->Get<double>("maximum_contact_age_sec");

    this->publisher =
        this->node.Advertise<gz::msgs::Boolean>(this->outputTopic);
    this->statePublisher =
        this->node.Advertise<gz::msgs::Boolean>(this->stateTopic);
    this->parentLinkEntity =
        gz::sim::Model(_entity).LinkByName(_ecm, this->parentLinkName);
    if (!this->leftTopic.empty())
      this->leftSubscribed = this->node.Subscribe(
          this->leftTopic, &GripperContactGateSystem::OnLeft, this);
    if (!this->rightTopic.empty())
      this->rightSubscribed = this->node.Subscribe(
          this->rightTopic, &GripperContactGateSystem::OnRight, this);
    this->node.Subscribe(
        this->attachTopic, &GripperContactGateSystem::OnAttach, this);
    this->node.Subscribe(
        this->detachTopic, &GripperContactGateSystem::OnDetach, this);
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt.count() <= 0)
      return;
    this->DiscoverContactTopics(_info);
    this->publishAccumulator +=
        std::chrono::duration<double>(_info.dt).count();
    if (this->publishAccumulator < 0.01)
      return;
    this->publishAccumulator = 0.0;

    const auto now = std::chrono::steady_clock::now();
    bool dualContact = false;
    gz::sim::Entity contactedCollision{gz::sim::kNullEntity};
    bool attachRequested = false;
    bool detachRequested = false;
    {
      std::lock_guard<std::mutex> guard(this->mutex);
      const bool leftFresh = this->leftSeen &&
          std::chrono::duration<double>(now - this->leftUpdate).count() <=
              this->maximumContactAgeSec;
      const bool rightFresh = this->rightSeen &&
          std::chrono::duration<double>(now - this->rightUpdate).count() <=
              this->maximumContactAgeSec;
      if (leftFresh && rightFresh)
      {
        const auto common = CommonContact(
            this->leftExternal, this->rightExternal);
        if (common.has_value())
        {
          dualContact = true;
          contactedCollision = common.value();
        }
      }
      attachRequested = this->attachRequested &&
          std::chrono::duration<double>(now - this->attachRequestTime).count()
              <= this->maximumAttachRequestAgeSec;
      detachRequested = this->detachRequested;
      if (!attachRequested)
        this->attachRequested = false;
      this->detachRequested = false;
    }
    gz::msgs::Boolean message;
    message.set_data(dualContact);
    this->publisher.Publish(message);

    if (detachRequested && this->jointEntity != gz::sim::kNullEntity)
    {
      _ecm.RequestRemoveEntity(this->jointEntity);
      this->jointEntity = gz::sim::kNullEntity;
      this->PublishAttachmentState(false);
    }
    else if (detachRequested)
    {
      this->PublishAttachmentState(false);
    }

    if (attachRequested && dualContact &&
        this->jointEntity == gz::sim::kNullEntity &&
        this->parentLinkEntity != gz::sim::kNullEntity)
    {
      auto childLink = contactedCollision;
      while (childLink != gz::sim::kNullEntity &&
          _ecm.Component<gz::sim::components::Link>(childLink) == nullptr)
        childLink = _ecm.ParentEntity(childLink);
      bool hasStaticAncestor = false;
      auto ancestor = childLink;
      while (ancestor != gz::sim::kNullEntity)
      {
        const auto *staticComponent =
            _ecm.Component<gz::sim::components::Static>(ancestor);
        if (staticComponent != nullptr && staticComponent->Data())
        {
          hasStaticAncestor = true;
          break;
        }
        ancestor = _ecm.ParentEntity(ancestor);
      }
      if (DynamicBodyEligible(
          childLink != gz::sim::kNullEntity, hasStaticAncestor))
      {
        const gz::sim::components::DetachableJointInfo info{
            this->parentLinkEntity, childLink, "fixed"};
        this->jointEntity = _ecm.CreateEntity();
        _ecm.CreateComponent(
            this->jointEntity,
            gz::sim::components::DetachableJoint(info));
        {
          std::lock_guard<std::mutex> guard(this->mutex);
          this->attachRequested = false;
        }
        this->PublishAttachmentState(true);
      }
    }
  }

  private: std::set<gz::sim::Entity> ExternalCollisions(
      const gz::msgs::Contacts &_message) const
  {
    std::set<gz::sim::Entity> result;
    for (int index = 0; index < _message.contact_size(); ++index)
    {
      const auto &contact = _message.contact(index);
      for (const auto *entity :
          {&contact.collision1(), &contact.collision2()})
      {
        const auto &collision = entity->name();
        if (IsExternalCollisionName(collision, this->vehicleModelName))
          result.insert(static_cast<gz::sim::Entity>(entity->id()));
      }
    }
    return result;
  }

  private: bool EndsWith(
      const std::string &_value, const std::string &_suffix) const
  {
    return _value.size() >= _suffix.size() &&
        _value.compare(
            _value.size() - _suffix.size(), _suffix.size(), _suffix) == 0;
  }

  private: void DiscoverContactTopics(const gz::sim::UpdateInfo &_info)
  {
    if (this->leftSubscribed && this->rightSubscribed)
      return;
    this->discoveryAccumulator +=
        std::chrono::duration<double>(_info.dt).count();
    if (this->discoveryAccumulator < 0.25)
      return;
    this->discoveryAccumulator = 0.0;
    std::vector<std::string> topics;
    this->node.TopicList(topics);
    const auto leftSuffix = "/model/" + this->vehicleModelName +
        "/link/robotiq_85_left_finger_tip_link/sensor/"
        "left_finger_tip_contact/contact";
    const auto rightSuffix = "/model/" + this->vehicleModelName +
        "/link/robotiq_85_right_finger_tip_link/sensor/"
        "right_finger_tip_contact/contact";
    for (const auto &topic : topics)
    {
      if (!this->leftSubscribed && this->EndsWith(topic, leftSuffix))
      {
        this->leftTopic = topic;
        this->leftSubscribed = this->node.Subscribe(
            topic, &GripperContactGateSystem::OnLeft, this);
      }
      if (!this->rightSubscribed && this->EndsWith(topic, rightSuffix))
      {
        this->rightTopic = topic;
        this->rightSubscribed = this->node.Subscribe(
            topic, &GripperContactGateSystem::OnRight, this);
      }
    }
  }

  private: void OnLeft(const gz::msgs::Contacts &_message)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->leftExternal = this->ExternalCollisions(_message);
    this->leftUpdate = std::chrono::steady_clock::now();
    this->leftSeen = true;
  }

  private: void OnRight(const gz::msgs::Contacts &_message)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->rightExternal = this->ExternalCollisions(_message);
    this->rightUpdate = std::chrono::steady_clock::now();
    this->rightSeen = true;
  }

  private: void OnAttach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->attachRequested = true;
    this->attachRequestTime = std::chrono::steady_clock::now();
  }

  private: void OnDetach(const gz::msgs::Empty &)
  {
    std::lock_guard<std::mutex> guard(this->mutex);
    this->detachRequested = true;
    this->attachRequested = false;
  }

  private: void PublishAttachmentState(const bool _attached)
  {
    gz::msgs::Boolean state;
    state.set_data(_attached);
    this->statePublisher.Publish(state);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: gz::transport::Node::Publisher statePublisher;
  private: std::mutex mutex;
  private: std::set<gz::sim::Entity> leftExternal;
  private: std::set<gz::sim::Entity> rightExternal;
  private: std::chrono::steady_clock::time_point leftUpdate;
  private: std::chrono::steady_clock::time_point rightUpdate;
  private: bool leftSeen{false};
  private: bool rightSeen{false};
  private: bool leftSubscribed{false};
  private: bool rightSubscribed{false};
  private: bool attachRequested{false};
  private: bool detachRequested{false};
  private: double maximumContactAgeSec{0.15};
  private: double maximumAttachRequestAgeSec{0.5};
  private: double publishAccumulator{0.0};
  private: double discoveryAccumulator{0.0};
  private: std::string vehicleModelName;
  private: std::string leftTopic;
  private: std::string rightTopic;
  private: std::string outputTopic{"/manipulation/gripper/dual_contact"};
  private: std::string attachTopic{"/manipulation/grasp/attach"};
  private: std::string detachTopic{"/manipulation/grasp/detach"};
  private: std::string stateTopic{"/manipulation/grasp/state"};
  private: std::string parentLinkName{"ur5e_wrist_3_link"};
  private: std::chrono::steady_clock::time_point attachRequestTime;
  private: gz::sim::Entity parentLinkEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity jointEntity{gz::sim::kNullEntity};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
    sanitation_gazebo_control::GripperContactGateSystem,
    gz::sim::System,
    sanitation_gazebo_control::GripperContactGateSystem::ISystemConfigure,
    sanitation_gazebo_control::GripperContactGateSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_control::GripperContactGateSystem,
    "sanitation_gazebo_control::GripperContactGateSystem")
