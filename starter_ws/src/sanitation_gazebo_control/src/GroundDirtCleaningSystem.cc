// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Transparency.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>

namespace sanitation_gazebo_control
{
namespace
{
double FirstValue(
    const gz::sim::EntityComponentManager &_ecm,
    const gz::sim::Entity _entity,
    const bool _velocity)
{
  if (_entity == gz::sim::kNullEntity)
    return 0.0;
  if (_velocity)
  {
    const auto *component =
        _ecm.Component<gz::sim::components::JointVelocity>(_entity);
    return component != nullptr && !component->Data().empty()
        ? component->Data().front() : 0.0;
  }
  const auto *component =
      _ecm.Component<gz::sim::components::JointPosition>(_entity);
  return component != nullptr && !component->Data().empty()
      ? component->Data().front() : 0.0;
}

double DistanceToSegment(
    const gz::math::Vector2d &_point,
    const gz::math::Vector2d &_start,
    const gz::math::Vector2d &_end)
{
  const auto delta = _end - _start;
  const double lengthSquared = delta.SquaredLength();
  if (lengthSquared <= 1e-12)
    return (_point - _start).Length();
  const double projection = std::clamp(
      (_point - _start).Dot(delta) / lengthSquared, 0.0, 1.0);
  return (_point - (_start + delta * projection)).Length();
}
}

/// Evaluation-only ground-soiling state driven by the formal vehicle's real
/// cleaning joints and link poses.
///
/// Dirt is a deterministic raster of visual cells emitted by the campus
/// generator.  A cell becomes transparent only when its centre is inside the
/// measured world-space sweep of a lowered, rotating side brush or central
/// roller.  The system never owns rigid litter models and never removes any
/// entity; discrete litter remains available exclusively to the manipulation
/// and dry-bin chain.
class GroundDirtCleaningSystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  private: struct Cell
  {
    gz::sim::Entity visual{gz::sim::kNullEntity};
    gz::math::Vector2d centre{0.0, 0.0};
    bool cleaned{false};
  };

  private: struct SweepState
  {
    gz::math::Vector2d centre{0.0, 0.0};
    double yaw{0.0};
    bool valid{false};
  };

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    const gz::sim::Model vehicle(_entity);
    this->leftBrushJoint = vehicle.JointByName(_ecm, "left_side_brush_joint");
    this->rightBrushJoint = vehicle.JointByName(_ecm, "right_side_brush_joint");
    this->rollerJoint = vehicle.JointByName(_ecm, "central_roller_joint");
    this->liftJoint = vehicle.JointByName(_ecm, "cleaning_lift_joint");
    this->leftBrushLink = vehicle.LinkByName(_ecm, "left_side_brush_link");
    this->rightBrushLink = vehicle.LinkByName(_ecm, "right_side_brush_link");
    this->rollerLink = vehicle.LinkByName(_ecm, "central_roller_link");

    for (const auto entity : {
        this->leftBrushJoint, this->rightBrushJoint, this->rollerJoint})
    {
      if (entity != gz::sim::kNullEntity)
        gz::sim::enableComponent<gz::sim::components::JointVelocity>(
            _ecm, entity, true);
    }
    if (this->liftJoint != gz::sim::kNullEntity)
      gz::sim::enableComponent<gz::sim::components::JointPosition>(
          _ecm, this->liftJoint, true);

    this->ReadDouble(_sdf, "cell_area_m2", this->cellAreaM2);
    this->ReadDouble(_sdf, "sweep_sample_spacing_m",
        this->sweepSampleSpacingM);
    this->ReadDouble(_sdf, "minimum_rotation_rad_s", this->minimumRotationRadS);
    this->ReadDouble(_sdf, "minimum_lift_position_m", this->minimumLiftPositionM);
    this->ReadDouble(_sdf, "side_brush_radius_m", this->sideBrushRadiusM);
    this->ReadDouble(_sdf, "side_brush_link_to_ground_m",
        this->sideBrushLinkToGroundM);
    this->ReadDouble(_sdf, "roller_radius_m", this->rollerRadiusM);
    this->ReadDouble(_sdf, "roller_width_m", this->rollerWidthM);
    this->ReadDouble(_sdf, "maximum_contact_clearance_m",
        this->maximumContactClearanceM);
    this->ReadDouble(_sdf, "publish_rate_hz", this->publishRateHz);
    if (_sdf->HasElement("enabled_by_default"))
      this->enabled.store(_sdf->Get<bool>("enabled_by_default"));

    this->node.Subscribe(this->enableTopic,
        &GroundDirtCleaningSystem::OnEnable, this);
    this->initialAreaPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/initial_area_m2");
    this->cleanedAreaPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/cleaned_area_m2");
    this->remainingAreaPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/remaining_area_m2");
    this->coveragePublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/cleaned_fraction");
    this->statusPublisher = this->node.Advertise<gz::msgs::StringMsg>(
        this->stateRoot + "/status_json");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt.count() <= 0)
      return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    if (!this->cellLayoutReady)
      this->DiscoverCells(_ecm);

    const double lift = FirstValue(_ecm, this->liftJoint, false);
    const double leftVelocity = std::abs(
        FirstValue(_ecm, this->leftBrushJoint, true));
    const double rightVelocity = std::abs(
        FirstValue(_ecm, this->rightBrushJoint, true));
    const double rollerVelocity = std::abs(
        FirstValue(_ecm, this->rollerJoint, true));
    const bool lowered = this->liftJoint != gz::sim::kNullEntity &&
        lift >= this->minimumLiftPositionM;

    const auto leftPose = this->Pose(this->leftBrushLink, _ecm);
    const auto rightPose = this->Pose(this->rightBrushLink, _ecm);
    const auto rollerPose = this->Pose(this->rollerLink, _ecm);
    this->leftClearanceM = leftPose.Pos().Z() - this->sideBrushLinkToGroundM;
    this->rightClearanceM = rightPose.Pos().Z() - this->sideBrushLinkToGroundM;
    this->rollerClearanceM = rollerPose.Pos().Z() - this->rollerRadiusM;
    const bool systemEnabled = this->enabled.load() && this->cellLayoutReady;
    this->leftReady = systemEnabled && lowered &&
        leftVelocity >= this->minimumRotationRadS &&
        this->ContactReady(this->leftBrushLink, this->leftClearanceM);
    this->rightReady = systemEnabled && lowered &&
        rightVelocity >= this->minimumRotationRadS &&
        this->ContactReady(this->rightBrushLink, this->rightClearanceM);
    this->rollerReady = systemEnabled && lowered &&
        rollerVelocity >= this->minimumRotationRadS &&
        this->ContactReady(this->rollerLink, this->rollerClearanceM);

    const SweepState left = this->State(leftPose, this->leftBrushLink);
    const SweepState right = this->State(rightPose, this->rightBrushLink);
    const SweepState roller = this->State(rollerPose, this->rollerLink);
    if (this->leftReady)
      this->CleanCircularSweep(_ecm, this->previousLeft, left);
    if (this->rightReady)
      this->CleanCircularSweep(_ecm, this->previousRight, right);
    if (this->rollerReady)
      this->CleanRollerSweep(_ecm, this->previousRoller, roller);

    this->previousLeft = this->leftReady ? left : SweepState{};
    this->previousRight = this->rightReady ? right : SweepState{};
    this->previousRoller = this->rollerReady ? roller : SweepState{};
    this->leftVelocityRadS = leftVelocity;
    this->rightVelocityRadS = rightVelocity;
    this->rollerVelocityRadS = rollerVelocity;
    this->liftPositionM = lift;
    this->leftWorld = left;
    this->rightWorld = right;
    this->rollerWorld = roller;

    this->publishAccumulator += dt;
    const double period = this->publishRateHz > 0.0
        ? 1.0 / this->publishRateHz : 0.1;
    if (this->publishAccumulator >= period)
    {
      this->PublishState();
      this->publishAccumulator = 0.0;
    }
  }

  private: void ReadDouble(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double &_value)
  {
    if (_sdf->HasElement(_name))
      _value = _sdf->Get<double>(_name);
  }

  private: gz::math::Pose3d Pose(
      const gz::sim::Entity _entity,
      const gz::sim::EntityComponentManager &_ecm) const
  {
    return _entity == gz::sim::kNullEntity
        ? gz::math::Pose3d() : gz::sim::worldPose(_entity, _ecm);
  }

  private: SweepState State(
      const gz::math::Pose3d &_pose,
      const gz::sim::Entity _entity) const
  {
    return {{_pose.Pos().X(), _pose.Pos().Y()}, _pose.Rot().Yaw(),
        _entity != gz::sim::kNullEntity};
  }

  private: bool ContactReady(
      const gz::sim::Entity _link, const double _clearance) const
  {
    return _link != gz::sim::kNullEntity &&
        _clearance >= this->minimumContactClearanceM &&
        _clearance <= this->maximumContactClearanceM;
  }

  private: void DiscoverCells(gz::sim::EntityComponentManager &_ecm)
  {
    std::vector<Cell> found;
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Visual>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Visual *)
        {
          const auto &name = _name->Data();
          const bool cleanableVisual =
              name.rfind("leaf_", 0) == 0 ||
              name.rfind("dust_mottle_", 0) == 0 ||
              name.rfind("puddle_lobe_", 0) == 0;
          if (!cleanableVisual)
            return true;
          const auto scoped = gz::sim::scopedName(_entity, _ecm, "/", false);
          if (scoped.find("surface_") == std::string::npos)
            return true;
          const auto pose = gz::sim::worldPose(_entity, _ecm);
          found.push_back({_entity, {pose.Pos().X(), pose.Pos().Y()}, false});
          return true;
        });
    if (!found.empty())
    {
      this->cells = std::move(found);
      this->cellLayoutReady = true;
    }
  }

  private: void CleanCircularSweep(
      gz::sim::EntityComponentManager &_ecm,
      const SweepState &_previous,
      const SweepState &_current)
  {
    const auto start = _previous.valid ? _previous.centre : _current.centre;
    for (auto &cell : this->cells)
    {
      if (!cell.cleaned && DistanceToSegment(
          cell.centre, start, _current.centre) <= this->sideBrushRadiusM)
        this->CleanCell(_ecm, cell);
    }
  }

  private: void CleanRollerSweep(
      gz::sim::EntityComponentManager &_ecm,
      const SweepState &_previous,
      const SweepState &_current)
  {
    const auto start = _previous.valid ? _previous.centre : _current.centre;
    const double distance = (_current.centre - start).Length();
    const int sampleCount = std::max(
        1, static_cast<int>(std::ceil(distance / this->sweepSampleSpacingM)));
    for (int sample = 0; sample <= sampleCount; ++sample)
    {
      const double fraction = static_cast<double>(sample) / sampleCount;
      const auto centre = start + (_current.centre - start) * fraction;
      const double yaw = _previous.valid
          ? _previous.yaw + (_current.yaw - _previous.yaw) * fraction
          : _current.yaw;
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      for (auto &cell : this->cells)
      {
        if (cell.cleaned)
          continue;
        const auto delta = cell.centre - centre;
        const double localX = c * delta.X() + s * delta.Y();
        const double localY = -s * delta.X() + c * delta.Y();
        if (std::abs(localX) <= this->rollerRadiusM &&
            std::abs(localY) <= this->rollerWidthM * 0.5)
          this->CleanCell(_ecm, cell);
      }
    }
  }

  private: void CleanCell(
      gz::sim::EntityComponentManager &_ecm, Cell &_cell)
  {
    _cell.cleaned = true;
    auto *component =
        _ecm.Component<gz::sim::components::Transparency>(_cell.visual);
    if (component == nullptr)
      _ecm.CreateComponent(
          _cell.visual, gz::sim::components::Transparency(1.0f));
    else
      component->Data() = 1.0f;
    _ecm.SetChanged(_cell.visual,
        gz::sim::components::Transparency::typeId,
        gz::sim::ComponentState::OneTimeChange);
  }

  private: void PublishState()
  {
    const std::size_t cleanedCells = std::count_if(
        this->cells.begin(), this->cells.end(),
        [](const Cell &_cell) {return _cell.cleaned;});
    const double cellArea = this->cellAreaM2;
    const double initialArea = this->cells.size() * cellArea;
    const double cleanedArea = cleanedCells * cellArea;
    const double remainingArea = (this->cells.size() - cleanedCells) * cellArea;
    const double cleanedFraction = initialArea > 0.0
        ? cleanedArea / initialArea : 0.0;
    const double balanceError = std::abs(
        initialArea - cleanedArea - remainingArea);
    this->PublishDouble(this->initialAreaPublisher, initialArea);
    this->PublishDouble(this->cleanedAreaPublisher, cleanedArea);
    this->PublishDouble(this->remainingAreaPublisher, remainingArea);
    this->PublishDouble(this->coveragePublisher, cleanedFraction);

    std::ostringstream stream;
    stream << std::boolalpha << std::fixed << std::setprecision(9)
        << "{\"enabled\":" << this->enabled.load()
        << ",\"cell_layout_ready\":" << this->cellLayoutReady
        << ",\"cell_count\":" << this->cells.size()
        << ",\"cleaned_cell_count\":" << cleanedCells
        << ",\"initial_area_m2\":" << initialArea
        << ",\"cleaned_area_m2\":" << cleanedArea
        << ",\"remaining_area_m2\":" << remainingArea
        << ",\"cleaned_fraction\":" << cleanedFraction
        << ",\"area_balance_error_m2\":" << balanceError
        << ",\"lift_position_m\":" << this->liftPositionM
        << ",\"left_velocity_rad_s\":" << this->leftVelocityRadS
        << ",\"right_velocity_rad_s\":" << this->rightVelocityRadS
        << ",\"roller_velocity_rad_s\":" << this->rollerVelocityRadS
        << ",\"left_ready\":" << this->leftReady
        << ",\"right_ready\":" << this->rightReady
        << ",\"roller_ready\":" << this->rollerReady
        << ",\"left_clearance_m\":" << this->leftClearanceM
        << ",\"right_clearance_m\":" << this->rightClearanceM
        << ",\"roller_clearance_m\":" << this->rollerClearanceM
        << ",\"left_world_x\":" << this->leftWorld.centre.X()
        << ",\"left_world_y\":" << this->leftWorld.centre.Y()
        << ",\"right_world_x\":" << this->rightWorld.centre.X()
        << ",\"right_world_y\":" << this->rightWorld.centre.Y()
        << ",\"roller_world_x\":" << this->rollerWorld.centre.X()
        << ",\"roller_world_y\":" << this->rollerWorld.centre.Y()
        << ",\"rigid_litter_entities_modified\":0}";
    gz::msgs::StringMsg status;
    status.set_data(stream.str());
    this->statusPublisher.Publish(status);
  }

  private: static void PublishDouble(
      gz::transport::Node::Publisher &_publisher, const double _value)
  {
    gz::msgs::Double message;
    message.set_data(_value);
    _publisher.Publish(message);
  }

  private: void OnEnable(const gz::msgs::Boolean &_message)
  {
    this->enabled.store(_message.data());
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher initialAreaPublisher;
  private: gz::transport::Node::Publisher cleanedAreaPublisher;
  private: gz::transport::Node::Publisher remainingAreaPublisher;
  private: gz::transport::Node::Publisher coveragePublisher;
  private: gz::transport::Node::Publisher statusPublisher;
  private: gz::sim::Entity leftBrushJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rightBrushJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity rollerJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity liftJoint{gz::sim::kNullEntity};
  private: gz::sim::Entity leftBrushLink{gz::sim::kNullEntity};
  private: gz::sim::Entity rightBrushLink{gz::sim::kNullEntity};
  private: gz::sim::Entity rollerLink{gz::sim::kNullEntity};
  private: std::vector<Cell> cells;
  private: SweepState previousLeft;
  private: SweepState previousRight;
  private: SweepState previousRoller;
  private: SweepState leftWorld;
  private: SweepState rightWorld;
  private: SweepState rollerWorld;
  private: std::atomic<bool> enabled{true};
  private: bool cellLayoutReady{false};
  private: bool leftReady{false};
  private: bool rightReady{false};
  private: bool rollerReady{false};
  private: double publishAccumulator{0.0};
  private: double liftPositionM{0.0};
  private: double leftVelocityRadS{0.0};
  private: double rightVelocityRadS{0.0};
  private: double rollerVelocityRadS{0.0};
  private: double leftClearanceM{0.0};
  private: double rightClearanceM{0.0};
  private: double rollerClearanceM{0.0};
  private: double cellAreaM2{0.01};
  private: double sweepSampleSpacingM{0.05};
  private: double minimumRotationRadS{2.0};
  private: double minimumLiftPositionM{0.095};
  private: double sideBrushRadiusM{0.15};
  private: double sideBrushLinkToGroundM{0.083};
  private: double rollerRadiusM{0.10};
  private: double rollerWidthM{0.62};
  private: double minimumContactClearanceM{-0.004};
  private: double maximumContactClearanceM{0.015};
  private: double publishRateHz{20.0};
  private: const std::string stateRoot{
      "/model/tzcup_formal_sanitation_vehicle/ground_dirt"};
  private: const std::string enableTopic{stateRoot + "/command/enable"};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
    sanitation_gazebo_control::GroundDirtCleaningSystem,
    gz::sim::System,
    sanitation_gazebo_control::GroundDirtCleaningSystem::ISystemConfigure,
    sanitation_gazebo_control::GroundDirtCleaningSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_control::GroundDirtCleaningSystem,
    "sanitation_gazebo_control::GroundDirtCleaningSystem")
