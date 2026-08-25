// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <gz/math/Pose3.hh>
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
}

/// Sparse 2.5-D wastewater recovery proxy for the formal vehicle.
///
/// Water is represented by finite rectangular cells with area, depth, volume
/// and mass.  Recovery is permitted only while the physical brush, lift,
/// squeegee/nozzle and pump joint states satisfy their operating envelope.
/// Pump throughput is bounded by the frozen Jabsco data-sheet flow and the
/// configured hydraulic derating.  Every litre removed from the ground is
/// added to the wastewater payload topic consumed by DynamicPayloadSystem.
class WaterRecoverySystem final:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  private: struct Cell
  {
    double x{0.0};
    double y{0.0};
    double volumeL{0.0};
  };

  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &) override
  {
    this->modelEntity = _entity;
    const gz::sim::Model model(_entity);
    this->leftBrush = model.JointByName(_ecm, "left_side_brush_joint");
    this->rightBrush = model.JointByName(_ecm, "right_side_brush_joint");
    this->roller = model.JointByName(_ecm, "central_roller_joint");
    this->lift = model.JointByName(_ecm, "cleaning_lift_joint");
    this->squeegeeFloat = model.JointByName(_ecm, "squeegee_float_joint");
    this->squeegeePitch = model.JointByName(_ecm, "squeegee_pitch_joint");
    this->pump = model.JointByName(_ecm, "recovery_pump_joint");
    this->squeegeeLink = model.LinkByName(_ecm, "squeegee_link");
    this->nozzle = model.LinkByName(_ecm, "suction_nozzle_link");
    // URDF fixed-joint reduction may lump the nozzle into squeegee_link.  The
    // 40 mm nozzle offset is then restored below when computing its envelope.
    if (this->nozzle == gz::sim::kNullEntity)
    {
      this->nozzle = this->squeegeeLink;
      this->nozzleUsesSqueegeeFrame = true;
    }

    for (const auto entity : {this->leftBrush, this->rightBrush,
        this->roller, this->pump})
    {
      if (entity != gz::sim::kNullEntity)
        gz::sim::enableComponent<gz::sim::components::JointVelocity>(
            _ecm, entity, true);
    }
    for (const auto entity : {this->lift, this->squeegeeFloat,
        this->squeegeePitch})
    {
      if (entity != gz::sim::kNullEntity)
        gz::sim::enableComponent<gz::sim::components::JointPosition>(
            _ecm, entity, true);
    }

    this->ReadDouble(_sdf, "pump_rated_flow_l_min", this->pumpRatedFlowLMin);
    this->ReadDouble(_sdf, "hydraulic_derating", this->hydraulicDerating);
    this->ReadDouble(_sdf, "tank_capacity_kg", this->tankCapacityKg);
    this->ReadDouble(_sdf, "initial_tank_mass_kg", this->tankMassKg);
    this->tankMassKg = std::clamp(
        this->tankMassKg, 0.0, this->tankCapacityKg);
    this->ReadDouble(_sdf, "initial_ground_volume_l", this->initialGroundVolumeL);
    this->ReadDouble(_sdf, "water_density_kg_l", this->waterDensityKgL);
    this->ReadDouble(_sdf, "patch_min_x", this->patchMinX);
    this->ReadDouble(_sdf, "patch_max_x", this->patchMaxX);
    this->ReadDouble(_sdf, "patch_min_y", this->patchMinY);
    this->ReadDouble(_sdf, "patch_max_y", this->patchMaxY);
    this->ReadDouble(_sdf, "cell_size_m", this->cellSizeM);
    this->ReadDouble(_sdf, "nozzle_length_m", this->nozzleLengthM);
    this->ReadDouble(_sdf, "nozzle_width_m", this->nozzleWidthM);
    this->ReadDouble(_sdf, "maximum_squeegee_clearance_m",
        this->maximumSqueegeeClearanceM);
    this->ReadDouble(_sdf, "maximum_intake_clearance_m",
        this->maximumIntakeClearanceM);
    this->BuildCells(this->initialGroundVolumeL);
    this->FindWaterVisuals(_ecm);
    this->UpdateWaterVisuals(_ecm);
    this->initialTankMassKg = this->tankMassKg;

    this->node.Subscribe(this->enableTopic,
        &WaterRecoverySystem::OnEnable, this);
    this->node.Subscribe(this->resetGroundTopic,
        &WaterRecoverySystem::OnResetGround, this);
    this->node.Subscribe(this->resetTankTopic,
        &WaterRecoverySystem::OnResetTank, this);

    this->groundPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/ground_volume_l");
    this->tankPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/tank_mass_kg");
    this->levelPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/tank_level_fraction");
    this->flowPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/flow_l_min");
    this->recoveredPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/recovered_volume_l");
    this->balancePublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/mass_balance_error_fraction");
    this->fullPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/tank_full");
    this->statusPublisher = this->node.Advertise<gz::msgs::StringMsg>(
        this->stateRoot + "/status_json");
    this->payloadPublisher = this->node.Advertise<gz::msgs::Double>(
        "/model/tzcup_formal_sanitation_vehicle/payload/wastewater_mass_kg");
    this->PublishPayload();
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt.count() <= 0)
      return;

    this->ApplyPendingResets(_ecm);
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double leftVelocity = std::abs(FirstValue(_ecm, this->leftBrush, true));
    const double rightVelocity = std::abs(FirstValue(_ecm, this->rightBrush, true));
    const double rollerVelocity = std::abs(FirstValue(_ecm, this->roller, true));
    const double pumpVelocity = std::abs(FirstValue(_ecm, this->pump, true));
    const double liftPosition = FirstValue(_ecm, this->lift, false);
    const double floatPosition = FirstValue(_ecm, this->squeegeeFloat, false);
    const double pitchPosition = FirstValue(_ecm, this->squeegeePitch, false);

    this->brushReady = leftVelocity >= this->minimumBrushRadS &&
        rightVelocity >= this->minimumBrushRadS &&
        rollerVelocity >= this->minimumBrushRadS;
    this->pumpReady = pumpVelocity >= this->minimumPumpRadS;
    const bool squeegeeJointsReady = liftPosition <= this->maximumLiftPositionM &&
        std::abs(floatPosition) <= this->maximumFloatTravelM &&
        std::abs(pitchPosition) <= this->maximumPitchRad;

    gz::math::Pose3d nozzlePose;
    if (this->nozzle != gz::sim::kNullEntity)
      nozzlePose = gz::sim::worldPose(this->nozzle, _ecm);
    if (this->nozzleUsesSqueegeeFrame)
      nozzlePose = nozzlePose * gz::math::Pose3d(0.040, 0, -0.005, 0, 0, 0);
    this->nozzleHeightM = nozzlePose.Pos().Z();
    this->nozzleWorldX = nozzlePose.Pos().X();
    this->nozzleWorldY = nozzlePose.Pos().Y();
    this->intakeClearanceM = nozzlePose.Pos().Z() - this->nozzleHalfHeightM;

    gz::math::Pose3d squeegeePose;
    if (this->squeegeeLink != gz::sim::kNullEntity)
      squeegeePose = gz::sim::worldPose(this->squeegeeLink, _ecm);
    this->squeegeeClearanceM = 1e9;
    for (const double x : {-this->squeegeeHalfLengthM, this->squeegeeHalfLengthM})
    {
      const auto point = squeegeePose.Pos() + squeegeePose.Rot().RotateVector(
          gz::math::Vector3d(x, 0.0, -this->squeegeeHalfHeightM));
      this->squeegeeClearanceM = std::min(this->squeegeeClearanceM, point.Z());
    }
    this->squeegeeReady = squeegeeJointsReady &&
        this->squeegeeClearanceM >= this->minimumSqueegeeClearanceM &&
        this->squeegeeClearanceM <= this->maximumSqueegeeClearanceM;
    this->nozzleReady = this->nozzle != gz::sim::kNullEntity &&
        this->intakeClearanceM >= this->minimumIntakeClearanceM &&
        this->intakeClearanceM <= this->maximumIntakeClearanceM;
    this->tankFull = this->tankMassKg >= this->tankCapacityKg - 1e-9;

    double recoveredThisStepL = 0.0;
    if (this->enabled.load() && this->brushReady && this->squeegeeReady &&
        this->nozzleReady && this->pumpReady && !this->tankFull &&
        this->visualLayoutReady)
    {
      const double pumpLimitL = this->pumpRatedFlowLMin /
          60.0 * this->hydraulicDerating * dt;
      const double tankLimitL = (this->tankCapacityKg - this->tankMassKg) /
          this->waterDensityKgL;
      double requestedL = std::min(pumpLimitL, tankLimitL);

      const double yaw = nozzlePose.Rot().Yaw();
      const double c = std::cos(yaw);
      const double s = std::sin(yaw);
      std::vector<std::size_t> candidates;
      double availableL = 0.0;
      for (std::size_t index = 0; index < this->cells.size(); ++index)
      {
        const auto &cell = this->cells[index];
        const double dx = cell.x - nozzlePose.Pos().X();
        const double dy = cell.y - nozzlePose.Pos().Y();
        const double localX = c * dx + s * dy;
        const double localY = -s * dx + c * dy;
        if (std::abs(localX) <= this->nozzleLengthM * 0.5 &&
            std::abs(localY) <= this->nozzleWidthM * 0.5 &&
            cell.volumeL > 0.0)
        {
          candidates.push_back(index);
          availableL += cell.volumeL;
        }
      }
      requestedL = std::min(requestedL, availableL);
      if (requestedL > 0.0)
      {
        double remainingL = requestedL;
        for (const auto index : candidates)
        {
          auto &cell = this->cells[index];
          const double fraction = availableL > 0.0
              ? cell.volumeL / availableL : 0.0;
          const double removedL = std::min(cell.volumeL,
              index == candidates.back() ? remainingL : requestedL * fraction);
          cell.volumeL -= removedL;
          remainingL -= removedL;
          recoveredThisStepL += removedL;
        }
        this->tankMassKg += recoveredThisStepL * this->waterDensityKgL;
        this->cumulativeRecoveredL += recoveredThisStepL;
        this->UpdateWaterVisuals(_ecm);
        this->PublishPayload();
      }
    }

    this->instantaneousFlowLMin = dt > 0.0
        ? recoveredThisStepL / dt * 60.0 : 0.0;
    this->tankFull = this->tankMassKg >= this->tankCapacityKg - 1e-9;
    this->publishAccumulator += dt;
    if (this->publishAccumulator >= this->publishPeriodS)
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

  private: void BuildCells(const double _requestedVolumeL)
  {
    this->cells.clear();
    this->cellCountX = std::max(1, static_cast<int>(
        std::ceil((this->patchMaxX - this->patchMinX) / this->cellSizeM)));
    this->cellCountY = std::max(1, static_cast<int>(
        std::ceil((this->patchMaxY - this->patchMinY) / this->cellSizeM)));
    const double perCellL = std::max(0.0, _requestedVolumeL) /
        static_cast<double>(this->cellCountX * this->cellCountY);
    for (int ix = 0; ix < this->cellCountX; ++ix)
    {
      for (int iy = 0; iy < this->cellCountY; ++iy)
      {
        this->cells.push_back({
            this->patchMinX + (ix + 0.5) *
                (this->patchMaxX - this->patchMinX) / this->cellCountX,
            this->patchMinY + (iy + 0.5) *
                (this->patchMaxY - this->patchMinY) / this->cellCountY,
            perCellL});
      }
    }
    this->groundReferenceL = std::max(0.0, _requestedVolumeL);
    this->cumulativeRecoveredL = 0.0;
  }

  private: double GroundVolumeL() const
  {
    double total = 0.0;
    for (const auto &cell : this->cells)
      total += cell.volumeL;
    return total;
  }

  private: void FindWaterVisuals(gz::sim::EntityComponentManager &_ecm)
  {
    this->waterVisuals.assign(this->cellCountX, gz::sim::kNullEntity);
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Visual>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Name *_name,
            const gz::sim::components::Visual *)
        {
          for (int ix = 0; ix < this->cellCountX; ++ix)
          {
            std::ostringstream expected;
            expected << "water_strip_" << std::setw(2) << std::setfill('0') << ix;
            const auto scoped = gz::sim::scopedName(_entity, _ecm, "/", false);
            if (_name->Data() == expected.str() &&
                scoped.find("formal_recoverable_water_patch") != std::string::npos)
              this->waterVisuals[ix] = _entity;
          }
          return true;
        });
    this->visualLayoutReady = std::count_if(
        this->waterVisuals.begin(), this->waterVisuals.end(),
        [](const gz::sim::Entity entity)
        {return entity != gz::sim::kNullEntity;}) == this->cellCountX;
  }

  private: void UpdateWaterVisuals(gz::sim::EntityComponentManager &_ecm)
  {
    if (this->waterVisuals.size() != static_cast<std::size_t>(this->cellCountX))
      return;
    for (int ix = 0; ix < this->cellCountX; ++ix)
    {
      double remainingL = 0.0;
      double referenceL = 0.0;
      for (int iy = 0; iy < this->cellCountY; ++iy)
      {
        remainingL += this->cells[ix * this->cellCountY + iy].volumeL;
        referenceL += this->groundReferenceL /
            static_cast<double>(this->cellCountX * this->cellCountY);
      }
      const float remainingFraction = referenceL > 0.0
          ? static_cast<float>(std::clamp(remainingL / referenceL, 0.0, 1.0))
          : 0.0f;
      const float transparency = 1.0f - 0.55f * remainingFraction;
      const auto entity = this->waterVisuals[ix];
      if (entity == gz::sim::kNullEntity)
        continue;
      auto *component = _ecm.Component<gz::sim::components::Transparency>(entity);
      if (component == nullptr)
        _ecm.CreateComponent(entity, gz::sim::components::Transparency(transparency));
      else
        component->Data() = transparency;
      _ecm.SetChanged(entity, gz::sim::components::Transparency::typeId,
          gz::sim::ComponentState::OneTimeChange);
    }
  }

  private: void ApplyPendingResets(gz::sim::EntityComponentManager &_ecm)
  {
    const auto groundRevision = this->groundResetRevision.load();
    if (groundRevision != this->appliedGroundResetRevision)
    {
      this->BuildCells(this->requestedGroundVolumeL.load());
      this->UpdateWaterVisuals(_ecm);
      this->appliedGroundResetRevision = groundRevision;
    }
    const auto tankRevision = this->tankResetRevision.load();
    if (tankRevision != this->appliedTankResetRevision)
    {
      this->tankMassKg = std::clamp(
          this->requestedTankMassKg.load(), 0.0, this->tankCapacityKg);
      this->initialTankMassKg = this->tankMassKg;
      this->cumulativeRecoveredL = 0.0;
      this->PublishPayload();
      this->appliedTankResetRevision = tankRevision;
    }
  }

  private: void PublishPayload()
  {
    gz::msgs::Double message;
    message.set_data(this->tankMassKg);
    this->payloadPublisher.Publish(message);
  }

  private: void PublishState()
  {
    const double groundL = this->GroundVolumeL();
    const double groundRemovedKg =
        (this->groundReferenceL - groundL) * this->waterDensityKgL;
    const double tankGainKg = this->tankMassKg - this->initialTankMassKg;
    const double balanceError = std::abs(groundRemovedKg) < 1e-9
        ? 0.0 : std::abs(groundRemovedKg - tankGainKg) /
            std::abs(groundRemovedKg);
    this->PublishDouble(this->groundPublisher, groundL);
    this->PublishDouble(this->tankPublisher, this->tankMassKg);
    this->PublishDouble(this->levelPublisher,
        this->tankMassKg / this->tankCapacityKg);
    this->PublishDouble(this->flowPublisher, this->instantaneousFlowLMin);
    this->PublishDouble(this->recoveredPublisher, this->cumulativeRecoveredL);
    this->PublishDouble(this->balancePublisher, balanceError);
    gz::msgs::Boolean full;
    full.set_data(this->tankFull);
    this->fullPublisher.Publish(full);

    std::ostringstream stream;
    stream << std::boolalpha << std::fixed << std::setprecision(9)
        << "{\"enabled\":" << this->enabled.load()
        << ",\"brush_ready\":" << this->brushReady
        << ",\"squeegee_ready\":" << this->squeegeeReady
        << ",\"nozzle_ready\":" << this->nozzleReady
        << ",\"nozzle_height_m\":" << this->nozzleHeightM
        << ",\"intake_clearance_m\":" << this->intakeClearanceM
        << ",\"squeegee_blade_clearance_m\":" << this->squeegeeClearanceM
        << ",\"nozzle_world_x\":" << this->nozzleWorldX
        << ",\"nozzle_world_y\":" << this->nozzleWorldY
        << ",\"pump_ready\":" << this->pumpReady
        << ",\"tank_full\":" << this->tankFull
        << ",\"ground_volume_l\":" << groundL
        << ",\"tank_mass_kg\":" << this->tankMassKg
        << ",\"flow_l_min\":" << this->instantaneousFlowLMin
        << ",\"recovered_volume_l\":" << this->cumulativeRecoveredL
        << ",\"visual_remaining_fraction\":"
        << (this->groundReferenceL > 0.0 ? groundL / this->groundReferenceL : 0.0)
        << ",\"water_visual_count\":" << std::count_if(
            this->waterVisuals.begin(), this->waterVisuals.end(),
            [](const gz::sim::Entity entity)
            {return entity != gz::sim::kNullEntity;})
        << ",\"visual_layout_ready\":" << this->visualLayoutReady
        << ",\"mass_balance_error_fraction\":" << balanceError << "}";
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

  private: void OnResetGround(const gz::msgs::Double &_message)
  {
    this->requestedGroundVolumeL.store(std::max(0.0, _message.data()));
    this->groundResetRevision.fetch_add(1);
  }

  private: void OnResetTank(const gz::msgs::Double &_message)
  {
    this->requestedTankMassKg.store(_message.data());
    this->tankResetRevision.fetch_add(1);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher groundPublisher;
  private: gz::transport::Node::Publisher tankPublisher;
  private: gz::transport::Node::Publisher levelPublisher;
  private: gz::transport::Node::Publisher flowPublisher;
  private: gz::transport::Node::Publisher recoveredPublisher;
  private: gz::transport::Node::Publisher balancePublisher;
  private: gz::transport::Node::Publisher fullPublisher;
  private: gz::transport::Node::Publisher statusPublisher;
  private: gz::transport::Node::Publisher payloadPublisher;
  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity leftBrush{gz::sim::kNullEntity};
  private: gz::sim::Entity rightBrush{gz::sim::kNullEntity};
  private: gz::sim::Entity roller{gz::sim::kNullEntity};
  private: gz::sim::Entity lift{gz::sim::kNullEntity};
  private: gz::sim::Entity squeegeeFloat{gz::sim::kNullEntity};
  private: gz::sim::Entity squeegeePitch{gz::sim::kNullEntity};
  private: gz::sim::Entity pump{gz::sim::kNullEntity};
  private: gz::sim::Entity squeegeeLink{gz::sim::kNullEntity};
  private: gz::sim::Entity nozzle{gz::sim::kNullEntity};
  private: std::vector<Cell> cells;
  private: std::vector<gz::sim::Entity> waterVisuals;
  private: int cellCountX{0};
  private: int cellCountY{0};
  private: std::atomic<bool> enabled{false};
  private: std::atomic<double> requestedGroundVolumeL{0.0};
  private: std::atomic<double> requestedTankMassKg{0.0};
  private: std::atomic<unsigned long> groundResetRevision{0};
  private: std::atomic<unsigned long> tankResetRevision{0};
  private: unsigned long appliedGroundResetRevision{0};
  private: unsigned long appliedTankResetRevision{0};
  private: bool brushReady{false};
  private: bool squeegeeReady{false};
  private: bool nozzleReady{false};
  private: bool nozzleUsesSqueegeeFrame{false};
  private: bool pumpReady{false};
  private: bool tankFull{false};
  private: bool visualLayoutReady{false};
  private: double tankMassKg{0.0};
  private: double initialTankMassKg{0.0};
  private: double groundReferenceL{0.0};
  private: double cumulativeRecoveredL{0.0};
  private: double instantaneousFlowLMin{0.0};
  private: double nozzleHeightM{0.0};
  private: double intakeClearanceM{0.0};
  private: double squeegeeClearanceM{1e9};
  private: double nozzleWorldX{0.0};
  private: double nozzleWorldY{0.0};
  private: double publishAccumulator{0.0};
  private: double pumpRatedFlowLMin{15.1};
  private: double hydraulicDerating{0.70};
  private: double tankCapacityKg{9.7064};
  private: double initialGroundVolumeL{2.88};
  private: double waterDensityKgL{1.0};
  private: double patchMinX{-0.60};
  private: double patchMaxX{1.80};
  private: double patchMinY{-0.30};
  private: double patchMaxY{0.30};
  private: double cellSizeM{0.10};
  private: double nozzleLengthM{0.14};
  private: double nozzleWidthM{0.65};
  private: double maximumSqueegeeClearanceM{0.012};
  private: double maximumIntakeClearanceM{0.012};
  private: const double minimumBrushRadS{2.0};
  private: const double minimumPumpRadS{10.0};
  private: const double maximumLiftPositionM{0.04};
  private: const double maximumFloatTravelM{0.0151};
  private: const double maximumPitchRad{0.1746};
  private: const double minimumSqueegeeClearanceM{-0.004};
  private: const double minimumIntakeClearanceM{-0.002};
  private: const double squeegeeHalfLengthM{0.0325};
  private: const double squeegeeHalfHeightM{0.0525};
  private: const double nozzleHalfHeightM{0.0425};
  private: const double publishPeriodS{0.05};
  private: const std::string stateRoot{
      "/model/tzcup_formal_sanitation_vehicle/water_recovery"};
  private: const std::string enableTopic{stateRoot + "/command/enable"};
  private: const std::string resetGroundTopic{
      stateRoot + "/command/reset_ground_volume_l"};
  private: const std::string resetTankTopic{
      stateRoot + "/command/reset_tank_mass_kg"};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
    sanitation_gazebo_control::WaterRecoverySystem,
    gz::sim::System,
    sanitation_gazebo_control::WaterRecoverySystem::ISystemConfigure,
    sanitation_gazebo_control::WaterRecoverySystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_control::WaterRecoverySystem,
    "sanitation_gazebo_control::WaterRecoverySystem")
