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
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Transparency.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/transport/Node.hh>

#include "sanitation_gazebo_control/WaterRecoveryPoseCore.hh"

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
    this->frontLeftWheel = model.JointByName(_ecm, "front_left_wheel_joint");
    this->frontRightWheel = model.JointByName(_ecm, "front_right_wheel_joint");
    this->rearLeftWheel = model.JointByName(_ecm, "rear_left_wheel_joint");
    this->rearRightWheel = model.JointByName(_ecm, "rear_right_wheel_joint");
    const auto baseLink = model.LinkByName(_ecm, "base_link");
    const auto baseFootprint = model.LinkByName(_ecm, "base_footprint");
    this->basePoseSource = SelectBasePoseSource(
        baseLink != gz::sim::kNullEntity,
        baseFootprint != gz::sim::kNullEntity,
        this->modelEntity != gz::sim::kNullEntity);
    switch (this->basePoseSource)
    {
      case BasePoseSource::kBaseLink:
        this->basePoseEntity = baseLink;
        break;
      case BasePoseSource::kBaseFootprint:
        this->basePoseEntity = baseFootprint;
        break;
      case BasePoseSource::kModelEntity:
        this->basePoseEntity = this->modelEntity;
        break;
      case BasePoseSource::kUnavailable:
      default:
        this->basePoseEntity = gz::sim::kNullEntity;
        break;
    }
    this->squeegeeLink = model.LinkByName(_ecm, "squeegee_link");
    // URDF fixed-joint reduction lumps the physical nozzle collision into
    // squeegee_link.  A preembedded contact sensor may recreate a sensor-only
    // link with the original nozzle name; never use that holder as geometry.
    this->nozzle = this->squeegeeLink;

    for (const auto entity : {this->leftBrush, this->rightBrush,
        this->roller, this->pump, this->frontLeftWheel,
        this->frontRightWheel, this->rearLeftWheel, this->rearRightWheel})
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
    this->ReadDouble(_sdf, "minimum_lift_position_m",
        this->minimumLiftPositionM);
    this->ReadDouble(_sdf, "filter_trip_pressure_kpa",
        this->filterTripPressureKpa);
    this->ReadDouble(_sdf, "filter_clean_pressure_kpa",
        this->filterCleanPressureKpa);
    this->ReadDouble(_sdf, "tank_low_probe_fraction",
        this->tankLowProbeFraction);
    this->ReadDouble(_sdf, "tank_high_probe_fraction",
        this->tankHighProbeFraction);
    this->ReadDouble(_sdf, "sensor_time_constant_s",
        this->sensorTimeConstantS);
    this->ReadDouble(_sdf, "service_drain_rate_l_min",
        this->serviceDrainRateLMin);
    this->ReadDouble(_sdf, "service_drain_command_timeout_s",
        this->serviceDrainCommandTimeoutS);
    this->BuildCells(this->initialGroundVolumeL);
    this->FindWaterVisuals(_ecm);
    this->UpdateWaterVisuals(_ecm);
    this->initialTankMassKg = this->tankMassKg;
    this->sensedTankLevelFraction = this->tankCapacityKg > 0.0
        ? this->tankMassKg / this->tankCapacityKg : 0.0;

    this->node.Subscribe(this->enableTopic,
        &WaterRecoverySystem::OnEnable, this);
    this->node.Subscribe(this->resetGroundTopic,
        &WaterRecoverySystem::OnResetGround, this);
    this->node.Subscribe(this->resetTankTopic,
        &WaterRecoverySystem::OnResetTank, this);
    this->node.Subscribe(this->filterBlockageCommandTopic,
        &WaterRecoverySystem::OnFilterBlockage, this);
    this->node.Subscribe(this->drainCommandTopic,
        &WaterRecoverySystem::OnDrainCommand, this);

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
    this->sensedFlowPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/sensed_flow_l_min");
    this->sensedLevelPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/sensed_tank_level_fraction");
    this->filterPressurePublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/filter_differential_pressure_kpa");
    this->filterBlockagePublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/filter_blockage_fraction");
    this->pumpCurrentPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/pump_current_a");
    this->lowProbePublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/tank_low_probe_wet");
    this->highProbePublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/tank_high_probe_wet");
    this->filterFaultPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/filter_protection_active");
    this->drainOpenPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/service_drain_open");
    this->drainPermittedPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/service_drain_permitted");
    this->drainedVolumePublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/service_drained_volume_l");
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
    const double simTimeS = std::chrono::duration<double>(_info.simTime).count();
    const auto drainRevision = this->drainCommandRevision.load();
    if (drainRevision != this->appliedDrainCommandRevision)
    {
      this->lastDrainCommandSimTimeS = simTimeS;
      this->appliedDrainCommandRevision = drainRevision;
    }
    const double drainCommandAgeS = simTimeS - this->lastDrainCommandSimTimeS;
    this->drainCommandFresh = this->lastDrainCommandSimTimeS >= 0.0 &&
        drainCommandAgeS >= 0.0 &&
        drainCommandAgeS <= this->serviceDrainCommandTimeoutS;
    if (!this->drainCommandFresh)
      this->serviceDrainRequestedOpen.store(false);
    const double leftVelocity = std::abs(FirstValue(_ecm, this->leftBrush, true));
    const double rightVelocity = std::abs(FirstValue(_ecm, this->rightBrush, true));
    const double rollerVelocity = std::abs(FirstValue(_ecm, this->roller, true));
    const double pumpVelocity = std::abs(FirstValue(_ecm, this->pump, true));
    this->liftPositionM = FirstValue(_ecm, this->lift, false);
    this->squeegeeFloatPositionM = FirstValue(
        _ecm, this->squeegeeFloat, false);
    this->squeegeePitchPositionRad = FirstValue(
        _ecm, this->squeegeePitch, false);
    const double maximumWheelVelocity = std::max({
        std::abs(FirstValue(_ecm, this->frontLeftWheel, true)),
        std::abs(FirstValue(_ecm, this->frontRightWheel, true)),
        std::abs(FirstValue(_ecm, this->rearLeftWheel, true)),
        std::abs(FirstValue(_ecm, this->rearRightWheel, true))});

    this->serviceDrainPermitted = this->serviceDrainRequestedOpen.load() &&
        !this->enabled.load() && pumpVelocity < this->minimumPumpRadS &&
        leftVelocity < this->minimumBrushRadS &&
        rightVelocity < this->minimumBrushRadS &&
        rollerVelocity < this->minimumBrushRadS &&
        maximumWheelVelocity < this->maximumDrainWheelRadS;
    // Publish and apply the physical valve state, not the raw request.  An
    // unsafe request therefore remains observable below but cannot make the
    // outlet appear open or transfer liquid while recovery is active.
    this->serviceDrainOpen = this->serviceDrainPermitted;
    if (this->serviceDrainPermitted && this->tankMassKg > 0.0)
    {
      const double drainedL = std::min(
          this->serviceDrainRateLMin / 60.0 * dt,
          this->tankMassKg / this->waterDensityKgL);
      this->tankMassKg -= drainedL * this->waterDensityKgL;
      this->cumulativeServiceDrainedL += drainedL;
      this->PublishPayload();
    }

    this->brushReady = leftVelocity >= this->minimumBrushRadS &&
        rightVelocity >= this->minimumBrushRadS &&
        rollerVelocity >= this->minimumBrushRadS;
    this->pumpReady = pumpVelocity >= this->minimumPumpRadS;
    const bool squeegeeJointsReady =
        this->liftPositionM >= this->minimumLiftPositionM &&
        std::abs(this->squeegeeFloatPositionM) <= this->maximumFloatTravelM &&
        std::abs(this->squeegeePitchPositionRad) <= this->maximumPitchRad;

    this->basePoseAvailable = this->basePoseEntity != gz::sim::kNullEntity &&
        _ecm.Component<gz::sim::components::Pose>(this->basePoseEntity) != nullptr;
    if (this->basePoseAvailable)
    {
      const auto basePose = gz::sim::worldPose(this->basePoseEntity, _ecm);
      this->baseWorldZM = basePose.Pos().Z();
      this->baseWorldRollRad = basePose.Rot().Roll();
      this->baseWorldPitchRad = basePose.Rot().Pitch();
    }

    gz::math::Pose3d nozzlePose;
    if (this->nozzle != gz::sim::kNullEntity)
      nozzlePose = gz::sim::worldPose(this->nozzle, _ecm);
    if (this->nozzle != gz::sim::kNullEntity)
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

    const double requestedBlockage = std::clamp(
        this->filterBlockageFraction.load(), 0.0, 1.0);
    this->filterDifferentialPressureKpa = this->filterCleanPressureKpa +
        (this->filterTripPressureKpa * 1.25 - this->filterCleanPressureKpa) *
        requestedBlockage * requestedBlockage;
    this->filterProtectionActive =
        this->filterDifferentialPressureKpa >= this->filterTripPressureKpa;

    double recoveredThisStepL = 0.0;
    if (this->enabled.load() && this->brushReady && this->squeegeeReady &&
        this->nozzleReady && this->pumpReady && !this->tankFull &&
        !this->filterProtectionActive &&
        !this->serviceDrainOpen && this->basePoseAvailable &&
        this->visualLayoutReady)
    {
      const double pumpLimitL = this->pumpRatedFlowLMin /
          60.0 * this->hydraulicDerating *
          std::max(0.0, 1.0 - requestedBlockage * requestedBlockage) * dt;
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
    const double sensorAlpha = this->sensorTimeConstantS <= 0.0
        ? 1.0 : 1.0 - std::exp(-dt / this->sensorTimeConstantS);
    this->sensedFlowLMin += sensorAlpha *
        (this->instantaneousFlowLMin - this->sensedFlowLMin);
    const double trueLevel = this->tankCapacityKg > 0.0
        ? this->tankMassKg / this->tankCapacityKg : 0.0;
    this->sensedTankLevelFraction += sensorAlpha *
        (trueLevel - this->sensedTankLevelFraction);
    this->tankLowProbeWet = trueLevel >= this->tankLowProbeFraction;
    this->tankHighProbeWet = trueLevel >= this->tankHighProbeFraction;
    const double normalizedPumpSpeed = std::clamp(
        pumpVelocity / this->ratedPumpRadS, 0.0, 1.25);
    this->pumpCurrentA = this->pumpReady
        ? this->pumpNoLoadCurrentA * normalizedPumpSpeed +
            this->pumpHydraulicCurrentA * normalizedPumpSpeed *
            (0.25 + 0.75 * requestedBlockage)
        : 0.0;
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
      this->cumulativeServiceDrainedL = 0.0;
      this->sensedTankLevelFraction = this->tankCapacityKg > 0.0
          ? this->tankMassKg / this->tankCapacityKg : 0.0;
      this->sensedFlowLMin = 0.0;
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
    const double tankGainKg = this->tankMassKg - this->initialTankMassKg +
        this->cumulativeServiceDrainedL * this->waterDensityKgL;
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
    this->PublishDouble(this->sensedFlowPublisher, this->sensedFlowLMin);
    this->PublishDouble(this->sensedLevelPublisher,
        this->sensedTankLevelFraction);
    this->PublishDouble(this->filterPressurePublisher,
        this->filterDifferentialPressureKpa);
    this->PublishDouble(this->filterBlockagePublisher,
        std::clamp(this->filterBlockageFraction.load(), 0.0, 1.0));
    this->PublishDouble(this->pumpCurrentPublisher, this->pumpCurrentA);
    this->PublishDouble(this->drainedVolumePublisher,
        this->cumulativeServiceDrainedL);
    gz::msgs::Boolean full;
    full.set_data(this->tankFull);
    this->fullPublisher.Publish(full);
    gz::msgs::Boolean lowProbe;
    lowProbe.set_data(this->tankLowProbeWet);
    this->lowProbePublisher.Publish(lowProbe);
    gz::msgs::Boolean highProbe;
    highProbe.set_data(this->tankHighProbeWet);
    this->highProbePublisher.Publish(highProbe);
    gz::msgs::Boolean filterFault;
    filterFault.set_data(this->filterProtectionActive);
    this->filterFaultPublisher.Publish(filterFault);
    gz::msgs::Boolean drainOpen;
    drainOpen.set_data(this->serviceDrainOpen);
    this->drainOpenPublisher.Publish(drainOpen);
    gz::msgs::Boolean drainPermitted;
    drainPermitted.set_data(this->serviceDrainPermitted);
    this->drainPermittedPublisher.Publish(drainPermitted);

    std::ostringstream stream;
    stream << std::boolalpha << std::fixed << std::setprecision(9)
        << "{\"enabled\":" << this->enabled.load()
        << ",\"brush_ready\":" << this->brushReady
        << ",\"squeegee_ready\":" << this->squeegeeReady
        << ",\"nozzle_ready\":" << this->nozzleReady
        << ",\"cleaning_lift_position_m\":" << this->liftPositionM
        << ",\"squeegee_float_position_m\":"
        << this->squeegeeFloatPositionM
        << ",\"squeegee_pitch_position_rad\":"
        << this->squeegeePitchPositionRad
        << ",\"base_pose_available\":" << this->basePoseAvailable
        << ",\"base_pose_source\":\""
        << BasePoseSourceName(this->basePoseSource) << "\"";
    if (this->basePoseAvailable)
    {
      stream << ",\"base_world_z_m\":" << this->baseWorldZM
          << ",\"base_world_roll_rad\":" << this->baseWorldRollRad
          << ",\"base_world_pitch_rad\":" << this->baseWorldPitchRad;
    }
    else
    {
      stream << ",\"base_world_z_m\":null"
          << ",\"base_world_roll_rad\":null"
          << ",\"base_world_pitch_rad\":null";
    }
    stream
        << ",\"nozzle_height_m\":" << this->nozzleHeightM
        << ",\"intake_clearance_m\":" << this->intakeClearanceM
        << ",\"squeegee_blade_clearance_m\":" << this->squeegeeClearanceM
        << ",\"nozzle_world_x\":" << this->nozzleWorldX
        << ",\"nozzle_world_y\":" << this->nozzleWorldY
        << ",\"pump_ready\":" << this->pumpReady
        << ",\"pump_current_a\":" << this->pumpCurrentA
        << ",\"filter_blockage_fraction\":"
        << std::clamp(this->filterBlockageFraction.load(), 0.0, 1.0)
        << ",\"filter_differential_pressure_kpa\":"
        << this->filterDifferentialPressureKpa
        << ",\"filter_protection_active\":"
        << this->filterProtectionActive
        << ",\"service_drain_requested_open\":"
        << this->serviceDrainRequestedOpen.load()
        << ",\"service_drain_open\":" << this->serviceDrainOpen
        << ",\"service_drain_command_fresh\":" << this->drainCommandFresh
        << ",\"service_drain_permitted\":" << this->serviceDrainPermitted
        << ",\"service_drained_volume_l\":"
        << this->cumulativeServiceDrainedL
        << ",\"tank_full\":" << this->tankFull
        << ",\"tank_low_probe_wet\":" << this->tankLowProbeWet
        << ",\"tank_high_probe_wet\":" << this->tankHighProbeWet
        << ",\"ground_volume_l\":" << groundL
        << ",\"tank_mass_kg\":" << this->tankMassKg
        << ",\"flow_l_min\":" << this->instantaneousFlowLMin
        << ",\"sensed_flow_l_min\":" << this->sensedFlowLMin
        << ",\"sensed_tank_level_fraction\":"
        << this->sensedTankLevelFraction
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

  private: void OnFilterBlockage(const gz::msgs::Double &_message)
  {
    this->filterBlockageFraction.store(
        std::clamp(_message.data(), 0.0, 1.0));
  }

  private: void OnDrainCommand(const gz::msgs::Boolean &_message)
  {
    this->serviceDrainRequestedOpen.store(_message.data());
    this->drainCommandRevision.fetch_add(1);
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
  private: gz::transport::Node::Publisher sensedFlowPublisher;
  private: gz::transport::Node::Publisher sensedLevelPublisher;
  private: gz::transport::Node::Publisher filterPressurePublisher;
  private: gz::transport::Node::Publisher filterBlockagePublisher;
  private: gz::transport::Node::Publisher pumpCurrentPublisher;
  private: gz::transport::Node::Publisher lowProbePublisher;
  private: gz::transport::Node::Publisher highProbePublisher;
  private: gz::transport::Node::Publisher filterFaultPublisher;
  private: gz::transport::Node::Publisher drainOpenPublisher;
  private: gz::transport::Node::Publisher drainPermittedPublisher;
  private: gz::transport::Node::Publisher drainedVolumePublisher;
  private: gz::transport::Node::Publisher payloadPublisher;
  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity leftBrush{gz::sim::kNullEntity};
  private: gz::sim::Entity rightBrush{gz::sim::kNullEntity};
  private: gz::sim::Entity roller{gz::sim::kNullEntity};
  private: gz::sim::Entity lift{gz::sim::kNullEntity};
  private: gz::sim::Entity squeegeeFloat{gz::sim::kNullEntity};
  private: gz::sim::Entity squeegeePitch{gz::sim::kNullEntity};
  private: gz::sim::Entity pump{gz::sim::kNullEntity};
  private: gz::sim::Entity frontLeftWheel{gz::sim::kNullEntity};
  private: gz::sim::Entity frontRightWheel{gz::sim::kNullEntity};
  private: gz::sim::Entity rearLeftWheel{gz::sim::kNullEntity};
  private: gz::sim::Entity rearRightWheel{gz::sim::kNullEntity};
  private: gz::sim::Entity basePoseEntity{gz::sim::kNullEntity};
  private: BasePoseSource basePoseSource{BasePoseSource::kUnavailable};
  private: gz::sim::Entity squeegeeLink{gz::sim::kNullEntity};
  private: gz::sim::Entity nozzle{gz::sim::kNullEntity};
  private: std::vector<Cell> cells;
  private: std::vector<gz::sim::Entity> waterVisuals;
  private: int cellCountX{0};
  private: int cellCountY{0};
  private: std::atomic<bool> enabled{false};
  private: std::atomic<double> requestedGroundVolumeL{0.0};
  private: std::atomic<double> requestedTankMassKg{0.0};
  private: std::atomic<double> filterBlockageFraction{0.0};
  private: std::atomic<bool> serviceDrainRequestedOpen{false};
  private: std::atomic<unsigned long> drainCommandRevision{0};
  private: std::atomic<unsigned long> groundResetRevision{0};
  private: std::atomic<unsigned long> tankResetRevision{0};
  private: unsigned long appliedGroundResetRevision{0};
  private: unsigned long appliedTankResetRevision{0};
  private: unsigned long appliedDrainCommandRevision{0};
  private: bool brushReady{false};
  private: bool squeegeeReady{false};
  private: bool nozzleReady{false};
  private: bool pumpReady{false};
  private: bool tankFull{false};
  private: bool tankLowProbeWet{false};
  private: bool tankHighProbeWet{false};
  private: bool filterProtectionActive{false};
  private: bool serviceDrainPermitted{false};
  private: bool serviceDrainOpen{false};
  private: bool drainCommandFresh{false};
  private: bool visualLayoutReady{false};
  private: double tankMassKg{0.0};
  private: double initialTankMassKg{0.0};
  private: double groundReferenceL{0.0};
  private: double cumulativeRecoveredL{0.0};
  private: double instantaneousFlowLMin{0.0};
  private: double sensedFlowLMin{0.0};
  private: double sensedTankLevelFraction{0.0};
  private: double filterDifferentialPressureKpa{0.0};
  private: double pumpCurrentA{0.0};
  private: double cumulativeServiceDrainedL{0.0};
  private: double liftPositionM{0.0};
  private: double squeegeeFloatPositionM{0.0};
  private: double squeegeePitchPositionRad{0.0};
  private: double baseWorldZM{0.0};
  private: double baseWorldRollRad{0.0};
  private: double baseWorldPitchRad{0.0};
  private: bool basePoseAvailable{false};
  private: double nozzleHeightM{0.0};
  private: double intakeClearanceM{0.0};
  private: double squeegeeClearanceM{1e9};
  private: double nozzleWorldX{0.0};
  private: double nozzleWorldY{0.0};
  private: double publishAccumulator{0.0};
  private: double pumpRatedFlowLMin{15.1};
  private: double hydraulicDerating{0.70};
  private: double tankCapacityKg{8.30};
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
  private: double filterTripPressureKpa{35.0};
  private: double filterCleanPressureKpa{2.0};
  private: double tankLowProbeFraction{0.20};
  private: double tankHighProbeFraction{0.875};
  private: double sensorTimeConstantS{0.18};
  private: double serviceDrainRateLMin{12.0};
  private: double serviceDrainCommandTimeoutS{0.25};
  private: double lastDrainCommandSimTimeS{-1.0};
  private: const double minimumBrushRadS{2.0};
  private: const double minimumPumpRadS{10.0};
  private: const double maximumDrainWheelRadS{0.10};
  private: const double ratedPumpRadS{120.0};
  private: const double pumpNoLoadCurrentA{0.65};
  private: const double pumpHydraulicCurrentA{3.85};
  private: double minimumLiftPositionM{0.095};
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
  private: const std::string filterBlockageCommandTopic{
      stateRoot + "/command/filter_blockage_fraction"};
  private: const std::string drainCommandTopic{
      stateRoot + "/command/service_drain_open"};
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
