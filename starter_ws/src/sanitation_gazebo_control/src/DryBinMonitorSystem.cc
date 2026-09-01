// Copyright 2026 TZCup team
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace sanitation_gazebo_control
{
/// Observe real rigid bodies resting inside the formal vehicle dry bin.
///
/// Discrete litter is never deleted or converted into aggregate payload by
/// this system.  It remains a physical rigid body supported by the bin, so its
/// material-dependent inertial contributes to vehicle loading through contact.
/// This monitor only emulates the bin level / load instrumentation and reports
/// a fail-closed full state.
class DryBinMonitorSystem final:
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
    this->modelEntity = _entity;
    const gz::sim::Model vehicle(_entity);
    this->binEntity = vehicle.LinkByName(_ecm, "dry_bin_link");
    this->ReadDouble(_sdf, "usable_size_x_m", this->usableSizeX);
    this->ReadDouble(_sdf, "usable_size_y_m", this->usableSizeY);
    this->ReadDouble(_sdf, "usable_size_z_m", this->usableSizeZ);
    this->ReadDouble(_sdf, "cube_edge_m", this->cubeEdgeM);
    this->ReadDouble(
        _sdf, "bottom_contact_tolerance_m", this->bottomContactToleranceM);
    this->ReadDouble(_sdf, "mass_capacity_kg", this->massCapacityKg);
    this->ReadDouble(
        _sdf, "initial_aggregate_mass_kg", this->initialAggregateMassKg);
    if (_sdf->HasElement("dry_accounting_mode"))
      this->dryAccountingMode = _sdf->Get<std::string>("dry_accounting_mode");
    this->ReadDouble(_sdf, "level_full_fraction", this->levelFullFraction);
    this->ReadDouble(_sdf, "publish_rate_hz", this->publishRateHz);
    this->ReadDouble(_sdf, "bin_center_model_x_m", this->binCenterModelX);
    this->ReadDouble(_sdf, "bin_center_model_y_m", this->binCenterModelY);
    this->ReadDouble(_sdf, "bin_center_model_z_m", this->binCenterModelZ);
    if (!std::isfinite(this->bottomContactToleranceM) ||
        this->bottomContactToleranceM < 0.0 ||
        this->bottomContactToleranceM > 0.005)
    {
      throw std::invalid_argument(
          "bottom_contact_tolerance_m must be finite and within [0, 0.005]");
    }
    if (this->dryAccountingMode != "aggregate" &&
        this->dryAccountingMode != "physical_resident")
    {
      throw std::invalid_argument(
          "dry_accounting_mode must be aggregate or physical_resident");
    }
    if (this->dryAccountingMode == "physical_resident" &&
        std::abs(this->initialAggregateMassKg) > 1e-9)
    {
      throw std::invalid_argument(
          "physical_resident mode requires initial_aggregate_mass_kg == 0");
    }

    this->countPublisher = this->node.Advertise<gz::msgs::Int32>(
        this->stateRoot + "/contained_object_count");
    this->massPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/contained_mass_kg");
    this->levelPublisher = this->node.Advertise<gz::msgs::Double>(
        this->stateRoot + "/fill_level_fraction");
    this->fullPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/full");
    this->readyPublisher = this->node.Advertise<gz::msgs::Boolean>(
        this->stateRoot + "/sensor_ready");
    this->statusPublisher = this->node.Advertise<gz::msgs::StringMsg>(
        this->stateRoot + "/status_json");
    this->observedStatusPublisher = this->node.Advertise<gz::msgs::StringMsg>(
        this->stateRoot + "/observed_status_json");
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override
  {
    if (_info.paused || _info.dt.count() <= 0)
      return;
    this->publishAccumulator +=
        std::chrono::duration<double>(_info.dt).count();
    const double period = this->publishRateHz > 0.0
        ? 1.0 / this->publishRateHz : 0.1;
    if (this->publishAccumulator < period)
      return;
    this->publishAccumulator = 0.0;

    int count = 0;
    double containedMassKg = 0.0;
    double highestTopM = -this->usableSizeZ * 0.5;
    this->candidateModelCount = 0;
    this->insideCandidateCount = 0;
    this->inertialCandidateCount = 0;
    this->lastCandidateX = 0.0;
    this->lastCandidateY = 0.0;
    this->lastCandidateZ = 0.0;
    const bool sensorReady = this->modelEntity != gz::sim::kNullEntity;
    if (sensorReady)
    {
      // Fixed-joint reduction can retain a named child entity while lumping
      // its collisions and inertia into base_link.  In that state a non-null
      // LinkByName result is not a dependable payload-volume frame.  Compose
      // the exact FK-validated bin datum with the live vehicle model pose for
      // both reduced and non-reduced imports.
      const auto binWorld = gz::sim::worldPose(this->modelEntity, _ecm) *
          gz::math::Pose3d(
              this->binCenterModelX, this->binCenterModelY,
              this->binCenterModelZ, 0.0, 0.0, 0.0);
      _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_modelEntity,
              const gz::sim::components::Model *,
              const gz::sim::components::Name *_name)
          {
            const auto &name = _name->Data();
            if (name != "material_cube" && name.rfind("object_", 0) != 0)
              return true;
            ++this->candidateModelCount;
            const gz::sim::Model litter(_modelEntity);
            const auto links = litter.Links(_ecm);
            auto physicalLink = litter.CanonicalLink(_ecm);
            if (physicalLink == gz::sim::kNullEntity && !links.empty())
              physicalLink = links.front();
            // A model entity retains its spawn-frame pose when a detachable
            // joint moves the physical child link.  Containment must follow
            // the rigid body's live canonical/physical link, not that stale
            // model frame.
            const auto physicalEntity = physicalLink == gz::sim::kNullEntity
                ? _modelEntity : physicalLink;
            const auto relative = binWorld.Inverse() *
                gz::sim::worldPose(physicalEntity, _ecm);
            const auto &position = relative.Pos();
            this->lastCandidateX = position.X();
            this->lastCandidateY = position.Y();
            this->lastCandidateZ = position.Z();
            const double halfEdge = this->cubeEdgeM * 0.5;
            const bool inside =
                std::abs(position.X()) + halfEdge <= this->usableSizeX * 0.5 &&
                std::abs(position.Y()) + halfEdge <= this->usableSizeY * 0.5 &&
                // Bullet/DART contact stabilization permits millimetre-scale
                // penetration into the real 8 mm floor collision.  Apply the
                // measured, bounded tolerance only to the supporting bottom
                // plane; the side walls and upper usable-volume boundary stay
                // strict so an object below/alongside the bin cannot be counted.
                position.Z() - halfEdge >=
                    -this->usableSizeZ * 0.5 - this->bottomContactToleranceM &&
                position.Z() + halfEdge <= this->usableSizeZ * 0.5;
            if (!inside)
              return true;
            ++this->insideCandidateCount;

            auto inertialLink = physicalLink;
            const gz::sim::components::Inertial *inertial =
                inertialLink == gz::sim::kNullEntity
                ? nullptr
                : _ecm.Component<gz::sim::components::Inertial>(inertialLink);
            // URDF-spawned one-link models do not consistently expose a
            // CanonicalLink component through every ros_gz / sdformat path.
            // Fall back to the first physical child link that actually owns
            // an Inertial component; never substitute a configured material
            // mass, because the measured rigid-body mass is the acceptance
            // truth.
            if (inertial == nullptr)
            {
              for (const auto link : links)
              {
                inertial =
                    _ecm.Component<gz::sim::components::Inertial>(link);
                if (inertial != nullptr)
                  break;
              }
            }
            if (inertial == nullptr)
              return true;
            ++this->inertialCandidateCount;
            ++count;
            containedMassKg += inertial->Data().MassMatrix().Mass();
            highestTopM = std::max(highestTopM, position.Z() + halfEdge);
            return true;
          });
    }

    const double fillLevel = count == 0 ? 0.0 : std::clamp(
        (highestTopM + this->usableSizeZ * 0.5) / this->usableSizeZ,
        0.0, 1.0);
    const double totalContainedMassKg =
        this->initialAggregateMassKg + containedMassKg;
    const bool massFull =
        totalContainedMassKg >= this->massCapacityKg - 1e-9;
    const bool levelFull = fillLevel >= this->levelFullFraction;
    // Aggregate dry payload is valid only when there are no independently
    // retained bodies in this bin. Such bodies would create a second physical
    // mass representation, so make the monitor fail closed.
    const bool accountingValid =
        this->dryAccountingMode == "physical_resident" || count == 0;
    const bool full = !sensorReady || !accountingValid || massFull || levelFull;
    this->Publish(count, containedMassKg, totalContainedMassKg, fillLevel,
        full, sensorReady, massFull, levelFull, accountingValid);
  }

  private: void ReadDouble(
      const std::shared_ptr<const sdf::Element> &_sdf,
      const std::string &_name,
      double &_value)
  {
    if (_sdf->HasElement(_name))
      _value = _sdf->Get<double>(_name);
  }

  private: void Publish(
      const int _count,
      const double _physicalMass,
      const double _totalMass,
      const double _level,
      const bool _full,
      const bool _ready,
      const bool _massFull,
      const bool _levelFull,
      const bool _accountingValid)
  {
    gz::msgs::Int32 count;
    count.set_data(_count);
    this->countPublisher.Publish(count);
    gz::msgs::Double mass;
    mass.set_data(_totalMass);
    this->massPublisher.Publish(mass);
    gz::msgs::Double level;
    level.set_data(_level);
    this->levelPublisher.Publish(level);
    gz::msgs::Boolean full;
    full.set_data(_full);
    this->fullPublisher.Publish(full);
    gz::msgs::Boolean ready;
    ready.set_data(_ready);
    this->readyPublisher.Publish(ready);

    std::ostringstream stream;
    stream << std::boolalpha << std::fixed << std::setprecision(9)
        << "{\"sensor_ready\":" << _ready
        << ",\"dry_accounting_mode\":\"" << this->dryAccountingMode
        << "\",\"resident_load_path\":\""
        << (this->dryAccountingMode == "physical_resident"
            ? "independent_rigid_bodies_contact"
            : "aggregate_inertial_payload")
        << "\""
        << ",\"accounting_valid\":" << _accountingValid
        << ",\"fixed_joint_reduction_fallback\":"
        << true
        << ",\"candidate_model_count\":" << this->candidateModelCount
        << ",\"inside_candidate_count\":" << this->insideCandidateCount
        << ",\"inertial_candidate_count\":" << this->inertialCandidateCount
        << ",\"last_candidate_bin_xyz_m\":[" << this->lastCandidateX
        << "," << this->lastCandidateY << "," << this->lastCandidateZ << "]"
        << ",\"bottom_contact_tolerance_m\":"
        << this->bottomContactToleranceM
        << ",\"contained_object_count\":" << _count
        << ",\"resident_rigid_body_count\":" << _count
        << ",\"physical_contained_mass_kg\":" << _physicalMass
        << ",\"resident_rigid_body_mass_kg\":" << _physicalMass
        << ",\"initial_aggregate_mass_kg\":"
        << this->initialAggregateMassKg
        << ",\"contained_mass_kg\":" << _totalMass
        << ",\"fill_level_fraction\":" << _level
        << ",\"mass_capacity_kg\":" << this->massCapacityKg
        << ",\"mass_full\":" << _massFull
        << ",\"level_full\":" << _levelFull
        << ",\"full\":" << _full << "}";
    gz::msgs::StringMsg status;
    status.set_data(stream.str());
    this->statusPublisher.Publish(status);

    // Product-facing instrumentation intentionally omits candidate entity
    // counts, names and poses.  Those values remain evaluator-only in the
    // status_json stream above; the control node sees only sensor-equivalent
    // load / level observations.
    std::ostringstream observedStream;
    observedStream << std::boolalpha << std::fixed << std::setprecision(9)
        << "{\"sensor_ready\":" << _ready
        << ",\"dry_accounting_mode\":\"" << this->dryAccountingMode
        << "\",\"resident_load_path\":\""
        << (this->dryAccountingMode == "physical_resident"
            ? "independent_rigid_bodies_contact"
            : "aggregate_inertial_payload")
        << "\""
        << ",\"accounting_valid\":" << _accountingValid
        << ",\"contained_object_count\":" << _count
        << ",\"resident_rigid_body_count\":" << _count
        << ",\"contained_mass_kg\":" << _totalMass
        << ",\"resident_rigid_body_mass_kg\":" << _physicalMass
        << ",\"fill_level_fraction\":" << _level
        << ",\"mass_capacity_kg\":" << this->massCapacityKg
        << ",\"full\":" << _full << "}";
    gz::msgs::StringMsg observedStatus;
    observedStatus.set_data(observedStream.str());
    this->observedStatusPublisher.Publish(observedStatus);
  }

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher countPublisher;
  private: gz::transport::Node::Publisher massPublisher;
  private: gz::transport::Node::Publisher levelPublisher;
  private: gz::transport::Node::Publisher fullPublisher;
  private: gz::transport::Node::Publisher readyPublisher;
  private: gz::transport::Node::Publisher statusPublisher;
  private: gz::transport::Node::Publisher observedStatusPublisher;
  private: gz::sim::Entity binEntity{gz::sim::kNullEntity};
  private: gz::sim::Entity modelEntity{gz::sim::kNullEntity};
  private: double usableSizeX{0.485};
  private: double usableSizeY{0.355};
  private: double usableSizeZ{0.233};
  private: double cubeEdgeM{0.030};
  private: double bottomContactToleranceM{0.005};
  private: double massCapacityKg{1.512};
  private: double initialAggregateMassKg{0.0};
  private: std::string dryAccountingMode{"aggregate"};
  private: double levelFullFraction{0.95};
  private: double publishRateHz{10.0};
  private: double binCenterModelX{-0.205};
  private: double binCenterModelY{0.160};
  private: double binCenterModelZ{0.5656};
  private: int candidateModelCount{0};
  private: int insideCandidateCount{0};
  private: int inertialCandidateCount{0};
  private: double lastCandidateX{0.0};
  private: double lastCandidateY{0.0};
  private: double lastCandidateZ{0.0};
  private: double publishAccumulator{0.0};
  private: const std::string stateRoot{
      "/model/tzcup_formal_sanitation_vehicle/dry_bin"};
};
}  // namespace sanitation_gazebo_control

GZ_ADD_PLUGIN(
    sanitation_gazebo_control::DryBinMonitorSystem,
    gz::sim::System,
    sanitation_gazebo_control::DryBinMonitorSystem::ISystemConfigure,
    sanitation_gazebo_control::DryBinMonitorSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    sanitation_gazebo_control::DryBinMonitorSystem,
    "sanitation_gazebo_control::DryBinMonitorSystem")
