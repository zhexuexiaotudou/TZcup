#include <gtest/gtest.h>
#include <gz/sim/EventManager.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include "../src/GroundDirtCleaningSystem.cc"

TEST(GroundDirtVisualEvent, RealBrushEventRetiresOnlySweptVisual)
{
  gz::sim::EntityComponentManager ecm;
  auto entity = [&](const std::string &name, gz::sim::Entity parent,
      const gz::math::Pose3d &pose)
  {
    auto id = ecm.CreateEntity();
    ecm.CreateComponent(id, gz::sim::components::Name(name));
    ecm.CreateComponent(id, gz::sim::components::Pose(pose));
    if (parent != gz::sim::kNullEntity)
    {
      ecm.CreateComponent(id, gz::sim::components::ParentEntity(parent));
      ecm.SetParentEntity(id, parent);
    }
    return id;
  };
  auto vehicle = entity("vehicle", gz::sim::kNullEntity, {});
  ecm.CreateComponent(vehicle, gz::sim::components::Model());
  std::vector<gz::sim::Entity> rotating;
  for (const auto &name : {"left_side_brush", "right_side_brush", "central_roller"})
  {
    auto joint = entity(std::string(name) + "_joint", vehicle, {});
    ecm.CreateComponent(joint, gz::sim::components::Joint());
    ecm.CreateComponent(joint, gz::sim::components::JointVelocity({0.0}));
    rotating.push_back(joint);
    const double z = std::string(name) == "central_roller" ? .10 : .078;
    auto link = entity(std::string(name) + "_link", vehicle, {0, 0, z, 0, 0, 0});
    ecm.CreateComponent(link, gz::sim::components::Link());
  }
  auto lift = entity("cleaning_lift_joint", vehicle, {});
  ecm.CreateComponent(lift, gz::sim::components::Joint());
  ecm.CreateComponent(lift, gz::sim::components::JointPosition({0.0}));
  auto surface = entity("surface_test", gz::sim::kNullEntity, {});
  ecm.CreateComponent(surface, gz::sim::components::Model());
  auto surfaceLink = entity("surface_link", surface, {});
  ecm.CreateComponent(surfaceLink, gz::sim::components::Link());
  auto hit = entity("leaf_hit", surfaceLink, {.05, .05, .002, 0, 0, 0});
  auto miss = entity("leaf_miss", surfaceLink, {3, .05, .002, 0, 0, 0});
  auto decor = entity("decoration", surfaceLink, {.05, .05, .002, 0, 0, 0});
  for (auto id : {hit, miss, decor})
    ecm.CreateComponent(id, gz::sim::components::Visual());
  auto litter = entity("rigid_litter", gz::sim::kNullEntity, {});
  ecm.CreateComponent(litter, gz::sim::components::Model());
  sanitation_gazebo_control::GroundDirtCleaningSystem system;
  gz::sim::EventManager events;
  auto config = std::make_shared<sdf::Element>();
  system.Configure(vehicle, config, ecm, events);
  gz::sim::UpdateInfo info;
  info.dt = std::chrono::milliseconds(1);
  info.simTime = info.dt;
  info.paused = false;
  system.PreUpdate(info, ecm);
  EXPECT_FALSE(ecm.HasEntitiesMarkedForRemoval());
  ecm.Component<gz::sim::components::JointPosition>(lift)->Data() = {.1};
  system.PreUpdate(info, ecm);
  EXPECT_FALSE(ecm.HasEntitiesMarkedForRemoval());
  for (auto joint : rotating)
    ecm.Component<gz::sim::components::JointVelocity>(joint)->Data() = {8.0};
  system.PreUpdate(info, ecm);
  ASSERT_TRUE(ecm.HasEntitiesMarkedForRemoval());
  ecm.ProcessRemoveEntityRequests();
  EXPECT_FALSE(ecm.HasEntity(hit));
  for (auto id : {miss, decor, surfaceLink, surface, litter, vehicle})
    EXPECT_TRUE(ecm.HasEntity(id));
  // The ledger must not rediscover/erase surviving unswept cells next update.
  system.PreUpdate(info, ecm);
  EXPECT_FALSE(ecm.HasEntitiesMarkedForRemoval());
  EXPECT_TRUE(ecm.HasEntity(miss));
}
