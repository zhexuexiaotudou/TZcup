cmake_minimum_required(VERSION 3.10)
project(sanitation_gazebo_control)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

# Gazebo Harmonic's generated message headers target Ubuntu's protobuf 3.x.
# Resolve the system module before ament's prefix chain can expose OR-Tools'
# private, newer protoc binary to CMake's package discovery.
if(UNIX AND EXISTS "/usr/bin/protoc")
  set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc" CACHE FILEPATH
      "System protoc matching Gazebo Harmonic protobuf" FORCE)
endif()
find_package(Protobuf REQUIRED MODULE)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
find_package(gz-gui8 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

set(SANITATION_MISSION_CONTROL_QML
  "${CMAKE_CURRENT_SOURCE_DIR}/SanitationMissionControl.qml"
)
configure_file(
  SanitationMissionControl.qrc.in
  "${CMAKE_CURRENT_BINARY_DIR}/SanitationMissionControl.qrc"
  @ONLY
)
qt5_add_resources(
  control_resources_RCC
  "${CMAKE_CURRENT_BINARY_DIR}/SanitationMissionControl.qrc"
)

add_library(SanitationMissionControl SHARED
  include/SanitationMissionControl.hh
  src/SanitationMissionControl.cc
  ${control_resources_RCC}
)
target_include_directories(SanitationMissionControl PRIVATE include)
ament_target_dependencies(SanitationMissionControl rclcpp std_msgs std_srvs)
target_link_libraries(SanitationMissionControl
  gz-gui8::gz-gui8
  Qt5::Core
  Qt5::Quick
  Qt5::QuickControls2
)

install(TARGETS SanitationMissionControl
  LIBRARY DESTINATION lib
)

add_library(DynamicPayloadSystem SHARED
  src/DynamicPayloadSystem.cc
)
target_link_libraries(DynamicPayloadSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS DynamicPayloadSystem
  LIBRARY DESTINATION lib
)

add_library(WaterRecoverySystem SHARED
  src/WaterRecoverySystem.cc
)
target_include_directories(WaterRecoverySystem PRIVATE include)
target_link_libraries(WaterRecoverySystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS WaterRecoverySystem
  LIBRARY DESTINATION lib
)

add_library(DryBinMonitorSystem SHARED
  src/DryBinMonitorSystem.cc
)
target_link_libraries(DryBinMonitorSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS DryBinMonitorSystem
  LIBRARY DESTINATION lib
)

add_library(GripperContactGateSystem SHARED
  src/GripperContactGateSystem.cc
)
target_link_libraries(GripperContactGateSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
target_include_directories(GripperContactGateSystem PRIVATE include)

install(TARGETS GripperContactGateSystem
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

add_library(GroundDirtCleaningSystem SHARED
  src/GroundDirtCleaningSystem.cc
)
target_link_libraries(GroundDirtCleaningSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS GroundDirtCleaningSystem
  LIBRARY DESTINATION lib
)

add_library(CleaningActuatorMotorSystem SHARED
  src/CleaningActuatorMotorCore.cc
  src/CleaningActuatorMotorSystem.cc
)
target_include_directories(CleaningActuatorMotorSystem PRIVATE include)
target_link_libraries(CleaningActuatorMotorSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
install(TARGETS CleaningActuatorMotorSystem
  LIBRARY DESTINATION lib
)

add_library(A300DrivetrainPlantSystem SHARED
  src/A300DrivetrainPlantCore.cc
  src/A300DrivetrainPlantSystem.cc
)
target_include_directories(A300DrivetrainPlantSystem PRIVATE include)
target_link_libraries(A300DrivetrainPlantSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
install(TARGETS A300DrivetrainPlantSystem
  LIBRARY DESTINATION lib
)

add_library(GripperMimicEffortSystem SHARED
  src/GripperMimicEffortSystem.cc
)
target_include_directories(GripperMimicEffortSystem PRIVATE include)
target_link_libraries(GripperMimicEffortSystem
  gz-sim8::gz-sim8
)
install(TARGETS GripperMimicEffortSystem
  LIBRARY DESTINATION lib
)

add_executable(a300_drivetrain_command_adapter
  src/A300DrivetrainCommandAdapter.cc
)
ament_target_dependencies(a300_drivetrain_command_adapter
  geometry_msgs
  rclcpp
  std_msgs
)
install(TARGETS a300_drivetrain_command_adapter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

add_executable(cleaning_actuator_vector_bridge
  src/CleaningActuatorVectorBridge.cc
)
target_include_directories(cleaning_actuator_vector_bridge PRIVATE include)
ament_target_dependencies(cleaning_actuator_vector_bridge
  rclcpp
  rosgraph_msgs
  std_msgs
)
target_link_libraries(cleaning_actuator_vector_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
install(TARGETS cleaning_actuator_vector_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
add_executable(water_evaluation_bridge
  src/WaterEvaluationBridge.cc
)
ament_target_dependencies(water_evaluation_bridge
  rclcpp
  std_msgs
)
target_link_libraries(water_evaluation_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
install(TARGETS water_evaluation_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
add_executable(manipulation_sim_bridge
  src/ManipulationSimBridge.cc
)
ament_target_dependencies(manipulation_sim_bridge
  rclcpp
  std_msgs
)
target_link_libraries(manipulation_sim_bridge
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
install(TARGETS manipulation_sim_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/cleaning_actuator_command_mirror.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME cleaning_actuator_command_mirror
)

# Make every installed model plugin discoverable from a clean sourced
# workspace. Product launch files also set the exact package lib path so a
# missing shell hook cannot silently disable physical effects.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/sanitation_gazebo_control.dsv.in"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_contact_gate_core
    test/test_contact_gate_core.cc
  )
  target_include_directories(test_contact_gate_core PRIVATE include)

  ament_add_gtest(test_water_recovery_pose_core
    test/test_water_recovery_pose_core.cc
  )
  target_include_directories(test_water_recovery_pose_core PRIVATE include)

  add_executable(test_a300_drivetrain_plant_core
    test/test_a300_drivetrain_plant_core.cc
    src/A300DrivetrainPlantCore.cc
  )
  target_include_directories(test_a300_drivetrain_plant_core PRIVATE include)
  add_test(
    NAME test_a300_drivetrain_plant_core
    COMMAND test_a300_drivetrain_plant_core
  )

  add_executable(test_cleaning_actuator_motor_core
    test/test_cleaning_actuator_motor_core.cc
    src/CleaningActuatorMotorCore.cc
  )
  target_include_directories(test_cleaning_actuator_motor_core PRIVATE include)
  add_test(
    NAME test_cleaning_actuator_motor_core
    COMMAND test_cleaning_actuator_motor_core
  )

  add_executable(test_gripper_mimic_effort_core
    test/test_gripper_mimic_effort_core.cc
  )
  target_include_directories(test_gripper_mimic_effort_core PRIVATE include)
  add_test(
    NAME test_gripper_mimic_effort_core
    COMMAND test_gripper_mimic_effort_core
  )
endif()

ament_package()
