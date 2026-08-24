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
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
find_package(gz-gui8 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

qt5_add_resources(control_resources_RCC SanitationMissionControl.qrc)

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
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
