cmake_minimum_required(VERSION 3.10)
project(sanitation_gazebo_control)

if(POLICY CMP0100)
  cmake_policy(SET CMP0100 NEW)
endif()

set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)
find_package(gz-gui8 REQUIRED)

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
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
