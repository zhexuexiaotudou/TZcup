cmake_minimum_required(VERSION 3.16)
project(sanitation_localization_acceptance)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(formal_localization_runtime_collector
  src/formal_localization_runtime_collector.cpp
)
target_compile_features(formal_localization_runtime_collector PUBLIC cxx_std_17)
target_compile_options(formal_localization_runtime_collector PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
ament_target_dependencies(formal_localization_runtime_collector
  geometry_msgs nav_msgs rclcpp sensor_msgs tf2_msgs
)
target_link_libraries(formal_localization_runtime_collector
  nlohmann_json::nlohmann_json
)

install(TARGETS formal_localization_runtime_collector
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
