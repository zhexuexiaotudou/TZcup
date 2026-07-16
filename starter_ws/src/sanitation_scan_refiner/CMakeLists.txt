cmake_minimum_required(VERSION 3.8)
project(sanitation_scan_refiner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(scan_refiner_core src/scan_refiner_core.cpp)
target_include_directories(scan_refiner_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(scan_refiner_node src/scan_refiner_node.cpp)
target_link_libraries(scan_refiner_node scan_refiner_core)
ament_target_dependencies(scan_refiner_node
  rclcpp diagnostic_msgs geometry_msgs nav_msgs sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros)

add_executable(hybrid_global_fuser_node src/hybrid_global_fuser_node.cpp)
ament_target_dependencies(hybrid_global_fuser_node
  rclcpp diagnostic_msgs geometry_msgs nav_msgs sensor_msgs
  tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS scan_refiner_core scan_refiner_node hybrid_global_fuser_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_scan_refiner_core test/test_scan_refiner_core.cpp)
  target_link_libraries(test_scan_refiner_core scan_refiner_core)
  target_include_directories(test_scan_refiner_core PRIVATE include)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(scan_refiner_core)
ament_package()
