cmake_minimum_required(VERSION 3.8)
project(sanitation_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/scan_self_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_self_filter
)
install(PROGRAMS scripts/pointcloud_self_filter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_self_filter
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
