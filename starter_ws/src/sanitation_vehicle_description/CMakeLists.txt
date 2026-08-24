cmake_minimum_required(VERSION 3.8)
project(sanitation_vehicle_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cad config launch urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
