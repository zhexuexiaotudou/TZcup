cmake_minimum_required(VERSION 3.8)
project(sanitation_vehicle_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cad config launch meshes urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

ament_package()
