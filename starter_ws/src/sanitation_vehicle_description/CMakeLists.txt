cmake_minimum_required(VERSION 3.8)
project(sanitation_vehicle_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY cad config launch meshes urdf rviz worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    scripts/formal_encoder_feedback_publisher.py
    scripts/formal_encoder_quantization.py
    scripts/formal_fisheye_camera_info_publisher.py
    scripts/formal_robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
