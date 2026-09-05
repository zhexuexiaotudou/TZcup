cmake_minimum_required(VERSION 3.10)
project(sanitation_gazebo_auxiliary)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Gazebo Harmonic's generated messages use Ubuntu's protobuf 3.x. Resolve the
# matching system protoc before ament prefix paths can expose another version.
if(UNIX AND EXISTS "/usr/bin/protoc")
  set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc" CACHE FILEPATH
      "System protoc matching Gazebo Harmonic protobuf" FORCE)
endif()
find_package(Protobuf REQUIRED MODULE)
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_library(sanitation_auxiliary_core
  src/EstopLatchCore.cc
  src/LightingCore.cc
  src/SqueegeeComplianceCore.cc
)
set_target_properties(sanitation_auxiliary_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)
target_include_directories(sanitation_auxiliary_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_library(FormalAuxiliaryVisualSystem SHARED
  src/FormalAuxiliaryVisualSystem.cc
)
target_include_directories(FormalAuxiliaryVisualSystem PRIVATE include)
target_link_libraries(FormalAuxiliaryVisualSystem
  sanitation_auxiliary_core
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

add_library(ServiceDoorSystem SHARED
  src/ServiceDoorSystem.cc
)
target_link_libraries(ServiceDoorSystem
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

add_library(SqueegeeComplianceSystem SHARED
  src/SqueegeeComplianceSystem.cc
)
target_link_libraries(SqueegeeComplianceSystem
  sanitation_auxiliary_core
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

install(TARGETS
  sanitation_auxiliary_core
  FormalAuxiliaryVisualSystem
  ServiceDoorSystem
  SqueegeeComplianceSystem
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/sanitation_gazebo_auxiliary.dsv.in"
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_lighting_core test/test_lighting_core.cc)
  target_link_libraries(test_lighting_core sanitation_auxiliary_core)
  target_include_directories(test_lighting_core PRIVATE include)

  ament_add_gtest(test_estop_latch_core test/test_estop_latch_core.cc)
  target_link_libraries(test_estop_latch_core sanitation_auxiliary_core)
  target_include_directories(test_estop_latch_core PRIVATE include)

  ament_add_gtest(test_squeegee_compliance_core test/test_squeegee_compliance_core.cc)
  target_link_libraries(test_squeegee_compliance_core sanitation_auxiliary_core)
  target_include_directories(test_squeegee_compliance_core PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(sanitation_auxiliary_core)
ament_package()
