cmake_minimum_required(VERSION 3.10)
project(a300_drivetrain_candidate_compile_test)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Match the package build: Gazebo Harmonic messages were generated with the
# Ubuntu protobuf 3.x toolchain, not an unrelated newer prefix installation.
if(UNIX AND EXISTS "/usr/bin/protoc")
  set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc" CACHE FILEPATH
      "System protoc matching Gazebo Harmonic protobuf" FORCE)
endif()
find_package(Protobuf REQUIRED MODULE)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

get_filename_component(PACKAGE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

add_library(A300DrivetrainPlantSystemCandidate SHARED
  "${PACKAGE_ROOT}/src/A300DrivetrainPlantCore.cc"
  "${PACKAGE_ROOT}/src/A300DrivetrainPlantSystem.cc"
)
target_include_directories(A300DrivetrainPlantSystemCandidate PRIVATE
  "${PACKAGE_ROOT}/include"
)
target_compile_options(A300DrivetrainPlantSystemCandidate PRIVATE
  -Wall -Wextra -Werror
)
target_link_libraries(A300DrivetrainPlantSystemCandidate
  gz-sim8::gz-sim8
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)
