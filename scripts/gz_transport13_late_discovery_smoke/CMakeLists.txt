cmake_minimum_required(VERSION 3.16)
project(tzcup_gz_transport13_late_discovery_smoke LANGUAGES CXX)

# The caller supplies the frozen merged install.  Do not silently fall back to
# the system gz_transport vendor: the run script checks the resulting binary
# and both live processes again before accepting a result.
if(NOT DEFINED TZCUP_FROZEN_RUNTIME_PREFIX)
  message(FATAL_ERROR "TZCUP_FROZEN_RUNTIME_PREFIX is required")
endif()
file(REAL_PATH "${TZCUP_FROZEN_RUNTIME_PREFIX}" TZCUP_FROZEN_RUNTIME_PREFIX)
if(NOT IS_DIRECTORY "${TZCUP_FROZEN_RUNTIME_PREFIX}/lib")
  message(FATAL_ERROR "frozen runtime has no lib directory")
endif()

find_package(gz-transport13 REQUIRED CONFIG)
find_package(gz-msgs10 REQUIRED CONFIG)

# The transport config's find_dependency chain may locate its support packages
# in Jazzy vendor prefixes, but the transport config itself must originate in
# the frozen merged install, never in /opt/ros.
if(NOT DEFINED gz-transport13_DIR)
  message(FATAL_ERROR "gz-transport13 config directory was not reported")
endif()
file(REAL_PATH "${gz-transport13_DIR}" TZCUP_TRANSPORT_CONFIG_DIRECTORY)
file(RELATIVE_PATH TZCUP_TRANSPORT_CONFIG_RELATIVE
  "${TZCUP_FROZEN_RUNTIME_PREFIX}" "${TZCUP_TRANSPORT_CONFIG_DIRECTORY}")
if(TZCUP_TRANSPORT_CONFIG_RELATIVE MATCHES "^\\.\\.")
  message(FATAL_ERROR
    "gz-transport13 config escaped frozen runtime: ${TZCUP_TRANSPORT_CONFIG_DIRECTORY}")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/target_contract.cmake")
file(REAL_PATH "${TZCUP_TRANSPORT_IMPORTED_LOCATION}"
  TZCUP_TRANSPORT_IMPORTED_LOCATION)
file(REAL_PATH "${TZCUP_FROZEN_RUNTIME_PREFIX}/lib"
  TZCUP_FROZEN_RUNTIME_LIBDIR)
file(RELATIVE_PATH TZCUP_TRANSPORT_RELATIVE
  "${TZCUP_FROZEN_RUNTIME_LIBDIR}" "${TZCUP_TRANSPORT_IMPORTED_LOCATION}")
if(TZCUP_TRANSPORT_RELATIVE MATCHES "^\\.\\." OR
   NOT TZCUP_TRANSPORT_IMPORTED_LOCATION MATCHES
       "/libgz-transport13\\.so(\\.13(\\.5\\.0)?)?$")
  message(FATAL_ERROR
    "gz-transport13 did not resolve to the frozen runtime: ${TZCUP_TRANSPORT_IMPORTED_LOCATION}")
endif()

add_executable(gz_transport13_late_discovery_smoke main.cc)
target_compile_features(gz_transport13_late_discovery_smoke PRIVATE cxx_std_17)
target_link_libraries(gz_transport13_late_discovery_smoke PRIVATE
  gz-transport13::core
  gz-msgs10::gz-msgs10)
set_target_properties(gz_transport13_late_discovery_smoke PROPERTIES
  BUILD_RPATH "${TZCUP_FROZEN_RUNTIME_LIBDIR}"
  BUILD_RPATH_USE_ORIGIN FALSE)
