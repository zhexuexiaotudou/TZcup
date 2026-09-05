cmake_minimum_required(VERSION 3.16)
project(tzcup_gz_transport13_target_contract_fixture NONE)

set(TZCUP_FIXTURE_LIBRARY "${CMAKE_CURRENT_BINARY_DIR}/libgz-transport13.so.13")
file(WRITE "${TZCUP_FIXTURE_LIBRARY}" "fixture transport bytes")
add_library(gz-transport13::gz-transport13 SHARED IMPORTED GLOBAL)
set_target_properties(gz-transport13::gz-transport13 PROPERTIES
  IMPORTED_LOCATION_RELEASE "${TZCUP_FIXTURE_LIBRARY}")
add_library(gz-transport13::core INTERFACE IMPORTED GLOBAL)
set(TZCUP_FIXTURE_CORE_LINKS "gz-transport13::gz-transport13")
if(TZCUP_FIXTURE_EXTRA_TRANSPORT_LINK)
  add_library(gz-transport13::unexpected INTERFACE IMPORTED GLOBAL)
  list(APPEND TZCUP_FIXTURE_CORE_LINKS "gz-transport13::unexpected")
endif()
set_target_properties(gz-transport13::core PROPERTIES
  INTERFACE_LINK_LIBRARIES "${TZCUP_FIXTURE_CORE_LINKS}")
include("${CMAKE_CURRENT_LIST_DIR}/../target_contract.cmake")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/resolved.txt"
  "${TZCUP_TRANSPORT_IMPORTED_LOCATION}\n")
