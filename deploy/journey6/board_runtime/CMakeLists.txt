cmake_minimum_required(VERSION 3.16)
project(tzcup_j6_board_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

function(tzcup_j6_enable_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endfunction()

add_library(tzcup_j6_contracts STATIC
  src/contracts.cpp
  src/d1_yolov9_postprocess.cpp
)
target_include_directories(tzcup_j6_contracts PUBLIC include)
tzcup_j6_enable_warnings(tzcup_j6_contracts)

add_executable(tzcup-j6-runtime src/main.cpp)
target_link_libraries(tzcup-j6-runtime PRIVATE tzcup_j6_contracts)
tzcup_j6_enable_warnings(tzcup-j6-runtime)

include(CTest)
if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
  add_executable(tzcup_j6_contracts_test test/contracts_test.cpp)
  target_link_libraries(tzcup_j6_contracts_test PRIVATE tzcup_j6_contracts)
  tzcup_j6_enable_warnings(tzcup_j6_contracts_test)
  add_test(NAME tzcup_j6_contracts_test COMMAND tzcup_j6_contracts_test)

  add_executable(tzcup_j6_d1_postprocess_test test/d1_yolov9_postprocess_test.cpp)
  target_link_libraries(tzcup_j6_d1_postprocess_test PRIVATE tzcup_j6_contracts)
  tzcup_j6_enable_warnings(tzcup_j6_d1_postprocess_test)
  add_test(NAME tzcup_j6_d1_postprocess_test COMMAND tzcup_j6_d1_postprocess_test)

  add_executable(tzcup_j6_d1_golden_test test/d1_golden_postprocess_test.cpp)
  target_link_libraries(tzcup_j6_d1_golden_test PRIVATE tzcup_j6_contracts)
  tzcup_j6_enable_warnings(tzcup_j6_d1_golden_test)
  get_filename_component(TZCUP_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  set(
    TZCUP_D1_GOLDEN_TENSOR
    "${TZCUP_REPO_ROOT}/.workspace/evidence/j6/d1_postprocess_golden/E1_scene30001_frame00_primary.f32"
    CACHE FILEPATH "Reference-only real D1 graph output for cross-language verification"
  )
  if(EXISTS "${TZCUP_D1_GOLDEN_TENSOR}")
    add_test(
      NAME tzcup_j6_d1_golden_test
      COMMAND tzcup_j6_d1_golden_test "${TZCUP_D1_GOLDEN_TENSOR}"
    )
    set_tests_properties(tzcup_j6_d1_golden_test PROPERTIES LABELS external_golden)
  else()
    message(STATUS "D1 external golden tensor absent; cross-language golden CTest remains blocked")
  endif()
endif()

install(TARGETS tzcup-j6-runtime RUNTIME DESTINATION bin)
