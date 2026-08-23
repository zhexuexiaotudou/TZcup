cmake_minimum_required(VERSION 3.16)
project(tzcup_j6_board_runtime LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(tzcup_j6_contracts STATIC src/contracts.cpp)
target_include_directories(tzcup_j6_contracts PUBLIC include)
target_compile_options(tzcup_j6_contracts PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(tzcup-j6-runtime src/main.cpp)
target_link_libraries(tzcup-j6-runtime PRIVATE tzcup_j6_contracts)
target_compile_options(tzcup-j6-runtime PRIVATE -Wall -Wextra -Wpedantic -Werror)

include(CTest)
if(BUILD_TESTING AND NOT CMAKE_CROSSCOMPILING)
  add_executable(tzcup_j6_contracts_test test/contracts_test.cpp)
  target_link_libraries(tzcup_j6_contracts_test PRIVATE tzcup_j6_contracts)
  add_test(NAME tzcup_j6_contracts_test COMMAND tzcup_j6_contracts_test)
endif()

install(TARGETS tzcup-j6-runtime RUNTIME DESTINATION bin)
