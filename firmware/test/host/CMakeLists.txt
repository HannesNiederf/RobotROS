# Host build of the firmware core and its tests (spez_firmware_protokoll.md section 11.1).
# The core is compiled as C++11 without exceptions and RTTI, like on the Nano Every (avr-gcc).
cmake_minimum_required(VERSION 3.16)
project(robotros_firmware_host_tests CXX)

set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib/core)
add_library(core STATIC ${CORE_DIR}/protocol.cpp ${CORE_DIR}/safety.cpp)
target_include_directories(core PUBLIC ${CORE_DIR})
set_target_properties(core PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON CXX_EXTENSIONS OFF)
target_compile_options(core PRIVATE -Wall -Wextra -Wpedantic -Wconversion -Wshadow -Werror -fno-exceptions -fno-rtti)

add_executable(test_core test_core.cpp)
target_link_libraries(test_core core)
set_target_properties(test_core PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(test_core PRIVATE -Wall -Wextra -Werror)
target_compile_definitions(test_core PRIVATE VECTORS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../vectors")

enable_testing()
add_test(NAME test_core COMMAND test_core)

option(WITH_SANITIZERS "Build with address and undefined behaviour sanitizers" OFF)
if(WITH_SANITIZERS)
  foreach(t core test_core)
    target_compile_options(${t} PRIVATE -fsanitize=address,undefined -fno-sanitize-recover=all -g)
    target_link_options(${t} PRIVATE -fsanitize=address,undefined)
  endforeach()
endif()
