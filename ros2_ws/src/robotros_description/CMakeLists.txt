cmake_minimum_required(VERSION 3.16)
project(robotros_description NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # QUIET: a machine without ament_cmake_pytest still builds; the test is then reported as missing.
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_urdf test/test_urdf.py TIMEOUT 120)
  else()
    message(WARNING "ament_cmake_pytest not found: tests of ${PROJECT_NAME} are not built")
  endif()
endif()

ament_package()
