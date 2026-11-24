cmake_minimum_required(VERSION 3.16)
project(robotros_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_smoke_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # QUIET: a machine without ament_cmake_pytest still builds; the test is then reported as missing.
  find_package(ament_cmake_pytest QUIET)
  if(ament_cmake_pytest_FOUND)
    ament_add_pytest_test(test_world test/test_world.py TIMEOUT 60)
  else()
    message(WARNING "ament_cmake_pytest not found: tests of ${PROJECT_NAME} are not built")
  endif()
endif()

ament_package()
