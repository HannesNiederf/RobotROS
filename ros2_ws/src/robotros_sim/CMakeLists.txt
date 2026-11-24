cmake_minimum_required(VERSION 3.16)
project(robotros_sim NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/sim_smoke_check.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_world test/test_world.py TIMEOUT 60)
endif()

ament_package()
