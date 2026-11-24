cmake_minimum_required(VERSION 3.16)
project(robotros_bringup NONE)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch behavior_trees DESTINATION share/${PROJECT_NAME})

ament_package()
