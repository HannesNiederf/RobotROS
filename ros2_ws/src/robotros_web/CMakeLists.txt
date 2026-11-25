cmake_minimum_required(VERSION 3.8)
project(robotros_web)

find_package(ament_cmake REQUIRED)

# The browser UI lives at the repository root (web_ui/, spec 13.1); it is installed as data of this package.
set(WEB_UI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../web_ui")
if(NOT EXISTS "${WEB_UI_DIR}/index.html")
  message(FATAL_ERROR "web_ui/index.html not found at ${WEB_UI_DIR}")
endif()
install(DIRECTORY "${WEB_UI_DIR}/" DESTINATION share/${PROJECT_NAME}/web_ui
        PATTERN "test" EXCLUDE PATTERN "node_modules" EXCLUDE)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/web_server DESTINATION lib/${PROJECT_NAME})

ament_package()
