cmake_minimum_required(VERSION 3.5)
project(navground_examples_yaml)

find_package(ament_cmake QUIET)

install(
  DIRECTORY experiment
  DESTINATION share/${PROJECT_NAME}
)

if (ament_cmake_FOUND)
  ament_package()
endif()