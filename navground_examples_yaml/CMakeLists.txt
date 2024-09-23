cmake_minimum_required(VERSION 3.5)
project(navground_examples_yaml)

install(
  DIRECTORY experiment
  DESTINATION share/${PROJECT_NAME}
)

if (ament_cmake_FOUND)
  ament_package()
endif()