cmake_minimum_required(VERSION 3.5)
project(navground_examples)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GenerateExportHeader)

include_directories(include ${PROJECT_BINARY_DIR})

find_package(ament_cmake REQUIRED)
find_package(navground_core REQUIRED)
find_package(navground_sim REQUIRED)

add_subdirectory(src/behavior)
add_subdirectory(src/controller)
add_subdirectory(src/collision)
add_subdirectory(src/scenario)
add_subdirectory(src/world)
add_subdirectory(src/experiment)

generate_export_header(my_behavior)
generate_export_header(my_scenario)

ament_export_targets(my_scenarioTargets HAS_LIBRARY_TARGET)
ament_export_targets(my_behaviorTargets HAS_LIBRARY_TARGET)

if (NOT WIN32)
ament_index_register_resource(navground_plugins CONTENT $<TARGET_FILE:my_behavior>;$<TARGET_FILE:my_scenario>)
else()
ament_index_register_resource(navground_plugins CONTENT ${CMAKE_INSTALL_PREFIX}/bin/$<TARGET_FILE_NAME:my_behavior>;${CMAKE_INSTALL_PREFIX}/bin/$<TARGET_FILE_NAME:my_scenario>)
endif()

ament_package()