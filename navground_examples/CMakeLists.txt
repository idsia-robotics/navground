cmake_minimum_required(VERSION 3.16.3)
project(navground_examples VERSION 0.5.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GenerateExportHeader)

include_directories(include ${PROJECT_BINARY_DIR})

option(PLUGINS "Whether to build the plugins" ON)
option(EXECUTABLES "Whether to build the executables" ON)

find_package(navground_core REQUIRED)
find_package(navground_sim REQUIRED)
find_package(ament_cmake QUIET)

if(PROJECT_IS_TOP_LEVEL)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

if(EXECUTABLES OR BUILD_TESTING)
  add_subdirectory(src/behavior)
  add_subdirectory(src/controller)
  add_subdirectory(src/collision)
  add_subdirectory(src/scenario)
  add_subdirectory(src/world)
  add_subdirectory(src/experiment)
endif()

if(PLUGINS)
  add_subdirectory(src/behavior_plugin)
  add_subdirectory(src/scenario_plugin)
  generate_export_header(my_behavior)
  generate_export_header(my_scenario)
  register_navground_plugins(TARGETS my_behavior my_scenario DESTINATION
                             $<IF:$<BOOL:${WIN32}>,bin,lib>)
endif()

if(ament_cmake_FOUND)
  # ament_export_targets(my_scenarioTargets HAS_LIBRARY_TARGET)
  # ament_export_targets(my_behaviorTargets HAS_LIBRARY_TARGET)
  ament_package()
endif()


