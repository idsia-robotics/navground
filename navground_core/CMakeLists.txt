cmake_minimum_required(VERSION 3.11)
project(navground_core VERSION 0.5.1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(MSVC)
  set(COMPILE_OPTIONS /wd4251 /wd4250 /wd4661 /wd4996)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILE_OPTIONS -Wall -Wextra -Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-deprecated-copy -Wno-unknown-pragmas -Wno-class-memaccess)
endif()

option(NAVGROUND_USES_DOUBLE "Whether to use double instead of float" OFF)
option(WITH_AMENT "Whether to use ament when available" ON)
option(BUILD_STANDALONE_CMDS "Whether to build commands" ${PROJECT_IS_TOP_LEVEL})

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# include(FetchContent) set(FETCHCONTENT_QUIET OFF)
# FetchContent_Declare(cmake_git_version_tracking GIT_REPOSITORY
# https://github.com/andrew-hardin/cmake-git-version-tracking.git GIT_TAG
# 904dbda1336ba4b9a1415a68d5f203f576b696bb )
# FetchContent_MakeAvailable(cmake_git_version_tracking)

if(WITH_AMENT)
  find_package(ament_cmake QUIET)
  find_package(ament_index_cpp QUIET)
endif()
# TODO: was commented for FETCH_EIGEN case ...
find_package(argparse REQUIRED)
find_package(yaml-cpp REQUIRED)
if(TARGET yaml-cpp::yaml-cpp)
  set(yaml-cpp-target yaml-cpp::yaml-cpp)
else()
  set(yaml-cpp-target yaml-cpp)
endif()
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(ORCA
    src/behaviors/ORCA/RVO2/Agent.cc src/behaviors/ORCA/RVO2/Obstacle.cc
    src/behaviors/ORCA/RVO2/KdTree.cc src/behaviors/ORCA/RVO2/Line.cc
    src/behaviors/ORCA/RVO2/Vector2.cc)

set(HRVO
    src/behaviors/HRVO/HRVO/Agent.cpp
    src/behaviors/HRVO/HRVO/Goal.cpp
    src/behaviors/HRVO/HRVO/HRVOSimulator.cpp
    src/behaviors/HRVO/HRVO/KdTree.cpp
    src/behaviors/HRVO/HRVO/Obstacle.cpp
    src/behaviors/HRVO/HRVO/RoadmapVertex.cpp
    src/behaviors/HRVO/HRVO/Vector2.cpp)

# include_directories(${PROJECT_BINARY_DIR})

add_library(
  navground_core SHARED
  src/collision_computation.cpp
  src/cached_collision_computation.cpp
  src/behavior.cpp
  src/behaviors/dummy.cpp
  src/behaviors/HL.cpp
  src/behaviors/ORCA/ORCA.cpp
  ${ORCA}
  src/behaviors/HRVO/HRVO.cpp
  ${HRVO}
  src/behavior_modulations/relaxation.cpp
  src/behavior_modulations/limit_acceleration.cpp
  src/behavior_modulations/limit_twist.cpp
  src/behavior_modulations/motor_pid.cpp
  src/states/gridmap.cpp
  src/controller.cpp
  src/controller_3d.cpp
  src/kinematics.cpp
  src/common.cpp
  src/plugins.cpp
  src/build_info.cpp
  src/register.cpp)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/register_plugins.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/build_info.cmake)

configure_build_info(include/navground/core/version.h.in
                     navground/core/version.h)

if(ament_cmake_FOUND AND ament_index_cpp_FOUND)
  message("Using ament index to register plugins")
  set(NAVGROUND_PLUGINS_IN_AMENT_INDEX TRUE)
  target_compile_definitions(navground_core
                             PUBLIC NAVGROUND_PLUGINS_IN_AMENT_INDEX)
  target_link_libraries(navground_core PUBLIC ament_index_cpp::ament_index_cpp)
else()
  message("Using ad-hoc index to register plugins")
  set(NAVGROUND_PLUGINS_IN_AMENT_INDEX FALSE)
endif()
# add_build_info(navground_core)

target_compile_definitions(
  navground_core PUBLIC NAVGROUND_PLUGINS_INDEX="${NAVGROUND_PLUGINS_INDEX}")
target_include_directories(
  navground_core PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
                        "$<INSTALL_INTERFACE:include>")

target_include_directories(navground_core
                           PUBLIC "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>")

generate_export_header(navground_core EXPORT_FILE_NAME navground/core/export.h)
target_link_libraries(navground_core PUBLIC Eigen3::Eigen ${yaml-cpp-target} argparse::argparse
                                            ${CMAKE_DL_LIBS})

if(NAVGROUND_USES_DOUBLE)
  message("Building navground using doubles instead of floats")
  target_compile_definitions(navground_core PUBLIC NAVGROUND_USES_DOUBLE)
endif()

# set_target_properties(navground_core PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS
# TRUE)

target_compile_options(navground_core PUBLIC ${COMPILE_OPTIONS})

if(BUILD_STANDALONE_CMDS)

  add_executable(info src/info.cpp)
  target_link_libraries(info navground_core)
  add_dependencies(info navground_core)

  add_executable(echo src/echo.cpp)
  target_link_libraries(echo navground_core)
  add_dependencies(echo navground_core)

  add_executable(schema src/schema.cpp)
  target_link_libraries(schema navground_core)
  add_dependencies(schema navground_core)

  add_executable(plugins src/list_plugins.cpp)
  target_link_libraries(plugins navground_core)
  add_dependencies(plugins navground_core)

  install(TARGETS info echo plugins schema DESTINATION lib/${PROJECT_NAME})

endif()

install(DIRECTORY include/navground/core DESTINATION include/navground)

install(FILES ${PROJECT_BINARY_DIR}/navground/core/export.h
              ${PROJECT_BINARY_DIR}/navground/core/version.h
        DESTINATION include/navground/core)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/register_plugins.cmake
        DESTINATION share/${PROJECT_NAME}/cmake)

if(ament_cmake_FOUND)
  if(ament_index_cpp_FOUND)
    ament_export_dependencies(ament_index_cpp)
  endif()
  ament_export_dependencies(Eigen3 yaml-cpp argparse)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_package(CONFIG_EXTRAS "cmake/plugins_config.cmake.in")

else()

  install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION share/${PROJECT_NAME}/cmake)

  include(CMakePackageConfigHelpers)

  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake)

  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
          DESTINATION share/${PROJECT_NAME}/cmake)

endif()

if(PROJECT_IS_TOP_LEVEL AND BUILD_TESTING)
  include(CTest)
  if (BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

if (PROJECT_IS_TOP_LEVEL)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cpack.cmake)
endif()