cmake_minimum_required(VERSION 3.5)
project(navground_core)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if (WIN32)
add_compile_options(/wd4251)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(NAVGROUND_USES_DOUBLE "Whether to use double instead of float" OFF)
option(WITH_AMENT "Whether to use ament when available" ON)

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# include(FetchContent)
# FetchContent_Declare(
#     argparse
#     GIT_REPOSITORY https://github.com/p-ranav/argparse.git
# )
# FetchContent_MakeAvailable(argparse)

if (WITH_AMENT)
  find_package(ament_cmake QUIET)
  find_package(ament_index_cpp QUIET)
endif()
find_package(argparse REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(ORCA
  src/behaviors/ORCA/RVO2/Agent.cc
  src/behaviors/ORCA/RVO2/Obstacle.cc
  src/behaviors/ORCA/RVO2/KdTree.cc
  src/behaviors/ORCA/RVO2/Line.cc
  src/behaviors/ORCA/RVO2/Vector2.cc
  )

set(HRVO
  src/behaviors/HRVO/HRVO/Agent.cpp
  src/behaviors/HRVO/HRVO/Goal.cpp
  src/behaviors/HRVO/HRVO/HRVOSimulator.cpp
  src/behaviors/HRVO/HRVO/KdTree.cpp
  src/behaviors/HRVO/HRVO/Obstacle.cpp
  src/behaviors/HRVO/HRVO/RoadmapVertex.cpp
  src/behaviors/HRVO/HRVO/Vector2.cpp
  )

include_directories(${PROJECT_BINARY_DIR})

add_library(navground_core SHARED
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
  src/behavior_modulations/motor_pid.cpp
  src/controller.cpp
  src/controller_3d.cpp
  src/kinematics.cpp
  src/common.cpp
  src/plugins.cpp
)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/register_plugins.cmake)

if (ament_cmake_FOUND AND ament_index_cpp_FOUND)
  message("Using ament index to register plugins")
  set(NAVGROUND_PLUGINS_IN_AMENT_INDEX TRUE)
  target_compile_definitions(navground_core PUBLIC NAVGROUND_PLUGINS_IN_AMENT_INDEX)
  target_link_libraries(navground_core PUBLIC ament_index_cpp::ament_index_cpp)
else()
  message("Using ad-hoc index to register plugins")
  set(NAVGROUND_PLUGINS_IN_AMENT_INDEX FALSE)
endif()

target_compile_definitions(navground_core PUBLIC NAVGROUND_PLUGINS_INDEX="${NAVGROUND_PLUGINS_INDEX}")
target_include_directories(navground_core
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

generate_export_header(navground_core EXPORT_FILE_NAME navground/core/export.h)
target_link_libraries(navground_core PUBLIC Eigen3::Eigen yaml-cpp::yaml-cpp ${CMAKE_DL_LIBS})
if (NAVGROUND_USES_DOUBLE)
  message("Building navground using doubles instead of floats")
  target_compile_definitions(navground_core PUBLIC NAVGROUND_USES_DOUBLE)
endif()

# set_target_properties(navground_core PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

add_executable(info src/info.cpp)
target_link_libraries(info navground_core argparse::argparse)
add_dependencies(info navground_core)

install(TARGETS
  info
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/navground/core
  DESTINATION include/navground
)

install(FILES
  ${PROJECT_BINARY_DIR}/navground/core/export.h DESTINATION include/navground/core
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES ${CMAKE_CURRENT_SOURCE_DIR}/cmake/register_plugins.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

set("DSV" "prepend-non-duplicate;NAVGROUND_PLUGINS_PREFIX;")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv" CONTENT "${DSV}")
install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv"
  DESTINATION share/${PROJECT_NAME}/hook
)

if(ament_cmake_FOUND)
  if (ament_index_cpp_FOUND)
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
    DESTINATION share/${PROJECT_NAME}/cmake
  )

  include(CMakePackageConfigHelpers)
    
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION share/${PROJECT_NAME}/cmake
  )
  
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION share/${PROJECT_NAME}/cmake
  )

endif()
