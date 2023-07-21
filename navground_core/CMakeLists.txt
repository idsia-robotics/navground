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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

include(FetchContent)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)


find_package(ament_cmake REQUIRED)
find_package(yaml-cpp)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(ORCA
  src/behaviors/ORCA/RVO/Agent.cpp
  src/behaviors/ORCA/RVO/Obstacle.cpp
  src/behaviors/ORCA/RVO/RVOSimulator.cpp
  src/behaviors/ORCA/RVO/KdTree.cpp
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

include_directories(include ${PROJECT_BINARY_DIR})

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
  src/controller.cpp
  src/controller_3d.cpp
  src/kinematics.cpp
)

generate_export_header(navground_core)

target_link_libraries(navground_core PUBLIC Eigen3::Eigen)

ament_export_dependencies(Eigen3)

target_link_libraries(navground_core PUBLIC yaml-cpp)
ament_export_dependencies(yaml-cpp)

ament_export_targets(navground_coreTargets HAS_LIBRARY_TARGET)

add_executable(info src/info.cpp)
target_link_libraries(info navground_core argparse)
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
 ${PROJECT_BINARY_DIR}/navground_core_export.h DESTINATION include
)


install(
  TARGETS navground_core
  EXPORT navground_coreTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
