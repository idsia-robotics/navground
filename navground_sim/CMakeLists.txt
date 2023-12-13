cmake_minimum_required(VERSION 3.5)
project(navground_sim)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/ament_python_install_namespace_package.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/generate_docstrings.cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments -Wno-unused-parameter)
endif()


include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

include(FetchContent)
FetchContent_Declare(
  cpptqdm
  GIT_REPOSITORY https://github.com/aminnj/cpptqdm.git
  GIT_TAG        master
)

FetchContent_MakeAvailable(cpptqdm)

message("cpptqdm_SOURCE_DIR ${cpptqdm_SOURCE_DIR}")

# add_subdirectory(HighFive)
# add_subdirectory(Crow)
# set(YAML_CPP_INSTALL ON CACHE INTERNAL "")
# add_subdirectory(../yaml-cpp yaml-cpp)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(navground_core REQUIRED)
find_package(navground_py REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(GEOS REQUIRED)
find_package(pybind11 REQUIRED)
find_package(HighFive REQUIRED)

# add_subdirectory(../external/geos geos)
# add_subdirectory(../external/pybind11 pybind11 EXCLUDE_FROM_ALL)
# add_subdirectory(../external/HighFive highFive)
# find_package(pybind11_vendor REQUIRED)
# find_package(pybind11 REQUIRED)

# Force to C++17
set(CMAKE_CXX_STANDARD 17)

include_directories(include ${PROJECT_BINARY_DIR})

# add_executable(test_samplers src/test_samplers.cpp src/sampling/sampler.cpp)
# target_link_libraries(test_samplers navground_core::navground_core )

# add_executable(test_samplers_yaml src/test_samplers_yaml.cpp src/sampling/sampler.cpp)
# target_link_libraries(test_samplers_yaml navground_core::navground_core)

add_library(navground_sim SHARED
  src/agent.cpp
  src/experiment.cpp 
  src/scenario.cpp
  src/state_estimation.cpp
  src/world.cpp 
  src/state_estimations/geometric_bounded.cpp
  src/state_estimations/sensor_lidar.cpp
  src/state_estimations/sensor_discs.cpp
  src/tasks/waypoints.cpp
  src/scenarios/antipodal.cpp 
  src/scenarios/corridor.cpp 
  src/scenarios/cross.cpp
  src/scenarios/cross_torus.cpp
  src/sampling/sampler.cpp
)

generate_export_header(navground_sim)

# -Wl,-force_load 
target_link_libraries(
  navground_sim PUBLIC 
  GEOS::geos 
  # yaml-cpp::yaml-cpp 
  HighFive
  navground_core::navground_core 
)
target_link_libraries(navground_sim PRIVATE )
target_include_directories(navground_sim INTERFACE $<TARGET_PROPERTY:HighFive,INTERFACE_INCLUDE_DIRECTORIES>)
# Else fails compiling with clang 15, see https://developer.apple.com/documentation/xcode-release-notes/xcode-15-release-notes
target_compile_definitions(navground_sim PUBLIC _LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION)

ament_export_dependencies(geos navground_core HighFive)
ament_export_targets(navground_simTargets HAS_LIBRARY_TARGET)

add_executable(run src/run.cpp)
target_link_libraries(run navground_sim)
add_dependencies(run navground_sim)
target_include_directories(run PRIVATE ${cpptqdm_SOURCE_DIR})

add_executable(sample src/sample.cpp)
target_link_libraries(sample navground_sim)
add_dependencies(sample navground_sim)

add_executable(info src/info.cpp)
target_link_libraries(info navground_sim)
add_dependencies(info navground_sim)

# add_executable(echo src/echo.cpp)
# target_link_libraries(echo navground_sim)
# add_dependencies(echo navground_sim)

ament_python_install_namespace_package(sim
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
  PACKAGE_DIR navground/sim
  NAMESPACE navground
)
pybind11_add_module(_navground_sim src/python.cpp)

generate_docstrings(
  _navground_sim 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp 
  ${CMAKE_CURRENT_SOURCE_DIR}/include/navground/sim
  ${CMAKE_INSTALL_PREFIX}/include 
  ${PROJECT_BINARY_DIR} 
  ${EIGEN3_INCLUDE_DIR}
)

# target_compile_options(_navground_sim PRIVATE -fvisibility=hidden)
target_link_libraries(_navground_sim 
  PRIVATE 
  navground_sim
)
target_link_libraries(_navground_sim INTERFACE navground_py)
# 
add_dependencies(_navground_sim navground_sim)

# target_link_libraries(_navground_sim PRIVATE -Wl,-force_load navground_sim)

install(TARGETS _navground_sim DESTINATION "${PYTHON_INSTALL_DIR}/navground/sim")

install(TARGETS
  info
  run
  sample
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/navground/sim
  DESTINATION include/navground
)

install(FILES
 ${PROJECT_BINARY_DIR}/navground_sim_export.h DESTINATION include
)

install(
  TARGETS navground_sim
  EXPORT navground_simTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
