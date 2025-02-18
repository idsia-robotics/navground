cmake_minimum_required(VERSION 3.11)
project(navground_sim VERSION 0.4.0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(MSVC)
  set(COMPILE_OPTIONS /wd4251 /wd4996 /wd4250 /wd4244 /wd4267)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILE_OPTIONS
      -Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments
      -Wno-unused-parameter -Wno-unused-but-set-variable)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILE_OPTIONS
      -Wall
      -Wextra
      -Wpedantic
      -Wno-unused-parameter
      -Wno-unused-but-set-variable
      -Wno-class-memaccess
      -Wno-deprecated-copy)
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

include(FetchContent)
FetchContent_Declare(
  cpptqdm
  GIT_REPOSITORY https://github.com/aminnj/cpptqdm.git
  GIT_SHALLOW TRUE
  GIT_TAG master)

FetchContent_MakeAvailable(cpptqdm)

find_package(HighFive QUIET)
if(NOT HighFive_FOUND)
  FetchContent_Declare(
    HighFive
    GIT_REPOSITORY https://github.com/BlueBrain/HighFive.git
    GIT_TAG v2.10.1
    GIT_SHALLOW TRUE
    OVERRIDE_FIND_PACKAGE)
  set(HIGHFIVE_USE_BOOST OFF)
  set(HIGHFIVE_BUILD_DOCS OFF)
  set(HIGHFIVE_EXAMPLES OFF)
  set(HIGHFIVE_USE_INSTALL_DEPS OFF)
  set(HIGHFIVE_STATIC_HDF5 OFF)
  FetchContent_MakeAvailable(HighFive)
endif()

option(BUILD_STANDALONE_CMDS "Whether to build commands"
       ${PROJECT_IS_TOP_LEVEL})

if(PROJECT_IS_TOP_LEVEL)
  find_package(navground_core REQUIRED)
  set(navground_core navground_core::navground_core)
else()
  set(navground_core navground_core)
endif()
find_package(GEOS REQUIRED)
find_package(HDF5 REQUIRED)

if(TARGET hdf5::hdf5)
  # set(hdf5-target hdf5::hdf5)
elseif(TARGET hdf5-shared)
  set(hdf5-target hdf5-shared)
elseif(TARGET hdf5-static)
  set(hdf5-target hdf5-static)
endif()

find_package(HighFive REQUIRED)

# Force to C++17
set(CMAKE_CXX_STANDARD 17)

include_directories(include ${PROJECT_BINARY_DIR})

add_library(
  navground_sim SHARED
  src/agent.cpp
  src/experiment.cpp
  src/experimental_run.cpp
  src/dataset.cpp
  src/probe.cpp
  src/scenario.cpp
  src/state_estimation.cpp
  src/world.cpp
  src/state_estimations/geometric_bounded.cpp
  src/state_estimations/sensor.cpp
  src/state_estimations/sensor_lidar.cpp
  src/state_estimations/sensor_discs.cpp
  src/state_estimations/sensor_boundary.cpp
  src/state_estimations/sensor_combination.cpp
  src/state_estimations/sensor_odometry.cpp
  src/tasks/waypoints.cpp
  src/tasks/direction.cpp
  src/scenarios/antipodal.cpp
  src/scenarios/corridor.cpp
  src/scenarios/cross.cpp
  src/scenarios/cross_torus.cpp
  src/scenarios/simple.cpp
  src/sampling.cpp
  src/yaml.cpp
  src/plugins.cpp
  src/build_info.cpp
  src/register.cpp)

target_compile_options(navground_sim PUBLIC ${COMPILE_OPTIONS})

generate_export_header(navground_sim EXPORT_FILE_NAME navground/sim/export.h)
include(${CMAKE_CURRENT_LIST_DIR}/../cmake/build_info.cmake)
configure_build_info(include/navground/sim/version.h.in navground/sim/version.h)

# -Wl,-force_load
target_link_libraries(
  navground_sim
  PUBLIC GEOS::geos
         # yaml-cpp::yaml-cpp hdf5_cpp-shared hdf5_hl_cpp-shared
         ${hdf5-target}
         # hdf5-shared hdf5_cpp-shared hdf5_hl-shared hdf5_hl_cpp-shared
         HighFive
         ${navground_core})

if(UNIX)
  find_package(Threads REQUIRED)
  target_link_libraries(navground_sim PRIVATE Threads::Threads)
endif()

target_include_directories(navground_sim
                           PUBLIC "$<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>")

# target_include_directories( navground_sim INTERFACE
# $<TARGET_PROPERTY:hdf5-shared,INTERFACE_INCLUDE_DIRECTORIES>)
# target_include_directories( navground_sim INTERFACE
# $<TARGET_PROPERTY:hdf5_hl-shared,INTERFACE_INCLUDE_DIRECTORIES>)

# target_include_directories(navground_sim INTERFACE
# $<TARGET_PROPERTY:${navground_core},INTERFACE_INCLUDE_DIRECTORIES>)
# target_include_directories(navground_sim INTERFACE
# $<TARGET_PROPERTY:HighFive,INTERFACE_INCLUDE_DIRECTORIES>)
# target_include_directories(navground_sim INTERFACE
# $<TARGET_PROPERTY:HighFive,PUBLIC_INCLUDE_DIRECTORIES>)

# target_include_directories(navground_sim INTERFACE
# $<TARGET_PROPERTY:HighFive,INTERFACE_INCLUDE_DIRECTORIES>) Else fails
# compiling with clang 15, see
# https://developer.apple.com/documentation/xcode-release-notes/xcode-15-release-notes
target_compile_definitions(
  navground_sim PUBLIC _LIBCPP_ENABLE_CXX17_REMOVED_UNARY_BINARY_FUNCTION)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/build_info.cmake)

if(BUILD_STANDALONE_CMDS)
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

  add_executable(echo src/echo.cpp)
  target_link_libraries(echo navground_sim)
  add_dependencies(echo navground_sim)

  add_executable(plugins src/list_plugins.cpp)
  target_link_libraries(plugins navground_sim)
  add_dependencies(plugins navground_sim)

  add_executable(schema src/schema.cpp)
  target_link_libraries(schema navground_sim)
  add_dependencies(schema navground_sim)

  install(TARGETS info run sample echo plugins schema
          DESTINATION lib/${PROJECT_NAME})
endif()

add_executable(navground src/main.cpp)
target_link_libraries(navground navground_sim)
add_dependencies(navground navground_sim)
target_include_directories(navground PRIVATE ${cpptqdm_SOURCE_DIR})
set_target_properties(navground PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                           "${CMAKE_BINARY_DIR}/bin")

install(TARGETS navground DESTINATION bin)

install(DIRECTORY include/navground/sim DESTINATION include/navground)

install(FILES ${PROJECT_BINARY_DIR}/navground/sim/export.h
              ${PROJECT_BINARY_DIR}/navground/sim/version.h
        DESTINATION include/navground/sim)

install(
  TARGETS navground_sim
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(ament_cmake_FOUND)
  ament_export_dependencies(geos navground_core HighFive)
  if(UNIX)
    ament_export_dependencies(Threads)
  endif()
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_package()
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