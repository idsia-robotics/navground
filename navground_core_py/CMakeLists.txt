cmake_minimum_required(VERSION 3.11)
project(navground_core_py VERSION 0.5.0)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic
# -Wno-gnu-zero-variadic-macro-arguments) endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

option(INSTALL_PYTHON_PKG "Whether to install the Python package"
       ${PROJECT_IS_TOP_LEVEL})
option(BUILD_PYTHON_PKG "Whether to build the Python package"
       ${PROJECT_IS_TOP_LEVEL})

set(BUILD_PKG_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}"
    CACHE STRING "Where to build the extension")

set(Python3_FIND_STRATEGY LOCATION)

find_package(navground_core REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v2.13.6
    GIT_SHALLOW TRUE
    OVERRIDE_FIND_PACKAGE)
  set(PYBIND11_INSTALL ON)
  set(INCLUDE_PYBIND11 true)
  FetchContent_MakeAvailable(pybind11)
  find_package(pybind11 REQUIRED)
endif()

add_library(navground_core_py INTERFACE)
target_include_directories(
  navground_core_py
  INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
            "$<INSTALL_INTERFACE:include>")

set(BUILD_INCLUDE_DIRECTORY ${PROJECT_BINARY_DIR}/include)
target_include_directories(
  navground_core_py INTERFACE "$<BUILD_INTERFACE:${BUILD_INCLUDE_DIRECTORY}>")

target_link_libraries(navground_core_py INTERFACE pybind11::pybind11)

pybind11_add_module(_navground src/python.cpp)

get_target_property(INCLUDE_DIRS navground_core::navground_core
                    INTERFACE_INCLUDE_DIRECTORIES)
list(GET INCLUDE_DIRS 0 INCLUDE_DIR)

# get_target_property(_location navground_core::navground_core LOCATION)
# cmake_path(GET _location PARENT_PATH _location) cmake_path(GET _location
# PARENT_PATH _location) message("_location ${_location}")

include(generate_docstrings)
generate_docstrings(
  _navground docstrings.h ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp
  ${INCLUDE_DIR}/navground/core ${INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# include(generate_stubs) generate_stubs( _navground
# ${CMAKE_BINARY_DIR}/navground/core )

# target_compile_options(_navground PRIVATE -fvisibility=hidden)
target_compile_definitions(_navground
                           PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
target_link_libraries(_navground PRIVATE navground_core_py
                                         navground_core::navground_core)
target_include_directories(_navground PRIVATE ${PROJECT_BINARY_DIR})
target_include_directories(
  _navground PUBLIC "$<BUILD_INTERFACE:${BUILD_INCLUDE_DIRECTORY}>")
add_dependencies(_navground navground_core_py)

if(MSVC)
  set(COMPILE_OPTIONS "/wd4335")
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILE_OPTIONS)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILE_OPTIONS "-Wno-unused-variable")
endif()

target_compile_options(_navground PRIVATE ${COMPILE_OPTIONS})

set_target_properties(
  _navground PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                        ${BUILD_PKG_DIRECTORY}/navground/core)
set_target_properties(
  _navground PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                        ${BUILD_PKG_DIRECTORY}/navground/core)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT error)
if(ipo_supported)
  set_property(TARGET _navground PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/build_info.cmake)
configure_build_info(include/navground/core_py/version.h.in
                     ${BUILD_INCLUDE_DIRECTORY}/navground/core_py/version.h)

install(DIRECTORY include/navground/core_py DESTINATION include/navground)

install(FILES ${BUILD_INCLUDE_DIRECTORY}/navground/core_py/version.h
        DESTINATION include/navground/core_py)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)

if(ament_cmake_python_FOUND)
  if(INSTALL_PYTHON_PKG)
    include(ament_python_install_namespace_package)

    # ament_python_install_package seems to not support namespace packages For
    # those: we should install the pkg to <site-packages>/navground and the meta
    # information in <site-packages>/core-0.0.0-py3.10.egg-info BUT the one
    # below install both under DESTINATION

    ament_python_install_namespace_package(
      core
      SCRIPTS_DESTINATION
      lib/${PROJECT_NAME}
      PACKAGE_DIR
      navground/core
      NAMESPACE
      navground)

    install(TARGETS _navground
            DESTINATION "${PYTHON_INSTALL_DIR}/navground/core")

    if(WIN32)
      # install(FILES ${CMAKE_INSTALL_PREFIX}/bin/navground_core.dll
      # ${CMAKE_INSTALL_PREFIX}/bin/ament_index_cpp.dll DESTINATION
      # "${PYTHON_INSTALL_DIR}/navground/core")
      set("DSV" "set;NAVGROUND_DLL_PATH;bin")
      file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv" "${DSV}")
      ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/plugins.dsv")
      # See https://github.com/ament/ament_package/issues/145
      file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/plugins.sh
           "# Dummy .sh file needed for .dsv file to be sourced.")
      ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/plugins.sh")
    endif()
  endif()

else()

  if(INSTALL_PYTHON_PKG)
    include(python_install_namespace_package)

    python_install_namespace_package(
      navground.core
      NAMESPACE
      navground
      VERSION
      "${CMAKE_PROJECT_VERSION}"
      TARGET
      _navground
      CONSOLE_SCRIPTS 
      )

  endif()

endif()

if(ament_cmake_FOUND)
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

if(PROJECT_IS_TOP_LEVEL)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cpack.cmake)
endif()


