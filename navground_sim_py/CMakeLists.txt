cmake_minimum_required(VERSION 3.11)
project(navground_sim_py VERSION 0.7.0)

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
# add_compile_options( -Wall -Wextra -Wpedantic
# -Wno-gnu-zero-variadic-macro-arguments -Wno-unused-parameter) endif()

# if(WIN32) add_compile_options(/wd4251 /wd4996 /wd4250 /wd4244 /wd4267) endif()

option(INSTALL_PYTHON_PKG "Whether to install the Python package"
       ${PROJECT_IS_TOP_LEVEL})
set(BUILD_PKG_DIRECTORY
    "${CMAKE_CURRENT_BINARY_DIR}"
    CACHE STRING "Where to build the extension")

find_package(navground_sim REQUIRED)

if(PROJECT_IS_TOP_LEVEL)
  find_package(navground_core_py REQUIRED)
  set(navground_core_py navground_core_py::navground_core_py)
else()
  set(navground_core_py navground_core_py)
endif()

set(Python3_FIND_STRATEGY LOCATION)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Force to C++17
set(CMAKE_CXX_STANDARD 17)

pybind11_add_module(_navground_sim src/python.cpp)

get_target_property(INCLUDE_DIRS navground_sim::navground_sim
                    INTERFACE_INCLUDE_DIRECTORIES)
list(GET INCLUDE_DIRS 0 INCLUDE_DIR)

include(generate_docstrings)
generate_docstrings(
  _navground_sim docstrings.h ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp
  ${INCLUDE_DIR}/navground/sim ${INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

# target_compile_options(_navground_sim PRIVATE -fvisibility=hidden)
target_link_libraries(_navground_sim PRIVATE navground_sim::navground_sim)

target_link_libraries(_navground_sim PRIVATE navground_sim::navground_sim
                                             ${navground_core_py})
target_compile_definitions(_navground_sim
                           PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
target_include_directories(_navground_sim PRIVATE ${PROJECT_BINARY_DIR})

if(MSVC)
  set(COMPILE_OPTIONS)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(COMPILE_OPTIONS)
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(COMPILE_OPTIONS "-Wno-unused-variable")
endif()

target_compile_options(_navground_sim PRIVATE ${COMPILE_OPTIONS})

set_target_properties(
  _navground_sim PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                            ${BUILD_PKG_DIRECTORY}/navground/sim)

set_target_properties(
  _navground_sim PROPERTIES LIBRARY_OUTPUT_DIRECTORY_RELEASE
                            ${BUILD_PKG_DIRECTORY}/navground/sim)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT error)
if(ipo_supported)
  set_property(TARGET _navground_sim PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/build_info.cmake)
configure_build_info(src/version.h.in version.h)

option(INSTALL_PYTHON_PKG "" ON)

if(ament_cmake_python_FOUND)
  if(INSTALL_PYTHON_PKG)
    include(ament_python_install_namespace_package)

    if(WIN32)
      set(scripts navground_py.exe navground_py-script.py)
    else()
      set(scripts navground_py)
    endif()

    ament_python_install_namespace_package(
      sim
      SCRIPTS_DESTINATION
      lib/${PROJECT_NAME}
      PACKAGE_DIR
      navground/sim
      NAMESPACE
      navground
      INSTALL_SCRIPTS_TO_BIN
      ${scripts})

    install(TARGETS _navground_sim
            DESTINATION "${PYTHON_INSTALL_DIR}/navground/sim")

    # if(WIN32) install( FILES ${CMAKE_INSTALL_PREFIX}/bin/navground_core.dll
    # ${CMAKE_INSTALL_PREFIX}/bin/ament_index_cpp.dll
    # ${CMAKE_INSTALL_PREFIX}/bin/navground_sim.dll
    # ${CMAKE_INSTALL_PREFIX}/bin/hdf5.dll ${CMAKE_INSTALL_PREFIX}/bin/geos.dll
    # DESTINATION "${PYTHON_INSTALL_DIR}/navground/sim") endif()

  endif()

else()
  if(INSTALL_PYTHON_PKG)
    include(python_install_namespace_package)

    python_install_namespace_package(
      navground.sim
      NAMESPACE
      navground
      VERSION
      "${CMAKE_PROJECT_VERSION}"
      TARGET
      _navground_sim
      DATA
      ui/templates/*
      CONSOLE_SCRIPTS
      "navground_py = navground.sim.main:main"
      )
  endif()

endif()

if(ament_cmake_FOUND)
  ament_package()
endif()

if(PROJECT_IS_TOP_LEVEL AND BUILD_TESTING)
  include(CTest)
  if (BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

