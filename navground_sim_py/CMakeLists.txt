cmake_minimum_required(VERSION 3.5)
project(navground_sim_py)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../cmake)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments
    -Wno-unused-parameter)
endif()

if(WIN32)
  add_compile_options(/wd4251 /wd4996 /wd4250 /wd4244 /wd4267)
endif()

find_package(navground_sim REQUIRED)
find_package(navground_core_py REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Force to C++17
set(CMAKE_CXX_STANDARD 17)

pybind11_add_module(_navground_sim src/python.cpp)

include(generate_docstrings)
generate_docstrings(
  _navground_sim
  ${CMAKE_BINARY_DIR}/docstrings.h
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp
  ${CMAKE_INSTALL_PREFIX}/include/navground/sim
  ${CMAKE_INSTALL_PREFIX}/include
  ${EIGEN3_INCLUDE_DIR})

# target_compile_options(_navground_sim PRIVATE -fvisibility=hidden)
target_link_libraries(_navground_sim PRIVATE navground_sim::navground_sim)

target_link_libraries(
  _navground_sim PRIVATE navground_sim::navground_sim
                         navground_core_py::navground_core_py)
target_compile_definitions(_navground_sim
                           PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
target_include_directories(_navground_sim PRIVATE ${PROJECT_BINARY_DIR})
set_target_properties(
  _navground_sim PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                            ${PROJECT_BINARY_DIR}/navground/sim)

include(${CMAKE_CURRENT_LIST_DIR}/../cmake/build_info.cmake)
configure_build_info(src/version.h.in version.h)

if(ament_cmake_python_FOUND)
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

  if(WIN32)
    install(
      FILES ${CMAKE_INSTALL_PREFIX}/bin/navground_core.dll
            ${CMAKE_INSTALL_PREFIX}/bin/ament_index_cpp.dll
            ${CMAKE_INSTALL_PREFIX}/bin/navground_sim.dll
            ${CMAKE_INSTALL_PREFIX}/bin/hdf5.dll
            ${CMAKE_INSTALL_PREFIX}/bin/geos.dll
      DESTINATION "${PYTHON_INSTALL_DIR}/navground/sim")
  endif()

else()

  include(python_install_namespace_package)

  python_install_namespace_package(
    navground.sim
    NAMESPACE
    navground
    VERSION
    "0.3.4"
    TARGET
    _navground_sim
    DATA
    ui/templates/*)

endif()

if(ament_cmake_FOUND)
  ament_package()
endif()
