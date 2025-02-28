cmake_minimum_required(VERSION 3.17)
project(navground_py VERSION 0.5.1)

set(Python_FIND_VIRTUALENV FIRST)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

set(BUILD_TESTING
    OFF
    CACHE INTERNAL "")

include(FetchContent)

FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG v2.13.6
  GIT_SHALLOW TRUE
  OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(pybind11)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../navground_core_py ${CMAKE_CURRENT_BINARY_DIR}/navground_core_py)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../navground_sim_py ${CMAKE_CURRENT_BINARY_DIR}/navground_sim_py)

