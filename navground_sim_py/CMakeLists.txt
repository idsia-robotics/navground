cmake_minimum_required(VERSION 3.5)
project(navground_sim_py)

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

if (WIN32)
add_compile_options(/wd4251 /wd4996 /wd4250 /wd4244 /wd4267)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(navground_sim REQUIRED)
find_package(navground_core_py REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)

# Force to C++17
set(CMAKE_CXX_STANDARD 17)

ament_python_install_namespace_package(sim
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
  PACKAGE_DIR navground/sim
  NAMESPACE navground
)
include_directories(include ${PROJECT_BINARY_DIR})
pybind11_add_module(_navground_sim src/python.cpp)

generate_docstrings(
  _navground_sim 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp 
  ${CMAKE_INSTALL_PREFIX}/include/navground/sim
  ${CMAKE_INSTALL_PREFIX}/include 
  ${EIGEN3_INCLUDE_DIR}
)

# target_compile_options(_navground_sim PRIVATE -fvisibility=hidden)
target_link_libraries(_navground_sim 
  PRIVATE 
  navground_sim::navground_sim
)

target_link_libraries(_navground_sim INTERFACE navground_core_py)
target_compile_definitions(_navground_sim PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1) 
add_dependencies(_navground_sim navground_sim::navground_sim docstrings)


install(TARGETS _navground_sim DESTINATION "${PYTHON_INSTALL_DIR}/navground/sim")

IF (WIN32)
install(
	FILES 
	${CMAKE_INSTALL_PREFIX}/bin/navground_core.dll 
	${CMAKE_INSTALL_PREFIX}/bin/navground_sim.dll 
	${CMAKE_INSTALL_PREFIX}/bin/hdf5.dll 
  ${CMAKE_INSTALL_PREFIX}/bin/geos.dll 
	DESTINATION "${PYTHON_INSTALL_DIR}/navground/sim"
)
ENDIF()

ament_package()
