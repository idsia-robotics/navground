cmake_minimum_required(VERSION 3.5)
project(navground_py)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/../cmake)
include(ament_python_install_namespace_package)
include(generate_docstrings)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments)
endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-Os")

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(navground_core REQUIRED)

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
# add_subdirectory(../external/pybind11 pybind11 EXCLUDE_FROM_ALL)


# message("${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp ${CMAKE_INSTALL_DIR}/include/navground/core ${EIGEN3_INCLUDE_DIR}")


include_directories(include ${PROJECT_BINARY_DIR})

add_library(navground_py INTERFACE )
target_include_directories(navground_py INTERFACE include)

# ament_python_install_package seems to not support namespace packages
# For those: we should install the pkg to <site-packages>/navground
# and the metainformation in <site-packages>/core-0.0.0-py3.10.egg-info
# BUT the one below install both under DESTINATION
ament_python_install_namespace_package(core
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
  PACKAGE_DIR navground/core
  NAMESPACE navground
)

pybind11_add_module(_navground src/python.cpp)

generate_docstrings(
  _navground 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp 
  ${CMAKE_INSTALL_PREFIX}/include/navground/core
  ${CMAKE_INSTALL_PREFIX}/include
  ${EIGEN3_INCLUDE_DIR}
)

# target_compile_options(_navground PRIVATE -fvisibility=hidden)
target_compile_definitions(_navground PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
target_link_libraries(_navground PRIVATE navground_core::navground_core)
add_dependencies(_navground navground_core::navground_core docstrings)

install(TARGETS _navground DESTINATION "${PYTHON_INSTALL_DIR}/navground/core")

install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

IF (WIN32)
install(FILES ${CMAKE_INSTALL_PREFIX}/bin/navground_core.dll DESTINATION "${PYTHON_INSTALL_DIR}/navground/core")
ENDIF()

ament_package()
