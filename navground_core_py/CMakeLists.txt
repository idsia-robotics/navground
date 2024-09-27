cmake_minimum_required(VERSION 3.5)
project(navground_core_py)

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
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-gnu-zero-variadic-macro-arguments)
endif()

set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_RELEASE "-Os")

find_package(navground_core REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
# add_subdirectory(../external/pybind11 pybind11 EXCLUDE_FROM_ALL)


# message("${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp ${CMAKE_INSTALL_DIR}/include/navground/core ${EIGEN3_INCLUDE_DIR}")

add_library(navground_core_py INTERFACE )
target_include_directories(navground_core_py
  INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")

pybind11_add_module(_navground src/python.cpp)

include(generate_docstrings)
generate_docstrings(
  _navground 
  ${CMAKE_BINARY_DIR}/docstrings.h 
  ${CMAKE_CURRENT_SOURCE_DIR}/src/python.cpp 
  ${CMAKE_INSTALL_PREFIX}/include/navground/core
  ${CMAKE_INSTALL_PREFIX}/include
  ${EIGEN3_INCLUDE_DIR}
)

# target_compile_options(_navground PRIVATE -fvisibility=hidden)
target_compile_definitions(_navground PRIVATE PYBIND11_DETAILED_ERROR_MESSAGES=1)
target_link_libraries(_navground PRIVATE navground_core_py navground_core::navground_core)
target_include_directories(_navground PRIVATE ${PROJECT_BINARY_DIR})
add_dependencies(_navground navground_core_py)
set_target_properties(_navground PROPERTIES
         LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/navground/core)

install(
  DIRECTORY include/navground/core_py
  DESTINATION include/navground
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if (ament_cmake_python_FOUND)
  include(ament_python_install_namespace_package)

# ament_python_install_package seems to not support namespace packages
# For those: we should install the pkg to <site-packages>/navground
# and the meta information in <site-packages>/core-0.0.0-py3.10.egg-info
# BUT the one below install both under DESTINATION
  ament_python_install_namespace_package(core
    SCRIPTS_DESTINATION lib/${PROJECT_NAME}
    PACKAGE_DIR navground/core
    NAMESPACE navground
  )

  install(
    TARGETS _navground 
    DESTINATION "${PYTHON_INSTALL_DIR}/navground/core"
  )

  if (WIN32)
    install(
      FILES ${CMAKE_INSTALL_PREFIX}/bin/navground_core.dll 
            ${CMAKE_INSTALL_PREFIX}/bin/ament_index_cpp.dll 
      DESTINATION "${PYTHON_INSTALL_DIR}/navground/core"
    )
  endif()

else()

  include(python_install_namespace_package)

  python_install_namespace_package(
    navground.core
    NAMESPACE navground
    VERSION "0.1.0"
    TARGET _navground
  )
  
endif()

if (ament_cmake_FOUND)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_package()
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
