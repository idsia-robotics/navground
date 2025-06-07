cmake_minimum_required(VERSION 3.17)
project(navground_minimal_plugin_cpp VERSION 0.7.0)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(navground_core REQUIRED)
find_package(ament_cmake QUIET)

add_library(plugin SHARED src/plugin.cpp)
target_link_libraries(plugin navground_core::navground_core)
set_target_properties(plugin PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS plugin LIBRARY DESTINATION lib COMPONENT plugins)

register_navground_plugins(TARGETS plugin DESTINATION
                           $<IF:$<BOOL:${WIN32}>,bin,lib>)

set(CPACK_PRODUCTBUILD_IDENTIFIER "org.idsia.navground.minimal")
set(CPACK_PACKAGE_DESCRIPTION "A minimal example of a navground C++ plugin")
set(CPACK_PACKAGE_VENDOR "IDSIA")
set(CPACK_PACKAGE_CONTACT "Jérôme Guzzi, jerome@idsia.ch")

if(PROJECT_IS_TOP_LEVEL AND BUILD_TESTING)
  include(CTest)
  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()
endif()

if(PROJECT_IS_TOP_LEVEL)
  cpack_navground_plugins()
endif()
