cmake_minimum_required(VERSION 3.5)
project(navground_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include(GenerateExportHeader)
# include_directories(include ${PROJECT_BINARY_DIR})

find_package(navground_core REQUIRED)
find_package(navground_sim REQUIRED)

add_executable(native_cpp src/native.cpp)
target_link_libraries(native_cpp navground_core::navground_core)

add_library(navground_demos SHARED src/scenario.cpp)
target_link_libraries(navground_demos navground_sim::navground_sim)
# ament_export_dependencies(navground_demos)
set_target_properties(navground_demos PROPERTIES LINKER_LANGUAGE CXX)

# generate_export_header(navground_demos)
# ament_export_targets(navground_demosTargets HAS_LIBRARY_TARGET)

register_navground_plugins(
  TARGETS navground_demos
  DESTINATION $<IF:$<BOOL:${WIN32}>,bin,lib>
)

add_executable(sim_cpp src/sim.cpp)
# ament_target_dependencies(sim_cpp navground_core navground_sim)
target_link_libraries(sim_cpp navground_demos)

install(TARGETS
  native_cpp sim_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navground_demos
  EXPORT navground_demosTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(FILES
#  ${PROJECT_BINARY_DIR}/navground_demos_export.h DESTINATION include
# )

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if (ament_cmake_python_FOUND)
  ament_python_install_package(${PROJECT_NAME}
    SCRIPTS_DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "ament_cmake_python is missing: skip installing python package!")
endif()

if (ament_cmake_FOUND)
  ament_package()
endif()