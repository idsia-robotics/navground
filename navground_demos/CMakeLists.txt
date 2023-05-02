 cmake_minimum_required(VERSION 3.5)
project(navground_demos)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(navground_core REQUIRED)
find_package(navground_sim REQUIRED)

add_executable(native_cpp src/native.cpp)
ament_target_dependencies(native_cpp navground_core)

add_executable(sim_cpp src/sim.cpp)
ament_target_dependencies(sim_cpp navground_core navground_sim)

add_library(navground_demos SHARED src/scenario.h)
ament_target_dependencies(navground_demos navground_core navground_sim)
set_target_properties(navground_demos PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS
  native_cpp sim_cpp
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  navground_demos
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)


ament_package()