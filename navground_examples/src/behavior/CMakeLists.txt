add_executable(behavior behavior.cpp)
target_link_libraries(behavior navground_core::navground_core)

add_executable(behavior_yaml echo_yaml.cpp)
target_link_libraries(behavior_yaml navground_core::navground_core)

add_executable(behavior_benchmark benchmark.cpp)
target_link_libraries(behavior_benchmark navground_core::navground_core)

add_executable(behavior_group behavior_group.cpp)
target_link_libraries(behavior_group navground_core::navground_core)

install(TARGETS
  behavior 
  behavior_benchmark 
  behavior_yaml
  behavior_group
  DESTINATION lib/${PROJECT_NAME}
)