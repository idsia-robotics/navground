add_executable(behavior behavior.cpp)
ament_target_dependencies(behavior navground_core)

add_executable(behavior_yaml echo_yaml.cpp)
ament_target_dependencies(behavior_yaml navground_core)

add_executable(behavior_benchmark benchmark.cpp)
ament_target_dependencies(behavior_benchmark navground_core)

add_library(my_behavior SHARED my_behavior.h)
ament_target_dependencies(my_behavior navground_core)
ament_export_targets(my_behaviorTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(my_behavior)
set_target_properties(my_behavior PROPERTIES LINKER_LANGUAGE CXX)


install(TARGETS
  behavior 
  behavior_benchmark 
  behavior_yaml
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS my_behavior
  EXPORT my_behaviorTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)