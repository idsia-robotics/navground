add_executable(behavior behavior.cpp)
target_link_libraries(behavior navground_core::navground_core)

add_executable(behavior_yaml echo_yaml.cpp)
target_link_libraries(behavior_yaml navground_core::navground_core)

add_executable(behavior_benchmark benchmark.cpp)
target_link_libraries(behavior_benchmark navground_core::navground_core)

add_library(my_behavior SHARED my_behavior.cpp)
target_link_libraries(my_behavior navground_core::navground_core)
set_target_properties(my_behavior PROPERTIES LINKER_LANGUAGE CXX)
# ament_export_dependencies(my_behavior)

install(TARGETS
  behavior 
  behavior_benchmark 
  behavior_yaml
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS my_behavior
  EXPORT my_behaviorTargets
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)

# install(FILES
#  ${PROJECT_BINARY_DIR}/my_behavior_export.h DESTINATION include/${PROJECT_NAME}
# )