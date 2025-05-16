add_library(my_behavior SHARED my_behavior.cpp my_behavior_group.cpp)
target_link_libraries(my_behavior navground_core::navground_core)
set_target_properties(my_behavior PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS my_behavior
  EXPORT my_behaviorTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)