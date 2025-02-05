add_library(my_scenario SHARED my_scenario.cpp)
target_link_libraries(my_scenario navground_sim::navground_sim)
set_target_properties(my_scenario PROPERTIES LINKER_LANGUAGE CXX)

install(
  TARGETS my_scenario
  EXPORT my_scenarioTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)