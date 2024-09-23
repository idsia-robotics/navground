add_executable(scenario_group scenario_with_group.cpp)
target_link_libraries(scenario_group navground_sim::navground_sim)

add_executable(scenario_yaml scenario_from_yaml.cpp)
target_link_libraries(scenario_yaml navground_sim::navground_sim)

add_executable(scenario_registered scenario_registered.cpp)
target_link_libraries(scenario_registered navground_sim::navground_sim)

add_executable(benchmark_sim benchmark_sim.cpp)
target_link_libraries(benchmark_sim navground_sim::navground_sim)

add_library(my_scenario SHARED my_scenario.cpp)
target_link_libraries(my_scenario navground_sim::navground_sim)
set_target_properties(my_scenario PROPERTIES LINKER_LANGUAGE CXX)
# ament_export_dependencies(my_scenario)

install(TARGETS
  scenario_group 
  scenario_yaml 
  scenario_registered
  benchmark_sim
  DESTINATION lib/${PROJECT_NAME}
)

install(
  TARGETS my_scenario
  EXPORT my_scenarioTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# install(FILES
#  ${PROJECT_BINARY_DIR}/my_scenario_export.h DESTINATION include/${PROJECT_NAME}
# )