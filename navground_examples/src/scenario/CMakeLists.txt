add_executable(scenario_group scenario_with_group.cpp)
ament_target_dependencies(scenario_group navground_sim)

add_executable(scenario_yaml scenario_from_yaml.cpp)
ament_target_dependencies(scenario_yaml navground_sim)

add_executable(scenario_registered scenario_registered.cpp)
ament_target_dependencies(scenario_registered navground_sim)

add_executable(benchmark_sim benchmark_sim.cpp)
ament_target_dependencies(benchmark_sim navground_sim)

add_library(my_scenario SHARED my_scenario.h)
ament_target_dependencies(my_scenario navground_sim)
ament_export_dependencies(my_scenario)
set_target_properties(my_scenario PROPERTIES LINKER_LANGUAGE CXX)


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
