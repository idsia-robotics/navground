add_executable(scenario_group scenario_with_group.cpp)
target_link_libraries(scenario_group navground_sim::navground_sim)

add_executable(scenario_yaml scenario_from_yaml.cpp)
target_link_libraries(scenario_yaml navground_sim::navground_sim)

add_executable(scenario_registered scenario_registered.cpp)
target_link_libraries(scenario_registered navground_sim::navground_sim)

add_executable(benchmark_sim benchmark_sim.cpp)
target_link_libraries(benchmark_sim navground_sim::navground_sim)

install(TARGETS
  scenario_group 
  scenario_yaml 
  scenario_registered
  benchmark_sim
  DESTINATION lib/${PROJECT_NAME}
)