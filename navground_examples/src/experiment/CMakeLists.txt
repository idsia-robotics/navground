add_executable(run run_with_custom_probes.cpp)
target_link_libraries(run navground_sim::navground_sim)

add_executable(experiment experiment_with_custom_probes.cpp)
target_link_libraries(experiment navground_sim::navground_sim)

install(TARGETS
  run
  experiment 
  DESTINATION lib/${PROJECT_NAME}
)
