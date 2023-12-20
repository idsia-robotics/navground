add_executable(run run_with_custom_probes.cpp)
ament_target_dependencies(run navground_sim)

add_executable(experiment experiment_with_custom_probes.cpp)
ament_target_dependencies(experiment navground_sim)

install(TARGETS
  run
  experiment 
  DESTINATION lib/${PROJECT_NAME}
)
