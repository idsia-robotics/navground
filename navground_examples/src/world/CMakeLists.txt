add_executable(world world.cpp)
target_link_libraries(world navground_sim::navground_sim)

add_executable(world_yaml world_from_yaml.cpp)
target_link_libraries(world_yaml navground_sim::navground_sim)

add_executable(world_group world_with_group.cpp)
target_link_libraries(world_group navground_sim::navground_sim)

install(TARGETS
  world 
  world_yaml 
  world_group
  DESTINATION lib/${PROJECT_NAME}
)
