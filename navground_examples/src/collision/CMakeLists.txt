add_executable(collision collision.cpp)
target_link_libraries(collision navground_core::navground_core)

add_executable(collision_cached cached_collision.cpp)
target_link_libraries(collision_cached navground_core::navground_core)

install(TARGETS
  collision 
  collision_cached 
  DESTINATION lib/${PROJECT_NAME}
)