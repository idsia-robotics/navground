add_executable(controller controller.cpp)
target_link_libraries(controller navground_core::navground_core)

add_executable(controller_3d controller_3d.cpp)
target_link_libraries(controller_3d navground_core::navground_core)

add_executable(controller_async controller_async.cpp)
target_link_libraries(controller_async navground_core::navground_core)


install(TARGETS
  controller 
  controller_3d 
  controller_async
  DESTINATION lib/${PROJECT_NAME}
)
