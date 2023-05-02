add_executable(controller controller.cpp)
ament_target_dependencies(controller navground_core)

add_executable(controller_3d controller_3d.cpp)
ament_target_dependencies(controller_3d navground_core)

add_executable(controller_async controller_async.cpp)
ament_target_dependencies(controller_async navground_core)


install(TARGETS
  controller 
  controller_3d 
  controller_async
  DESTINATION lib/${PROJECT_NAME}
)
