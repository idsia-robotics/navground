=====================
navground_coppeliasim
=====================

CoppeliaSim plugin.


LUA API
=======



.. lua:function:: simNavground.make_controller(string behavior,map kinematics,float radius)-> int handle

   Instantiate a navigation controller


   :param behavior: The obstacle avoidance behavior
   :param kinematics: The kinematics
   :param radius: The radius

   :return: - **handle** -- An handle that identifies the controller
            



.. lua:function:: simNavground.get_property(int handle,string name)-> map value

   Get the value of one of the navigation behavior properties


   :param handle: The controller handle
   :param name: The property name

   :return: - **value** -- The property value
            



.. lua:function:: simNavground.set_property(int handle,string name,map value)

   Set the value of one of the navigation behavior properties


   :param handle: The controller handle
   :param name: The property name
   :param value: The property value




.. lua:function:: simNavground.go_to_position(int handle,float[] position,float tolerance)

   TODO


   :param handle: The controller handle
   :param position: The target position
   :param tolerance: The target tolerance




.. lua:function:: simNavground.go_to_pose(int handle,float[] position,float orientation,float position_tolerance,float orientation_tolerance)

   TODO


   :param handle: The controller handle
   :param position: The target position
   :param orientation: The target orientation
   :param position_tolerance: The target tolerance
   :param orientation_tolerance: The target tolerance




.. lua:function:: simNavground.follow_point(int handle,float[] point)

   TODO


   :param handle: The controller handle
   :param point: The target position




.. lua:function:: simNavground.follow_pose(int handle,float[] position,float orientation)

   TODO


   :param handle: The controller handle
   :param position: The target position
   :param orientation: The target orientation




.. lua:function:: simNavground.set_pose(int handle,float[] position,float orientation)

   TODO


   :param handle: The controller handle
   :param position: The 3d position
   :param orientation: The orientation in radians




.. lua:function:: simNavground.set_twist(int handle,float[] velocity,float angular_speed)

   TODO


   :param handle: The controller handle
   :param velocity: The 3d velocity
   :param angular_speed: The angular speed in radians/s




.. lua:function:: simNavground.set_rotation_tau(int handle,float value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.set_horizon(int handle,float value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.set_safety_margin(int handle,float value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.set_optimal_speed(int handle,float value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.set_heading_behavior(int handle,int value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.set_speed_tolerance(int handle,float value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.should_be_limited_to_2d(int handle,bool value)

   TODO


   :param handle: The controller handle
   :param value: The value




.. lua:function:: simNavground.set_cmd_frame(int handle,int value)

   TODO


   :param handle: The controller handle
   :param value: The value (0 for relative, 1 for absolute)




.. lua:function:: simNavground.update(int handle,float time_step)-> float[] velocity,float angular_speed,float state

   TODO


   :param handle: The controller handle
   :param time_step: The time step

   :return: - **velocity** -- The 3d velocity
            - **angular_speed** -- The angular speed in radians/s
            - **state** -- The angular speed in radians/s
            



.. lua:function:: simNavground.set_static_obstacles(int handle,map[] obstacles)

   TODO


   :param handle: The controller handle
   :param obstacles: The controller handle




.. lua:function:: simNavground.set_neighbors(int handle,map[] neighbors)

   TODO


   :param handle: The controller handle
   :param neighbors: The obstacles




.. lua:function:: simNavground.set_line_obstacles(int handle,map[] obstacles)

   TODO


   :param handle: The controller handle
   :param obstacles: The lines




.. lua:function:: simNavground.get_state(int handle)-> int state

   TODO


   :param handle: The controller handle

   :return: - **state** -- TODO
            



.. lua:function:: simNavground.get_actuated_wheel_speeds(int handle)-> float[] speeds

   TODO


   :param handle: The controller handle

   :return: - **speeds** -- TODO
            



.. lua:function:: simNavground.add_obstacle(int handle,float radius)

   TODO


   :param handle: The object handle
   :param radius: The object radius




.. lua:function:: simNavground.add_agent_from_yaml(int handle,string yaml)-> int handle

   TODO


   :param handle: The object handle
   :param yaml: The yaml text

   :return: - **handle** -- The agent handle
            



.. lua:function:: simNavground.get_last_cmd(int handle,int frame)-> float[] velocity,float angular_speed

   TODO


   :param handle: The agent handle
   :param frame: The value (0 for relative, 1 for absolute)

   :return: - **velocity** -- The horizontal velocity
            - **angular_speed** -- The angular speed in radians/s
            



.. lua:function:: simNavground.get_last_wheel_cmd(int handle)-> float[] speeds

   TODO


   :param handle: The agent handle

   :return: - **speeds** -- TODO
            



.. lua:function:: simNavground.enable_recording(map config)

   TODO


   :param config: The recording configuration


