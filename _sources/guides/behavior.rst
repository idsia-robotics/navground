========= 
Behaviors 
=========

:cpp:func:`navground::core::Behavior::get_environment_state` 
:cpp:func:`navground::core::Behavior::compute_cmd_internal` 
:cpp:func:`navground::core::Behavior::cmd_twist_along_path` 
:cpp:func:`navground::core::Behavior::cmd_twist_towards_pose`
:cpp:func:`navground::core::Behavior::cmd_twist_towards_point` 
:cpp:func:`navground::core::Behavior::cmd_twist_towards_velocity` 
:cpp:func:`navground::core::Behavior::cmd_twist_towards_orientation` 
:cpp:func:`navground::core::Behavior::cmd_twist_towards_stopping`
:cpp:func:`navground::core::Behavior::desired_velocity_towards_point`
:cpp:func:`navground::core::Behavior::desired_velocity_towards_velocity`
:cpp:func:`navground::core::Behavior::twist_towards_velocity`

.. todo::

   - env state(may be own state) 
   - public vs inner cmd api 
   - complex, many way to specialize

.. code-block:: c++

   #include "navground/core/behavior.h"
   
                namespace core = navground::core;
   
   struct MyBehavior : public core::Behavior {
    
     // SHOULD override ... the base returns an null pointer. 
     EnvironmentState *get_environment_state() const {...}
   
     // CAN override
     Twist2 compute_cmd_internal(ng_float_t time_step, Frame frame) override {}
   
     // CAN override
     Twist2 cmd_twist_along_path(Path &path, ng_float_t speed,
                                 ng_float_t time_step, Frame frame) override {}
     // CAN override
     Twist2 cmd_twist_towards_pose(const Pose2 &pose, ng_float_t speed,
                                   Radians angular_speed, ng_float_t time_step,
                                   Frame frame) override {}
     // CAN override
     Twist2 cmd_twist_towards_point(const Vector2 &point, ng_float_t speed,
                                    ng_float_t time_step, Frame frame) override {}
     // CAN override
     Twist2 cmd_twist_towards_velocity(const Vector2 &velocity,
                                       ng_float_t time_step,
                                       Frame frame) override {}
     // CAN override
     Twist2 cmd_twist_towards_orientation(Radians orientation,
                                          ng_float_t angular_speed,
                                          ng_float_t time_step,
                                          Frame frame) override {}
     // CAN override
     Twist2 cmd_twist_towards_angular_speed(ng_float_t angular_speed,
                                            ng_float_t time_step,
                                            Frame frame) override {}
     // CAN override
     Twist2 cmd_twist_towards_stopping(ng_float_t time_step,
                                       Frame frame) override {}
     // CAN override
     Vector2 desired_velocity_towards_point(const Vector2 &point, ng_float_t speed,
                                            ng_float_t time_step) override {}
     // CAN override
     Vector2 desired_velocity_towards_velocity(const Vector2 &velocity,
                                               ng_float_t time_step) override {}
     // CAN override
     Twist2 twist_towards_velocity(const Vector2 &velocity, Frame frame) override {
   
     }
   };
