#include "navground/core/behavior.h"

namespace core = navground::core;

// May use a custom environment state
struct MyEnvironmentState {

};

struct MyBehavior : public core::Behavior {
  MyEnvironmentState _env_state;
  // MUST override ... the base returns an null pointer.
  EnvironmentState *get_environment_state() const {return &_env_state; }

  // CAN override
  // executed before the first evaluation
  // void prepare() override;

  // CAN override
  // executed after the last evaluation
  // void close() override;

  // CAN override
  // core::Twist2 compute_cmd_internal(ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 cmd_twist_along_path(Path &path, ng_float_t speed, ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 cmd_twist_towards_pose(const Pose2 &pose, ng_float_t speed, Radians angular_speed, ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 cmd_twist_towards_point(const core::Vector2 &point, ng_float_t speed, ng_float_t time_step) override;
 
  // CAN override
  // core::Twist2 cmd_twist_towards_velocity(const core::Vector2 &velocity, ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 cmd_twist_towards_orientation(Radians orientation, ng_float_t angular_speed, ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 cmd_twist_towards_angular_speed(ng_float_t angular_speed, ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 cmd_twist_towards_stopping(ng_float_t time_step) override;
  
  // CAN override
  // core::Vector2 desired_velocity_towards_point(const core::Vector2 &point, ng_float_t speed, ng_float_t time_step) override;
  
  // CAN override
  // core::Vector2
  // desired_velocity_towards_velocity(const core::Vector2 &velocity, ng_float_t time_step) override;
  
  // CAN override
  // core::Twist2 twist_towards_velocity(const core::Vector2 &velocity) override;
};