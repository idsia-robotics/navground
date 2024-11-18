#include "navground/core/kinematics.h"

namespace core = navground::core;

struct MyKinematics : public core::Kinematics {

  // must override
  // return the nearest feasible twist to value.
  core::Twist2 feasible(const core::Twist2 &value) const override;

  // can override
  // return the nearest feasible twist to a target value from 
  // the current value in a time step, taking into account dynamic constraints.
  // core::Twist2 feasible_from_current(const core::Twist2 &value, const core::Twist2 &current, ng_float time_step) const override;

  // set to whether there are wheels or not
  static constexpr bool IS_WHEELED = false;

  // must override
  // return whether we are using wheels
  // should be a constant
  bool is_wheeled() const override { return IS_WHEELED; }

  // set the number of dof
  static constexpr unsigned DOF = 2;

  // must override
  // return the number of degree of freedom
  // should be a constant
  unsigned dof() const override { return DOF; }

  // should override
  float get_max_angular_speed() const override;

  // should override
  float get_max_speed() const override;
};