#include "navground/core/behavior_modulation.h"

namespace core = navground::core;

struct MyBehaviorModulation : public core::BehaviorModulation {
  // CAN override
  // modulates the behavior by setting parameters while caching their original value
  // void pre(core::Behavior &behavior, ng_float_t time_step) override;
 
  // CAN override
  // modulates the cmd and resets parameters to their original values
  // core::Twist2 post(core::Behavior &behavior, ng_float_t time_step, const core::Twist2 & cmd) override;
};