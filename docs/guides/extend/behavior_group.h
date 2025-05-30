#include "navground/core/behavior_group.h"

namespace core = navground::core;

struct MyBehaviorGroup : public core::BehaviorGroup {
  // MUST override
  core::Twist2 compute_cmd_internal(ng_float_t time_step) override;
};