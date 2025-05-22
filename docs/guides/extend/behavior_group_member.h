#include "navground/core/behavior_group.h"

namespace core = navground::core;

struct MyBehaviorGroupMember : public core::BehaviorGroupMember {
  using core::BehaviorGroupMember::Groups;
  
  // MUST override
  Groups * get_groups() const override;

  // MUST override
  std::shared_ptr<core::BehaviorGroup> * make_group() const override;

  // CAN override
  // size_t get_group_hash() const override;
};