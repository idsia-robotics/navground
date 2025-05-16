/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_EXAMPLES_MY_GROUP_BEHAVIOR_H_
#define NAVGROUND_CORE_EXAMPLES_MY_GROUP_BEHAVIOR_H_

#include "my_behavior_export.h"
#include "navground/core/behavior_group.h"
#include <algorithm>

namespace navground::core {

/**
 * @brief      A group behavior that makes all it's members
 * make still.
 *
 * It showcases how to define and use a group behavior.
 */
class MY_BEHAVIOR_EXPORT IdleBehaviorGroup : public BehaviorGroup {
public:
  using BehaviorGroup::BehaviorGroup;

protected:
  std::vector<Twist2> compute_cmds(ng_float_t) override {
    std::vector<Twist2> cmds(size());
    std::transform(get_members().cbegin(), get_members().cend(), cmds.begin(),
                   [](Behavior *) { return Twist2(); });
    return cmds;
  }
};

/**
 * @brief      Behavior that delegate the computation of its controller
 * to \ref IdleBehaviorGroup
 *
 * It showcases how to define and use a group behavior.
 */
class MY_BEHAVIOR_EXPORT IdleBehaviorGroupMember : public BehaviorGroupMember {
public:
  using BehaviorGroupMember::BehaviorGroupMember;
  using BehaviorGroupMember::Groups;
  static const std::string type;

protected:
  std::shared_ptr<BehaviorGroup> make_group() const override {
    return std::make_shared<IdleBehaviorGroup>();
  }
  size_t get_group_hash() const override { return 0; }
};

} // namespace navground::core

#endif // NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_
