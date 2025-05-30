/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <random>
#include <vector>

#include "navground/core/behavior_group.h"

namespace navground::core {

/**
 * This example shows how to define and use a group behavior.
 */

/**
 * @brief      A group behavior that makes all it's members
 * move at different speed but arriving at the same time at +x
 *
 */
class RandomSyncBehaviorGroup : public BehaviorGroup {
public:
  RandomSyncBehaviorGroup()
      : BehaviorGroup(), step(0), rg(), dist(0, 1), cmds() {}

protected:
  /**
   * @brief      Calculates the commands.
   *
   * Draws random commands every #behaviors steps then cycle them inside
   * the group.
   *
   * @param[in]  time_step  time step
   *
   * @return     The commands.
   */
  std::vector<Twist2> compute_cmds(ng_float_t) override {
    if (step % size() == 0 || size() != cmds.size()) {
      cmds.resize(size());
      std::transform(get_members().cbegin(), get_members().cend(), cmds.begin(),
                     [this](Behavior *behavior) {
                       return Twist2(
                           {dist(rg) * behavior->get_target_speed(), 0}, 0);
                     });

    } else {
      cmds.insert(cmds.begin(), cmds.back());
      cmds.pop_back();
    }
    step++;
    return cmds;
  }

private:
  unsigned step;
  std::mt19937 rg;
  std::uniform_real_distribution<ng_float_t> dist;
  std::vector<Twist2> cmds;
};

/**
 * @brief      Behavior that delegate the computation of its controller
 * to \ref RandomSyncBehaviorGroup
 *
 * It showcases how to define and use a group behavior.
 */
class RandomSyncBehavior : public BehaviorGroupMember {
public:
  using BehaviorGroupMember::BehaviorGroupMember;
  using BehaviorGroupMember::Groups;

protected:
  std::shared_ptr<BehaviorGroup> make_group() const override {
    return std::make_shared<RandomSyncBehaviorGroup>();
  }
  size_t get_group_hash() const override { return 0; }
};

} // namespace navground::core

using navground::core::OmnidirectionalKinematics;
using navground::core::RandomSyncBehavior;

int main(int, char *[]) {
  std::vector<RandomSyncBehavior> behaviors(10);
  ng_float_t y = 0;
  std::cout << "Start loop:" << std::endl;
  for (auto &behavior : behaviors) {
    behavior.set_kinematics(std::make_shared<OmnidirectionalKinematics>(
        static_cast<ng_float_t>(1), static_cast<ng_float_t>(1)));
    behavior.set_position({0, y});
    behavior.prepare();
    y += 1;
    std::cout << "-  " << behavior.get_position()[0] << " ("
              << behavior.get_velocity()[0] << ")" << std::endl;
  }

  const auto dt = static_cast<ng_float_t>(0.1);
  for (size_t i = 0; i < 10; i++) {
    for (auto &behavior : behaviors) {
      const auto cmd = behavior.compute_cmd(dt);
      behavior.actuate(cmd, dt);
    }
  }
  std::cout << "End loop:" << std::endl;
  for (auto &behavior : behaviors) {
    behavior.close();
    std::cout << "-  " << behavior.get_pose().position[0] << " ("
              << behavior.get_twist().velocity[0] << ")" << std::endl;
  }
  return 0;
}
