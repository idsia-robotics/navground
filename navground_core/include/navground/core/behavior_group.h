/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_GROUP_H_
#define NAVGROUND_CORE_BEHAVIOR_GROUP_H_

#include <map>
#include <memory>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/common.h"
#include "navground/core/export.h"
#include "navground/core/kinematics.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief  Behavior group holds a list of behaviors for which they compute
 * control commands at once.
 * 
 * 
 */
class NAVGROUND_CORE_EXPORT BehaviorGroup {
public:
  friend class BehaviorGroupMember;

  /**
   * @private
   */
  BehaviorGroup() : _behaviors(), _cmds() {}
  virtual ~BehaviorGroup() = default;
  /**
   * @brief      Gets the members of the group.
   *
   * @return     The behaviors.
   */
  const std::vector<Behavior *> &get_members() { return _behaviors; }
  /**
   * @brief      Returns the size of the group
   *
   * @return     The number of behaviors
   */
  size_t size() const;

protected:
  /**
   * @brief      Calculates the commands for all behaviors at once.
   * 
   * Commands should (but not strictly required) be feasible.
   * 
   * This method is called each time the behavior at the first index
   * requires a command.
   *
   * Users must override this method to define a \ref BehaviorGroup.
   *
   * @param[in]  time_step  The time step
   *
   * @return     The commands.
   */
  virtual std::vector<Twist2> compute_cmds(ng_float_t time_step) = 0;

private:
  std::vector<Behavior *> _behaviors;
  std::vector<Twist2> _cmds;

  bool has(Behavior *behavior) const;
  size_t index(Behavior *behavior) const;
  void add(Behavior *behavior);
  void remove(Behavior *behavior);
  Twist2 compute_cmd(ng_float_t time_step, Behavior *behavior);
};

/**
 * @brief  Members of a behavior group delegate the computation
 * of their control commands to the group.
 * 
 * \warning Users must call \ref Behavior::prepare before the first 
 * call to \ref Behavior::compute_cmd to setup the group and
 * call \ref Behavior::close after the last call to tear it down.
 */
class NAVGROUND_CORE_EXPORT BehaviorGroupMember : public Behavior {
public:
  /**
   * A map of groups indexed by an integer key, 
   * which may be computed using hashing. 
   */
  using Groups = std::map<size_t, std::shared_ptr<BehaviorGroup>>;
  /** @private **/
  BehaviorGroupMember(std::shared_ptr<Kinematics> kinematics = nullptr,
                      ng_float_t radius = 0)
      : Behavior(kinematics, radius), _group() {}
  virtual ~BehaviorGroupMember() = default;
  /** @private **/
  void prepare() override;
  /** @private **/
  void close() override;
  /**
   * @brief      Gets the group.
   *
   * @return     The group.
   */
  std::shared_ptr<BehaviorGroup> get_group() const { return _group; }

protected:
  /**
   * @brief      Makes a group.
   * 
   * Users must specialize this method when defining a new group/member pair.
   *
   * @return     The new group
   */
  virtual std::shared_ptr<BehaviorGroup> make_group() const = 0;
  /**
   * @brief      Gets the key associated to the group.
   * 
   * Users can specialize this method when defining a new group/member pair.
   * The default implementation returns 0, i.e., 
   * it groups all behaviors together.
   *
   * @return     The hash key.
   */
  virtual size_t get_group_hash() const { return 0; }
  /**
   * @brief      Returns all groups.
   * 
   * Users must specialize this method when defining a new group/member pair.
   *
   * @return     All groups.
   */  
  virtual Groups * get_groups();
  /** @private **/
  Twist2 compute_cmd_internal(ng_float_t time_step) override;
  /** @private **/
  void set_group(const std::shared_ptr<BehaviorGroup> & value);
  /** @private **/
  virtual void remove_group(size_t key);
  /** @private **/
  virtual std::shared_ptr<BehaviorGroup> get_or_create_group(size_t key);

private:
  std::shared_ptr<BehaviorGroup> _group;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_GROUP_H_
