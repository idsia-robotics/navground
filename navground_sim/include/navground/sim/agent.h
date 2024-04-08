/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_AGENT_H_
#define NAVGROUND_SIM_AGENT_H_

#include <memory>
#include <set>
#include <utility>
// #include <limits>

#include "navground/core/behavior.h"
#include "navground/core/common.h"
#include "navground/core/controller.h"
#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground/sim/entity.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/task.h"
#include "navground_sim_export.h"

using navground::core::Behavior;
using navground::core::Controller;
using navground::core::GeometricState;
using navground::core::Kinematics;
using navground::core::Neighbor;
using navground::core::Pose2;
using navground::core::Twist2;
using navground::core::Vector2;

namespace navground::sim {

/**
 * @brief      This class describes an agent.
 *
 * The agent navigates in the environment using
 * a task, a state estimation, a kinematic and a behavior,
 * and a controller.
 *
 * Agents have a circular shape which should match the shape of their navigation
 * \ref navground::core::Behavior.
 *
 * The role of task and state estimation is to provide goals and
 * environment state (perception) to the behavior.
 *
 * Agents have a public identifies \ref id that is accessible by the
 * other agents' state estimation and may be passed to their behavior as \ref
 * navground::core::Neighbor::id. This identifier may not be unique (e.g., may
 * be used to identifies *groups* of agents).
 *
 * Agents runs their update at the rate set by \ref control_period even if
 * the world is updated at a faster rate.
 */
class NAVGROUND_SIM_EXPORT Agent : public Entity {
 public:
  using C = std::shared_ptr<Agent>;
  using B = Behavior;
  using K = Kinematics;
  using T = Task;
  using S = StateEstimation;

  friend class World;
  friend struct StateEstimation;
  friend struct Task;

  virtual ~Agent() = default;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  radius          The radius of the agent
   * @param[in]  behavior        The behavior
   * @param[in]  kinematics      The kinematics
   * @param[in]  task            The task
   * @param[in]  estimation      The estimation
   * @param[in]  control_period  The control period
   * @param[in]  id              The public identifier
   */
  Agent(ng_float_t radius = 0, std::shared_ptr<Behavior> behavior = nullptr,
        std::shared_ptr<Kinematics> kinematics = nullptr,
        std::shared_ptr<Task> task = nullptr,
        std::shared_ptr<StateEstimation> estimation = nullptr,
        ng_float_t control_period = 0, unsigned id = 0)
      : Entity(),
        id(id),
        radius(radius),
        control_period(control_period),
        type(""),
        color(""),
        tags(),
        external(false),
        task(task),
        state_estimation(estimation),
        behavior(behavior),
        kinematics(kinematics),
        controller(behavior),
        control_deadline(0.0),
        collision_correction(),
        is_stuck_since_time(-1),
        ready(false) {}

  /**
   * @brief      Factory method to build an agent
   *
   * @param[in]  radius          The radius of the agent
   * @param[in]  behavior        The behavior
   * @param[in]  kinematics      The kinematics
   * @param[in]  task            The task
   * @param[in]  estimation      The estimation
   * @param[in]  control_period  The control period
   * @param[in]  id              The public identifier
   *
   * @return     A new agent
   */
  static std::shared_ptr<Agent> make(
      ng_float_t radius = 0, std::shared_ptr<Behavior> behavior = nullptr,
      std::shared_ptr<Kinematics> kinematics = nullptr,
      std::shared_ptr<Task> task = nullptr,
      std::shared_ptr<StateEstimation> estimation = nullptr,
      ng_float_t control_period = 0, unsigned id = 0) {
    return std::make_shared<Agent>(radius, behavior, kinematics, task,
                                   estimation, control_period, id);
  }

  /**
   * @brief      Returns a neighbor representation of the agent with the same
   * shape, position and id.
   *
   * @return     The neighbor representation
   */
  Neighbor as_neighbor() const {
    return Neighbor(pose.position, radius, twist.velocity, id);
  }

  /**
   * @brief      Returns a neighbor representation of the agent with the same
   * shape and id but with position translated by delta
   *
   * @param[in]  delta  The delta
   *
   * @return     The neighbor representation
   */
  Neighbor as_translated_neighbor(const Vector2 &delta) const {
    return Neighbor(pose.position + delta, radius, twist.velocity, id);
  }

  /**
   * @brief      Sets the state estimation.
   *
   * @param[in]  value  The desired value
   */
  void set_state_estimation(const std::shared_ptr<StateEstimation> &value) {
    state_estimation = value;
  }

  /**
   * @brief      Gets the state estimation.
   *
   * @return     The state estimation.
   */
  StateEstimation *get_state_estimation() const {
    return state_estimation.get();
  }

  /**
   * @brief      Sets the navigation behavior.
   *
   * Automatically set the behavior radius and kinematics to match the agent.
   *
   * @param[in]  value  The desired value
   */
  void set_behavior(const std::shared_ptr<Behavior> &value);

  /**
   * @brief      Gets the navigation behavior.
   *
   * @return     The navigation behavior.
   */
  Behavior *get_behavior() const { return behavior.get(); }

  /**
   * @brief      Sets the kinematics.
   *
   * @param[in]  value  The desired value
   */
  void set_kinematics(const std::shared_ptr<Kinematics> &value) {
    kinematics = value;
    if (behavior && !behavior->get_kinematics()) {
      behavior->set_kinematics(kinematics);
    }
  }

  /**
   * @brief      Gets the kinematics.
   *
   * @return     The kinematics.
   */
  Kinematics *get_kinematics() const { return kinematics.get(); }

  /**
   * @brief      Sets the task.
   *
   * @param[in]  value  The desired value
   */
  void set_task(const std::shared_ptr<Task> &value) { task = value; }

  /**
   * @brief      Gets the task.
   *
   * @return     The task.
   */
  Task *get_task() const { return task.get(); }

  /**
   * @brief      Gets the navigation controller.
   *
   * @return     The controller.
   */
  Controller *get_controller() { return &controller; }

  /**
   * @brief      Returns whether the task is done and the control is idle.
   *
   * @return     False if it has an active task or if the control is running
   */
  bool idle() const;

  /**
   * @brief      Gets the last command.
   *
   * @return     The controller.
   */
  Twist2 get_last_cmd(core::Frame frame) const;

  /**
   * @brief      Gets the last command.
   *
   * @return     The controller.
   */
  Twist2 get_last_cmd() const { return last_cmd; }

  /**
   * @brief      Sets the last command.
   *
   * @param[in]  value  The value
   *
   */
  void set_last_cmd(const Twist2 &value) { last_cmd = value; }

  /**
   * @brief      Gets the current pose.
   *
   * @return     The current pose.
   */
  Pose2 get_pose() const { return pose; }

  /**
   * @brief      Sets the current pose.
   *
   * @param[in]  value  The pose
   *
   */
  void set_pose(const Pose2 &value) { pose = value; }

  /**
   * @brief      Gets the current twist.
   *
   * @return     The current twist.
   */
  Twist2 get_twist() const { return twist; }

  /**
   * @brief      Sets the current twist.
   *
   * @param[in]  value  The value
   *
   */
  void set_twist(const Twist2 &value) { twist = value; }

 public:
  /**
   * The agent public identifier
   */
  unsigned id;
  /**
   * The agent radius
   */
  ng_float_t radius;
  /**
   * The control period
   */
  ng_float_t control_period;
  /**
   * The current pose
   */
  Pose2 pose;
  /**
   * The current twist
   */
  Twist2 twist;
  /**
   * The last control command
   */
  Twist2 last_cmd;
  /**
   * @brief The type of the agent.
   *
   * The agent type should not used by the neighbors state estimation.
   * It is mainly used internally to draw the agents in the UI.
   */
  std::string type;

  /**
   * @brief The color of the agent.
   *
   * A valid CSS color to fill the agent in the UI or empty to use the default
   * color.
   */
  std::string color;

  /**
   * @brief A set of tags used to label the agent.
   *
   * Tags are mainly used to add meta-information about an agent, for instance
   * by the \ref navground::sim::Scenario::Group that generated it (if any),
   * to simplify analysis.
   */
  std::set<std::string> tags;

  /**
   * @brief Whether the agent is controlled externally.
   *
   * External agents are not controlled by this simulation but are part of the
   * state estimation used by the agents controlled in this simulation.
   */
  bool external;

  /**
   * @brief      Actuate the current agent control command.
   *
   * @param[in]  dt    The time step
   * @param[in]  cmd The desired command
   */
  void actuate(const Twist2 &cmd, ng_float_t dt);

  /**
   * @brief      Determines if the agent has been stuck since a given time.
   *
   * @param[in]  time  The time
   *
   * @return     True if been stuck since, False otherwise.
   */
  bool has_been_stuck_since(ng_float_t time) const {
    return is_stuck_since_time >= 0 && is_stuck_since_time < time;
  }

  /**
   * @brief      Gets the time since the agents has been stuck.
   *
   * @return     The time since stuck.
   */
  ng_float_t get_time_since_stuck() const { return is_stuck_since_time; }

 private:
  std::shared_ptr<Task> task;
  std::shared_ptr<StateEstimation> state_estimation;
  std::shared_ptr<Behavior> behavior;
  std::shared_ptr<Kinematics> kinematics;
  Controller controller;
  ng_float_t control_deadline;
  Vector2 collision_correction;
  ng_float_t is_stuck_since_time;
  bool ready;

  /**
   * @brief      Tick the agent for a time step.
   *
   * @param[in]  dt    The time step
   * @param[in]  time    The current time
   * @param[in]  world    The world that the agent is part of
   */
  void update(ng_float_t dt, ng_float_t time, World *world);

  /**
   * @brief      Actuate the current agent control command.
   *
   * @param[in]  dt    The time step
   */
  void actuate(ng_float_t dt);

  /**
   * @brief      Prepare the agent for simulation (only called by world)
   *
   * @param      world  The world
   */
  void prepare(World *world);
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_AGENT_H_ */
