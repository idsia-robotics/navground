/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_CONTROLLER_H_
#define NAVGROUND_CORE_CONTROLLER_H_

#include <algorithm>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/export.h"
#include "navground/core/types.h"

namespace navground::core {

class Controller;

/**
 * @brief      Holds the state of a long running task
 * and notifies observers when it terminates.
 */
struct NAVGROUND_CORE_EXPORT Action {
  /**
   * @brief      The state of an action.
   */
  enum class State {
    idle,    /**< the action has not started */
    running, /**< the action is running */
    failure, /**< the action failed */
    success  /**< the action succeeded */
  };

  /**
   * The current state
   */
  State state;

  Action() : state(State::idle), running_cb(), done_cb() {}

  /**
   * @brief      Update the action.
   *
   * Should only be called by the controller owning the action!
   *
   * @private
   * @param      controller  The controller ticking the action
   * @param[in]  dt          The time step
   */
  void update(Controller *controller, ng_float_t dt);

  /**
   * A callback called when the action is running
   *
   * The callback argument is the minimal expected time to terminate the action.
   */
  std::optional<std::function<void(ng_float_t)>> running_cb;
  /**
   * A callback called when the action terminates
   *
   * The callback argument is the final state of the action.
   */
  std::optional<std::function<void(State state)>> done_cb;

  /**
   * @brief      Return whenever the action is done or not.
   *
   * @return     True if the action failed or succeeded
   */
  bool done() const {
    return state == State::failure || state == State::success;
  }

  /**
   * @brief      Return whenever the action is running.
   *
   * @return     True if the action is running
   */
  bool running() const { return state == State::running; }

  /**
   * @brief      Abort the action, calling the \ref done_cb if set.
   */
  void abort();
  /**
   * @brief      Pure virtual method to tick the action
   * that has need to be overridden by sub-classes.
   *
   * @private
   * @param      controller  The controller ticking the action
   * @param[in]  dt          The time step
   *
   * @return     A measure of progress
   */
  virtual ng_float_t tick(Controller *controller, ng_float_t dt);

  virtual ~Action() { abort(); }
};

struct NAVGROUND_CORE_EXPORT FollowTwistAction : public Action {
  using Action::Action;
};

struct NAVGROUND_CORE_EXPORT FollowManualCommandAction : public Action {
  using Action::Action;

  Twist2 manual_cmd;
  explicit FollowManualCommandAction(const Twist2 &manual_cmd)
      : Action(), manual_cmd(manual_cmd) {}
};

struct NAVGROUND_CORE_EXPORT FollowAction : Action {
  using Action::Action;
};

struct NAVGROUND_CORE_EXPORT MoveAction : Action {
  using Action::Action;
  ng_float_t tick(Controller *controller, ng_float_t dt) override;
};

/**
 * @brief      This class exposes a higher level, stateful interface to a
 * behavior, to which it delegates 2D collision avoidance.
 *
 * The controller provides actions (event-based interfaces) to
 *
 * - go to a point/pose, stopping once the target has been reached,
 *
 * - follow a point/pose, which does not terminates once the target has been
 * reached
 *
 * - follow a velocity/twist
 *
 * *Typical usage of a controller*
 *
 * 1. Pick and configure a \ref Behavior
 *
 * 2. Initialize and configure the controller
 *
 * 3. At regular intervals, update the state, using the \ref Behavior API, and
 * call \ref update.
 *
 * 4. When needed, trigger one of the actions.
 *
 * 5. Either manually check the action's \ref get_state or set callbacks to be
 * notified when the action terminates or updates.
 *
 * 6. Actuate the target command by using the return value of \ref update
 *    or by setting a callback \ref set_cmd_cb
 */
class NAVGROUND_CORE_EXPORT Controller {
public:
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  behavior                  The navigation behavior
   */
  explicit Controller(std::shared_ptr<Behavior> behavior = nullptr)
      : behavior(behavior), speed_tolerance(1e-2), cmd_frame(std::nullopt) {}

  virtual ~Controller() = default;

  /**
   * @private
   */
  virtual ng_float_t estimate_time_until_target_satisfied() const {
    if (behavior) {
      return behavior->estimate_time_until_target_satisfied();
    }
    return std::numeric_limits<ng_float_t>::infinity();
  }

  /**
   * @private
   */
  virtual bool is_still() const {
    if (behavior) {
      return behavior->get_speed() < speed_tolerance;
    }
    return true;
  }

  /**
   * @brief      Gets the state of the control action.
   *
   * @return     The state.
   */
  Action::State get_state() const {
    if (action) {
      return action->state;
    }
    return Action::State::idle;
  }

  /**
   * @brief      Gets the navigation behavior used by the controller
   *
   * @return     The navigation behavior.
   */
  std::shared_ptr<Behavior> get_behavior() const { return behavior; }
  /**
   * @brief      Sets the navigation behavior used by the controller
   *
   * @param[in]  value  The navigation behavior
   */
  void set_behavior(std::shared_ptr<Behavior> value) { behavior = value; }
  /**
   * @brief      Gets the minimal speed to consider the agent as stopped.
   *
   * The default is 1 cm/s.
   *
   * @return     The speed tolerance.
   */
  ng_float_t get_speed_tolerance() const { return speed_tolerance; }
  /**
   * @brief      Sets the minimal speed to consider the agent as stopped.
   *
   * @param[in]  value  The speed tolerance
   */
  void set_speed_tolerance(ng_float_t value) {
    speed_tolerance = std::max<ng_float_t>(0, value);
  }

  /**
   * @brief      Gets the frame of reference for the command.
   *
   * @return     The frame of reference.
   */
  std::optional<Frame> get_cmd_frame() const { return cmd_frame; }

  /**
   * @brief      Sets the frame of reference for the command.
   *
   * @param[in]  frame  The desired value
   */
  void set_cmd_frame(const std::optional<Frame> &frame) { cmd_frame = frame; }

  /**
   * @brief      Starts an action to go to a point.
   *
   * The action succeed once the agent arrives within a tolerance
   * from the target point and comes to a stop.
   *
   * If an action is already running, the controller aborts it.
   *
   * @param[in]  point              The target point
   * @param[in]  tolerance  The spatial tolerance
   * @param[in]  along_path  An optional path to follow
   *
   * @return     The new action.
   */
  std::shared_ptr<Action>
  go_to_position(const Vector2 &point, ng_float_t tolerance,
                 std::optional<Path> along_path = std::nullopt);
  /**
   * @brief      Starts an action to go to a pose.
   *
   * The action succeed once the agent arrives within a tolerance
   * from the target pose and comes to a stop.
   *
   * If an action is already running, the controller aborts it.
   *
   * @param[in]  pose              The target pose
   * @param[in]  position_tolerance  The spatial tolerance
   * @param[in]  orientation_tolerance  The spatial tolerance
   * @param[in]  along_path  An optional path to follow
   *
   * @return     The new action.
   */
  std::shared_ptr<Action>
  go_to_pose(const Pose2 &pose, ng_float_t position_tolerance,
             ng_float_t orientation_tolerance,
             std::optional<Path> along_path = std::nullopt);
  /**
   * @brief     Start following a path.
   *
   * Internally calls \ref go_to_position to move towards the last point of the
   * path .
   *
   * @param[in]  path       The path
   * @param[in]  tolerance  The spatial tolerance
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_path(const Path &path, ng_float_t tolerance);
  /**
   * @brief      Starts an action to follow a point.
   *
   * The action keeps running even after the agent arrive at the target.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a point/pose, in which case it just updates the target.
   *
   * @param[in]  point              The target point
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_point(const Vector2 &point);
  /**
   * @brief      Starts an action to follow a pose
   *
   * The action keeps running even after the agent arrive at the target.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a point/pose, in which case it just updates the target.
   *
   * @param[in]  pose              The target pose
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_pose(const Pose2 &pose);
  /**
   * @brief      Starts an action to follow a target direction.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a velocity/twist, in which case it just updates the target.
   *
   * @param[in]  direction              The target direction. Must have positive
   * norm.
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_direction(const Vector2 &direction);
  /**
   * @brief      Starts an action to follow a target velocity.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a velocity/twist, in which case it just updates the target.
   *
   * @param[in]  velocity              The target velocity
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_velocity(const Vector2 &velocity);
  /**
   * @brief      Starts an action to follow a target twist.
   *
   * If an action is already running, the controller aborts it, unless it was
   * following a velocity/twist, in which case it just updates the target.
   *
   * @param[in]  twist              The target twist
   *
   * @return     The new action.
   */
  std::shared_ptr<Action> follow_twist(const Twist2 &twist);
  /**
   * @brief      Returns whether the control action is idle.
   *
   * @return     True only if the control action is not running.
   */
  bool idle() const { return action == nullptr || !action->running(); }

  /**
   * @brief      Abort the running action.
   */
  void stop();

  /**
   * @brief      Updates the control for time step.
   *
   * Internally calls \ref Behavior::compute_cmd for collision avoidance.
   *
   * @param[in]  time_step  The time step
   *
   * @return     The command twist to execute the current action
   */
  Twist2 update(ng_float_t time_step);
  /**
   * @brief      Sets the callback called each time a command is computed for an
   * active action.
   *
   * @param[in]  value  A callback that takes the command twist as argument.
   */
  void set_cmd_cb(std::function<void(const Twist2 &)> value) { cmd_cb = value; }

  std::shared_ptr<Action> follow_manual_cmd(const Twist2 &cmd);

protected:
  std::shared_ptr<Action> action;
  std::shared_ptr<Behavior> behavior;
  /**
   * Speed tolerance to transition to idle
   */
  ng_float_t speed_tolerance;
  std::optional<Frame> cmd_frame;

private:
  std::optional<std::function<void(const Twist2 &)>> cmd_cb;
};

} // namespace navground::core

#endif /* navground_CONTROLLER_H_ */
