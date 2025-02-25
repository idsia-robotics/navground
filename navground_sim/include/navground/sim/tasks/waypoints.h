/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASKS_WAYPOINTS_H_
#define NAVGROUND_SIM_TASKS_WAYPOINTS_H_

#include <limits>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/task.h"

namespace navground::sim {

/**
 * A sequence of points to reach.
 */
using Waypoints = std::vector<core::Vector2>;

/**
 * @brief      This class implement a task that makes the agent reach a sequence
 * of waypoints, calling \ref navground::core::Controller::go_to_position for
 * the next waypoint after the current has been reached within a tolerance.
 *
 * The task notifies when a new waypoint is set by calling a callback.
 *
 * *Registered properties*:
 *
 *   - `waypoints` (list of \ref navground::core::Vector2, \ref get_waypoints)
 *
 *   - `loop` (bool, \ref get_loop)
 *
 *   - `random` (bool, \ref get_random)
 *
 *   - `tolerance` (bool, \ref get_tolerance)
 */
struct NAVGROUND_SIM_EXPORT WaypointsTask : Task {
  static const std::string type;

  /**
   * Whether by default the task loops.
   */
  inline static const bool default_loop = true;
  /**
   * The default goal spatial tolerance.
   */
  inline static const ng_float_t default_tolerance = 1;
  /**
   * The default goal orientation tolerance.
   */
  inline static const ng_float_t default_angular_tolerance =
      std::numeric_limits<ng_float_t>::infinity();
  /**
   * By default moves in sequence.
   */
  inline static const bool default_random = false;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  waypoints  The waypoints
   * @param[in]  loop       Whether it should start from begin after reaching
   *                        the last waypoint
   * @param[in]  tolerance  The goal tolerance applied to each waypoint.
   * @param[in]  random     Whether to pick the next waypoint randomly
   * @param[in]  orientations  The goal orientation at the waypoints. 
   * @param[in]  angular_tolerance  The goal angular tolerance applied to each waypoint.
   */
  explicit WaypointsTask(
      const Waypoints &waypoints = {}, bool loop = default_loop,
      ng_float_t tolerance = default_tolerance, bool random = default_random,
      const std::vector<ng_float_t> &orientations = {},
      ng_float_t angular_tolerance = default_angular_tolerance)
      : Task(), _waypoints(waypoints), _orientations(orientations), _loop(loop),
        _tolerance(tolerance), _angular_tolerance(angular_tolerance),
        _random(random), _first(true), _index(-1), _running(false) {}

  virtual ~WaypointsTask() = default;

  /**
   * @brief      The size of the data passed to callbacks when events occur,
   *             see \ref TaskCallback and \ref Task::add_callback.
   *
   *             The data is composed of 4 numbers:
   *             ``[time, target_x, target_y, target_theta]``
   *
   * @return     4
   */
  size_t get_log_size() const override { return 4; }

  /**
   * @private
   */
  bool done() const override;

  /**
   * @brief      Sets the waypoints.
   *
   * @param[in]  value  The desired waypoints
   */
  void set_waypoints(const Waypoints &value) {
    _waypoints = value;
    _first = true;
  }
  /**
   * @brief      Sets the goal orientations at the waypoints.
   *
   * @param[in]  value  The desired orientations (in radians)
   */
  void set_orientations(const std::vector<ng_float_t> &values) {
    _orientations = values;
  }
  /**
   * @brief      Sets the goal tolerance applied to each waypoint.
   *
   * @param[in]  value  The desired value
   */
  void set_tolerance(ng_float_t value) {
    _tolerance = std::max<ng_float_t>(value, 0);
  }
  /**
   * @brief      Sets the goal angular tolerance applied to each waypoint.
   *
   * @param[in]  value  The desired value. A negative values equals to infinite
   * tolerance.
   */
  void set_angular_tolerance(ng_float_t value) { _angular_tolerance = value; }
  /**
   * @brief      Sets whether it should start from begin after reaching the last
   * waypoint
   *
   * @param[in]  value  The desired value
   */
  void set_loop(bool value) { _loop = value; }
  /**
   * @brief      Gets the waypoints.
   *
   * @return     The waypoints.
   */
  Waypoints get_waypoints() const { return _waypoints; }
  /**
   * @brief      Gets the goal orientations at the waypoints.
   *
   * @return     The orientations (in radians).
   */
  const std::vector<ng_float_t> &get_orientations() const {
    return _orientations;
  }
  /**
   * @brief      Gets the goal spatial tolerance applied to each waypoint.
   *
   * @return     The tolerance.
   */
  ng_float_t get_tolerance() const { return _tolerance; }
  /**
   * @brief      Gets the goal angular tolerance applied to each waypoint.
   *
   * Negative values equals infinite tolerance.
   *
   * @return     The tolerance.
   */
  ng_float_t get_angular_tolerance() const { return _angular_tolerance; }
  /**
   * @brief      Gets whether it should start from begin after reaching the last
   * waypoint.
   *
   * @return     True if it should loop.
   */
  bool get_loop() const { return _loop; }
  /**
   * @brief      Gets whether to pick the next waypoint randomly
   *
   * @return     True if it should pick randomly.
   */
  bool get_random() const { return _random; }
  /**
   * @brief      Sets whether to pick the next waypoint randomly
   *
   * @param[in]  value  The desired value
   */
  void set_random(bool value) { _random = value; }

protected:
  /**
   * @private
   */
  void update(Agent *agent, World *world, ng_float_t time) override;
  void prepare(Agent *agent, World *world) override;

private:
  Waypoints _waypoints;
  std::vector<ng_float_t> _orientations;
  bool _loop;
  ng_float_t _tolerance;
  ng_float_t _angular_tolerance;
  std::vector<ng_float_t> _tolerances;
  std::vector<ng_float_t> _angular_tolerances;
  bool _random;
  bool _first;
  int _index;
  bool _running;
  std::optional<core::Vector2> next_waypoint(World *world);
  std::optional<ng_float_t> next_goal_orientation() const;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_WAYPOINTS_H_ */
