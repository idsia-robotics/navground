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
 * of waypoints, calling \ref navground::core::Controller::go_to_pose or
 * \ref navground::core::Controller::go_to_position for
 * the next waypoint after the current has been reached within a tolerance,
 * depending if a goal orientation has been specified using \ref set_orientations
 * or not.
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
 *
 *   - `tolerances` (list of float, \ref get_tolerances)
 *
 *   - `angular_tolerance` (float, \ref get_angular_tolerance)
 *
 *   - `angular_tolerances` (list of float, \ref get_angular_tolerances)
 *
 *   - `orientations` (list of float, \ref get_orientations)
 *
 *   - `wait_time` (float, \ref get_wait_time)
 *
 *   - `wait_times` (list of float, \ref get_wait_times)
 */
struct NAVGROUND_SIM_EXPORT WaypointsTask : Task {

  enum class State { idle, done, moving, waiting, waited };

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
   * @param[in]  waypoints          The waypoints
   * @param[in]  loop               Whether it should start from begin after
   *                                reaching the last waypoint
   * @param[in]  tolerance          The default goal tolerance applied to each
   *                                waypoint.
   * @param[in]  random             Whether to pick the next waypoint randomly
   * @param[in]  tolerances         The goal tolerance applied to specific
   *                                waypoints.
   * @param[in]  orientations       The goal orientation at the waypoints.
   * @param[in]  angular_tolerance  The default goal angular tolerance applied
   *                                to each waypoint.
   * @param[in]  angular_tolerances The goal angular tolerances applied to
   *                                specific waypoints.
   * @param[in]  wait_time          The time to wait at each waypoint.
   * @param[in]  wait_times         The time to wait at specific waypoint.
   */
  explicit WaypointsTask(
      const Waypoints &waypoints = {}, bool loop = default_loop,
      ng_float_t tolerance = default_tolerance, bool random = default_random,
      const std::vector<ng_float_t> &tolerances = std::vector<ng_float_t>{},
      const std::vector<ng_float_t> &orientations = {},
      ng_float_t angular_tolerance = default_angular_tolerance,
      const std::vector<ng_float_t> &angular_tolerances =
          std::vector<ng_float_t>{},
      ng_float_t wait_time = 0,
      const std::vector<ng_float_t> &wait_times = std::vector<ng_float_t>{})
      : Task(), _waypoints(waypoints), _orientations(orientations), _loop(loop),
        _tolerance(tolerance), _tolerances(tolerances),
        _angular_tolerance(angular_tolerance),
        _angular_tolerances(angular_tolerances), _wait_time(wait_time),
        _wait_times(wait_times), _random(random), _first(true), _index(-1),
        _state(State::idle) {}

  virtual ~WaypointsTask() = default;

  /**
   * @brief      The size of the data passed to callbacks when events occur,
   *             see \ref TaskCallback and \ref Task::add_callback.
   *
   * The data is composed of 7 numbers
   * ``[time, action_id, data_1, ..., data_5]``:
   *
   * - idle: ``[time, 0, 0, 0, 0, 0, 0]``
   *
   * - start waiting for t seconds: ``[time, 1, t, 0, 0, 0, 0]``
   *
   * - start moving towards pose (x, y, theta): ``[time, 2, x, y, theta,
   * tolerance, angular tolerance]``
   *
   * - start moving towards point (x, y): ``[time, 3, x, y, 0, tolerance, 0]``
   *
   * @return      7
   */
  size_t get_log_size() const override { return 7; }

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
   * If there are more orientations specified than waypoints,
   * those extra orientation are ignore.
   *
   * If there are less  orientations specified than waypoints,
   * the missing orientation are effectively filled with
   * the last orientation during control.
   *
   * To ignore the orientation at a specific  waypoint index,
   * set the related angular tolerance above PI.
   *
   * @param[in]  values  The desired orientations (in radians)
   */
  void set_orientations(const std::vector<ng_float_t> &values) {
    _orientations = values;
  }
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
  const Waypoints &get_waypoints() const { return _waypoints; }
  /**
   * @brief      Gets the goal orientations at the waypoints.
   *
   * If there are more orientations specified than waypoints,
   * those extra orientation are ignore.
   *
   * If there are less orientations specified than waypoints,
   * the missing orientation are effectively filled with
   * the last orientation during control.
   *
   * @return     The orientations (in radians).
   */
  const std::vector<ng_float_t> &get_orientations() const {
    return _orientations;
  }
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
  /**
   * @brief      Returns the spatial tolerance applied to
   * the waypoint at a given index.
   *
   * If a specific positive value is present at the same index in \ref
   * get_tolerances, it returns it.
   * Else it returns the default value from \ref get_tolerance.
   *
   * For example, if there are three waypoints,
   * the specific tolerances are set to ``-1, 0.2``,
   * and the default tolerance is set to ``0.25``,
   * the effective tolerances will be ``0.25, 0.2, 0.25``.
   *
   * @param[in]  index  The waypoint index
   *
   * @return     The tolerance in meters.
   */
  ng_float_t get_effective_tolerance(unsigned index) const {
    if (index < _tolerances.size() && _tolerances[index] > 0) {
      return _tolerances[index];
    }
    return _tolerance;
  }
  /**
   * @brief      Gets the default goal spatial tolerance.
   *
   * This value is used in \ref get_effective_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * specific (positive) tolerances returned by
   * \ref get_tolerances will overwrites this default value.
   *
   * @return     The default spatial tolerance in meters.
   */
  ng_float_t get_tolerance() const { return _tolerance; }
  /**
   * @brief      Gets the specific goal spatial tolerances.
   *
   * These values are used in \ref get_effective_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * negative values are ignored and replaced by the default value
   * \ref get_tolerance.
   * Extra items (not paired to \ref get_waypoints) are also ignored.
   *
   * @return     The spatial tolerances of waypoints at specific indices in
   * meters
   */
  const std::vector<ng_float_t> &get_tolerances() const { return _tolerances; }
  /**
   * @brief      Sets the default goal spatial tolerance.
   *
   * This value is used in \ref get_effective_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * specific (positive) tolerances set by
   * \ref set_tolerances will overwrites this default value.
   *
   * @param[in]  value  The desired positive value in meters.
   */
  void set_tolerance(ng_float_t value) {
    _tolerance = std::max<ng_float_t>(value, 0);
  }
  /**
   * @brief      Sets the specific goal spatial tolerances.
   *
   * These values are used in \ref get_effective_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * negative values are ignored and replaced by the default value set with
   * \ref set_tolerance.
   * Extra items (not paired to \ref get_waypoints) are also ignored.
   *
   * @param[in]  values  The desired values at specific indices in
   * meters
   */
  void set_tolerances(const std::vector<ng_float_t> &values) {
    _tolerances = values;
  }
  /**
   * @brief      Returns the angular tolerance applied to
   * the waypoint at a given index.
   *
   * If a specific positive value is present at the same index in \ref
   * get_angular_tolerances, it returns it.
   * Else it returns the default value from \ref get_angular_tolerance.
   *
   * For example, if there are three waypoints,
   * the specific angular tolerances are set to ``-1, 0.2``,
   * and the default angular tolerance is set to ``0.25``,
   * the effective angular tolerances will be ``0.25, 0.2, 0.25``.
   *
   * @param[in]  index  The waypoint index
   *
   * @return     The angular tolerance in radians.
   */
  ng_float_t get_effective_angular_tolerance(unsigned index) const {
    if (index < _angular_tolerances.size() && _angular_tolerances[index] > 0) {
      return _angular_tolerances[index];
    }
    return _angular_tolerance;
  }
  /**
   * @brief      Gets the default goal angular tolerance
   *
   * This value is used in \ref get_effective_angular_tolerance to compute
   * the effective angular tolerance applied to a selected waypoint:
   * specific (positive) angular tolerances returned by
   * \ref get_angular_tolerances will overwrites this default value.
   *
   * @return     The default angular tolerances in meters
   */
  ng_float_t get_angular_tolerance() const { return _angular_tolerance; }
  /**
   * @brief      Gets the specific goal angular tolerances.
   *
   * These values are used in \ref get_effective_angular_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * negative values are ignored and replaced by the default value
   * \ref get_angular_tolerance.
   * Extra items (not paired to \ref get_waypoints) are also ignored.
   *
   * @return     The specific waypoints angular tolerances.
   */
  const std::vector<ng_float_t> &get_angular_tolerances() const {
    return _angular_tolerances;
  }
  /**
   * @brief      Sets the goal angular tolerance applied to each waypoint.
   *
   * This value is used in \ref get_effective_angular_tolerance to compute
   * the effective angular tolerance applied to a selected waypoint:
   * specific (positive) angular tolerances set by
   * \ref set_angular_tolerances will overwrites this default value.
   *
   * @param[in]  value  The desired positive value.
   */
  void set_angular_tolerance(ng_float_t value) {
    _angular_tolerance = std::max<ng_float_t>(value, 0);
  }
  /**
   * @brief      Sets the specific goal angular tolerances.
   *
   * These values are used in \ref get_effective_angular_tolerance to compute
   * the effective angular tolerance applied to a selected waypoint:
   * negative values are ignored and replaced by the default value set with
   * \ref set_angular_tolerance.
   * Extra items (not paired to \ref get_waypoints) are also ignored.
   *
   * @param[in]  values  The desired values at specific indices in
   * radians.
   */
  void set_angular_tolerances(const std::vector<ng_float_t> &values) {
    _angular_tolerances = values;
  }
  /**
   * @brief      Gets the effective wait time before moving towards the waypoint
   * at given index.
   *
   * If a specific positive value is present at the same index in \ref
   * get_wait_times, it returns it.
   * Else it returns the default value from \ref get_wait_time.
   *
   * @param[in]  index  The index
   *
   * @return     The time in seconds.
   */
  ng_float_t get_effective_wait_time(unsigned index) const {
    if (index < _wait_times.size() && _wait_times[index] > 0) {
      return _wait_times[index];
    }
    return _wait_time;
  }
  /**
   * @brief      Gets the default wait time before moving towards a waypoint.
   *
   * This value is used in \ref get_effective_wait_time to compute
   * the effective wait time applied to a selected waypoint:
   * specific (positive) wait times returned by
   * \ref get_wait_times will overwrites this default value.
   * @return     The time in seconds.
   */
  ng_float_t get_wait_time() const { return _wait_time; }
  /**
   * @brief      Gets the wait times before moving towards specific waypoints.
   *
   * These values are used in \ref get_effective_angular_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * negative values are ignored and replaced by the default value
   * \ref get_angular_tolerance.
   * Extra items (not paired to \ref get_waypoints) are also ignored.
   *
   * @return     The wait times.
   */
  const std::vector<ng_float_t> &get_wait_times() const { return _wait_times; }
  /**
   * @brief      Sets the default wait time before moving towards a waypoint.
   *
   * This value is used in \ref get_effective_wait_time to compute
   * the effective wait time applied to a selected waypoint:
   * specific (positive) wait times returned by
   * \ref get_wait_times will overwrites this default value.
   *
   * @param[in]  value  The desired positive value.
   */
  void set_wait_time(ng_float_t value) {
    _wait_time = std::max<ng_float_t>(value, 0);
  }
  /**
   * @brief      Sets the wait times before moving towards specific waypoints.
   *
   * These values are used in \ref get_effective_angular_tolerance to compute
   * the effective tolerance applied to a selected waypoint:
   * negative values are ignored and replaced by the default value
   * \ref get_angular_tolerance.
   * Extra items (not paired to \ref get_waypoints) are also ignored.
   *
   * @param[in]  values  The values
   */
  void set_wait_times(const std::vector<ng_float_t> &values) {
    _wait_times = values;
  }
  /**
   * @brief      Gets the orientation at a given index.
   *
   * @param[in]  index  The index
   *
   * @return     The orientation or null the index is not valid.
   */
  std::optional<ng_float_t> get_orientation(unsigned index) const {
    if (index < _orientations.size()) {
      return _orientations[index];
    }
    return std::nullopt;
  }
  /**
   * @brief      Gets the waypoint at a given index
   *
   * @param[in]  index  The index
   *
   * @return     The waypoint or null the index is not valid.
   */
  std::optional<core::Vector2> get_waypoint(unsigned index) const {
    if (index < _waypoints.size()) {
      return _waypoints[index];
    }
    return std::nullopt;
  }

protected:
  /**
   * @private
   */
  void update(Agent *agent, World *world, ng_float_t time) override;
  /**
   * @private
   */
  void prepare(Agent *agent, World *world) override;

private:
  Waypoints _waypoints;
  std::vector<ng_float_t> _orientations;
  bool _loop;
  ng_float_t _tolerance;
  std::vector<ng_float_t> _tolerances;
  ng_float_t _angular_tolerance;
  std::vector<ng_float_t> _angular_tolerances;
  ng_float_t _wait_time;
  std::vector<ng_float_t> _wait_times;
  bool _random;
  bool _first;
  int _index;
  State _state;
  ng_float_t _deadline;
  bool next(World *world);
  core::Vector2 get_next_waypoint() const;
  std::optional<ng_float_t> get_next_goal_orientation() const;
  int get_next_wait_time() const;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_WAYPOINTS_H_ */
