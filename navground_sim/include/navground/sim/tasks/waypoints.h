/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASKS_WAYPOINTS_H_
#define NAVGROUND_SIM_TASKS_WAYPOINTS_H_

#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/task.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;
using navground::core::Vector2;

namespace navground::sim {

/**
 * A sequence of points to reach.
 */
using Waypoints = std::vector<navground::core::Vector2>;

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
  DECLARE_TYPE_AND_PROPERTIES
  
  /**
   * Whether by default the task loops.
   */
  inline static const bool default_loop = true;
  /**
   * The default goal tolerance.
   */
  inline static const bool default_tolerance = 1;
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
   */
  explicit WaypointsTask(Waypoints waypoints = {}, bool loop = default_loop,
                         ng_float_t tolerance = default_tolerance,
                         bool random = default_random)
      : Task(), _waypoints(waypoints), _loop(loop), _tolerance(tolerance),
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
   * @brief      Sets the goal tolerance applied to each waypoint.
   *
   * @param[in]  value  The desired value
   */
  void set_tolerance(ng_float_t value) {
    _tolerance = std::max<ng_float_t>(value, 0);
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
  Waypoints get_waypoints() const { return _waypoints; }
  /**
   * @brief      Gets the goal tolerance applied to each waypoint.
   *
   * @return     The tolerance.
   */
  ng_float_t get_tolerance() const { return _tolerance; }
  /**
   * @brief      Gets whether it should start from begin after reaching the last
   * waypoint.
   *
   * @return     True if it should loop.
   */
  ng_float_t get_loop() const { return _loop; }
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
  bool _loop;
  ng_float_t _tolerance;
  bool _random;
  bool _first;
  int _index;
  bool _running;
  std::optional<navground::core::Vector2> next_waypoint(World *world);
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_WAYPOINTS_H_ */
