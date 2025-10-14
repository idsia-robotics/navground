/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASKS_GO_TO_POSE_H_
#define NAVGROUND_SIM_TASKS_GO_TO_POSE_H_

#include <limits>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/task.h"
#include "navground/sim/tasks/waypoints.h"

namespace navground::sim {

/**
 * @brief      This class offers a simplified interface to \ref
 * navground::sim::WaypointsTask to go to a single point/pose.
 *
 * *Registered properties*:
 *
 *   - `point` (\ref navground::core::Vector2, \ref get_point)
 *
 *   - `orientation` (float, \ref get_orientation)
 *
 *   - `tolerance` (bool, \ref navground::sim::WaypointsTask::get_tolerance)
 *
 *   - `angular_tolerance` (float, \ref
 * navground::sim::WaypointsTask::get_angular_tolerance)
 *
 */
struct NAVGROUND_SIM_EXPORT GoToPoseTask : WaypointsTask {
  static const std::string type;

  /**
   * The default goal spatial tolerance.
   */
  inline static const ng_float_t default_tolerance = 1;
  /**
   * The default goal orientation tolerance.
   */
  inline static const ng_float_t default_angular_tolerance =
      std::numeric_limits<ng_float_t>::infinity();

  inline static const core::Vector2 default_point = core::Vector2::Zero();
      
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  point              The goal point
   * @param[in]  orientation        The goal orientation
   * @param[in]  tolerance          The goal spatial tolerance
   * @param[in]  angular_tolerance  The goal angular tolerance
   */
  explicit GoToPoseTask(
      const core::Vector2 &point = core::Vector2::Zero(),
      ng_float_t orientation = 0, ng_float_t tolerance = default_tolerance,
      ng_float_t angular_tolerance = default_angular_tolerance)
      : WaypointsTask(std::vector<core::Vector2>{point}, false, tolerance,
                      false, std::vector<ng_float_t>{},
                      std::vector<ng_float_t>{orientation}, angular_tolerance,
                      std::vector<ng_float_t>{}) {}

  /**
   * @brief      Sets the goal point.
   *
   * @param[in]  point  The desired point
   */
  void set_point(const core::Vector2 &point) {
    set_waypoints(std::vector<core::Vector2>{point});
  }
  /**
   * @brief      Sets the goal orientations
   *
   * @param[in]  value  The desired orientations (in radians)
   */
  void set_orientation(ng_float_t value) {
    set_orientations(std::vector<ng_float_t>{value});
  }
  /**
   * @brief      Gets the goal point.
   *
   * @return     The waypoints.
   */
  core::Vector2 get_point() const {
    const auto ws = get_waypoints();
    if (ws.size()) {
      return ws[0];
    }
    return core::Vector2::Zero();
  }
  /**
   * @brief      Gets the goal orientations at the waypoints.
   *
   * @return     The orientations (in radians).
   */
  ng_float_t get_orientation() const {
    const auto os = get_orientations();
    if (os.size()) {
      return os[0];
    }
    return 0;
  }
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_GO_TO_POSE_H_ */
