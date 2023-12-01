/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASKS_WAYPOINTS_H_
#define NAVGROUND_SIM_TASKS_WAYPOINTS_H_

#include <vector>

#include "navground/core/common.h"
#include "navground/sim/task.h"
#include "navground_sim_export.h"

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
 */
struct NAVGROUND_SIM_EXPORT WaypointsTask : Task {

  /**
   * Whether by default the task loops.
   */
  inline static const bool default_loop = true;
  /**
   * The default goal tolerance.
   */
  inline static const bool default_tolerance = 1.0f;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  waypoints_  The waypoints
   * @param[in]  loop_       Whether it should start from begin after reaching
   *                         the last waypoint
   * @param[in]  tolerance_  The goal tolerance applied to each waypoint.
   */
  WaypointsTask(Waypoints waypoints_ = {}, bool loop_ = default_loop,
                float tolerance_ = default_tolerance)
      : Task(),
        waypoints(waypoints_),
        waypoint(waypoints.begin()),
        loop(loop_),
        tolerance(tolerance_) {}

  virtual ~WaypointsTask() = default;

  /**
   * @brief      The size of the data passed to callbacks when events occur,
   *             see \ref TaskCallback and \ref add_callback.
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
    waypoints = value;
    waypoint = waypoints.begin();
  }
  /**
   * @brief      Sets the goal tolerance applied to each waypoint.
   *
   * @param[in]  value  The desired value
   */
  void set_tolerance(float value) { tolerance = std::max(value, 0.0f); }
  /**
   * @brief      Sets whether it should start from begin after reaching the last
   * waypoint
   *
   * @param[in]  value  The desired value
   */
  void set_loop(bool value) { loop = value; }
  /**
   * @brief      Gets the waypoints.
   *
   * @return     The waypoints.
   */
  Waypoints get_waypoints() const { return waypoints; }
  /**
   * @brief      Gets the goal tolerance applied to each waypoint.
   *
   * @return     The tolerance.
   */
  float get_tolerance() const { return tolerance; }
  /**
   * @brief      Gets whether it should start from begin after reaching the last
   * waypoint.
   *
   * @return     True if it should loop.
   */
  float get_loop() const { return loop; }
  /**
   * @brief      Gets the properties.
   *
   * @private
   * @return     The properties.
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties = Properties{
      {"waypoints",
       make_property<Waypoints, WaypointsTask>(&WaypointsTask::get_waypoints,
                                               &WaypointsTask::set_waypoints,
                                               Waypoints{}, "waypoints")},
      {"loop", make_property<bool, WaypointsTask>(&WaypointsTask::get_loop,
                                                  &WaypointsTask::set_loop,
                                                  default_loop, "loop")},
      {"tolerance",
       make_property<float, WaypointsTask>(&WaypointsTask::get_tolerance,
                                           &WaypointsTask::set_tolerance,
                                           default_tolerance, "tolerance")},
  };

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 protected:
  /**
   * @private
   */
  void update(Agent *agent, World *world, float time) override;

 private:
  Waypoints waypoints;
  Waypoints::iterator waypoint;
  bool loop;
  float tolerance;
  bool running;
  inline const static std::string type =
      register_type<WaypointsTask>("Waypoints");
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_WAYPOINTS_H_ */
