/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_ORCA_H_
#define NAVGROUND_CORE_BEHAVIOR_ORCA_H_

#include <memory>

#include "navground/core/behavior.h"
#include "navground/core/property.h"
#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground/core/export.h"

namespace RVO {
class Agent;
class Obstacle;
}  // namespace RVO

namespace navground::core {

// TODO(J): complete the effective center -> use Omni.

/**
 * @brief      Optimal Reciprocal Collision Avoidance
 *             (see http://gamma.cs.unc.edu/RVO2)
 *
 * A wrapper of the open-source implementation
 * from https://github.com/snape/RVO2 
 *
 *
 * *Registered properties*:
 *
 * - `time_horizon` (float, \ref get_time_horizon),
 *
 * - `effective_center` (float, \ref is_using_effective_center),
 *
 * - `treat_obstacles_as_agents` (bool, \ref get_treat_obstacles_as_agents)
 * 
 * - `max_neighbors` (int, \ref get_max_number_of_neighbors)
 * 
 * *State*: \ref GeometricState
 */
class NAVGROUND_CORE_EXPORT ORCABehavior : public Behavior {
 public:
  static const std::string type;

  /**
   * @brief  A line 
   */
  struct Line {
    /**
     * A point on the line
     */
    Vector2 point;
    /**
     * The direction
     */
    Vector2 direction;
  };

  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  ORCABehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
               ng_float_t radius = 0);
  ~ORCABehavior();

  // ---------------------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the maximal number of neighbors
   *
   * @return     The maximal number of neighbors
   */
  unsigned get_max_number_of_neighbors() const;
  /**
   * @brief      Sets the maximal number of neighbors
   *
   * @param[in]  value The desired value
   */
  void set_max_number_of_neighbors(unsigned value);

  /**
   * @brief      Gets whether to treat obstacles as static, disc-shaped, [RVO]
   * agents. Else will treat them as squared obstacles.
   *
   * @return     True if static obstacles are passed as [RVO] neighbors.
   */
  bool get_treat_obstacles_as_agents() const;
  /**
   * @brief      Sets whether to treat obstacles as static, disc-shaped, [RVO]
   * agents. Else will treat them as squared obstacles.
   *
   * @param[in]  value The desired value
   */
  void set_treat_obstacles_as_agents(bool value);

  /**
   * @brief      Gets the time horizon. Collisions predicted to happen after
   * this time are ignored
   *
   * @return     The time horizon.
   */
  ng_float_t get_time_horizon() const;
  /**
   * @brief      Sets the time horizon. Collisions predicted to happen after
   * this time are ignored
   *
   * @param[in]  value
   */
  void set_time_horizon(ng_float_t value);

  /**
   * @brief      Gets the time horizon applied to static linear obstacles.
   * Collisions predicted to happen after this time are ignored
   *
   * @return     The time horizon.
   */
  ng_float_t get_static_time_horizon() const;
  /**
   * @brief      Sets the time horizon applied to static linear obstacles.
   * Collisions predicted to happen after this time are ignored
   *
   * @param[in]  value
   */
  void set_static_time_horizon(ng_float_t value);

  /**
   * @brief      Determines if an effective center is being used.
   *
   * Using an effective center placed with an offset towards the front, allows
   * to consider the kinematics as holonomic instead of a two-wheeled
   * differential drive. See
   *
   *     J. Snape, J. van den Berg, S. J. Guy, and D. Manocha,
   *     "Smooth and collision-free navigation for multiple robots under
   *     differential-drive constraints," in 2010 IEEE/RSJ International
   *     Conference on Intelligent, Robots and Systems, 2010, pp. 4584-4589.
   *
   * with ``D=L/2``.
   *
   * Only possibly true if the  kinematics is wheeled and constrained.
   *
   * @return     True if using effective center, False otherwise.
   */
  bool is_using_effective_center() const {
    if (!kinematics) return false;
    return use_effective_center && kinematics->is_wheeled() &&
           kinematics->dof() == 2;
  }
  /**
   * @brief      Specifies if the kinematics should be using an shifted
   * effective center, see \ref set_time_horizon
   *
   * @param[in]  value Whenever is should use an effective center when
   * kinematics is wheeled and constrained.
   */
  void should_use_effective_center(bool value) { use_effective_center = value; }

  // void set_time_step(ng_float_t value);
  // ng_float_t get_time_step() const;

  /**
   * @private
   */
  EnvironmentState* get_environment_state() override { return &state; }

  /**
   * @brief      Gets the ORCA lines computed during the last update
   *
   * @return     The ORCA lines.
   */
  std::vector<Line> get_lines() const;

 protected:
  Twist2 twist_towards_velocity(const Vector2& absolute_velocity) override;
  Vector2 desired_velocity_towards_point(const Vector2& point, ng_float_t speed,
                                         ng_float_t time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2& velocity,
                                            ng_float_t time_step) override;

 private:
  GeometricState state;
  bool use_effective_center;
  bool treat_obstacles_as_agents;
  ng_float_t D;
  std::unique_ptr<RVO::Agent> _RVOAgent;
  std::vector<std::unique_ptr<const RVO::Agent>> rvo_neighbors;
  std::vector<std::unique_ptr<const RVO::Agent>> rvo_static_obstacles;
  std::vector<std::unique_ptr<const RVO::Obstacle>> rvo_line_obstacles;
  std::vector<std::unique_ptr<const RVO::Obstacle>> rvo_square_obstacles;

  void add_line_obstacle(const LineSegment& line);
  void add_neighbor(const Neighbor& disc, bool push_away = false,
                    ng_float_t epsilon = 2e-3);
  void add_obstacle_as_agent(const Disc& disc, bool push_away = false,
                             ng_float_t epsilon = 2e-3);
  void add_obstacle_as_square(const Disc& disc, bool push_away = false,
                              ng_float_t epsilon = 2e-3);
  void prepare_line_obstacles();
  void prepare_eval(const Vector2& target_velocity);

  Vector2 effective_position() const;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_ORCA_H_
