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
#include "navground_core_export.h"

namespace RVO {
class Agent;
class Obstacle;
}  // namespace RVO

namespace navground::core {

// TODO(J): complete the effective center -> use Omni.

/**
 * @brief      Optimal Reciprocal Collision Avoidance
 *
 * A wrapper of the open-source implementation from
 * http://gamma.cs.unc.edu/RVO2/
 *
 * *Registered properties*:
 *
 * - `time_horizon` (float, \ref get_time_horizon),
 *
 * - `effective_center` (float, \ref is_using_effective_center),
 *
 * *State*: \ref GeometricState
 */
class NAVGROUND_CORE_EXPORT ORCABehavior : public Behavior {
 public:
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
   * @brief      Determines if an effective center is being used.
   *
   * Using an effective center placed with an offset towards the front, allows
   * to consider the kinematics as holonomic instead of a two-wheeled
   * differential drive. See
   *
   *     J. Snape, J. van den Berg, S. J. Guy, and D. Manocha,
   *     "Smooth and collision-free navigation for multiple robots under
   *     differential-drive constraints," in 2010 IEEE/RSJ International
   *     Conference on Intelligent, Robots and Systems, 2010, pp. 4584–4589.
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
  virtual const Properties& get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static const std::map<std::string, Property> properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  EnvironmentState* get_environment_state() override { return &state; }

 protected:
  Twist2 twist_towards_velocity(const Vector2& absolute_velocity,
                                Frame frame) override;
  Vector2 desired_velocity_towards_point(const Vector2& point, ng_float_t speed,
                                         ng_float_t time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2& velocity,
                                            ng_float_t time_step) override;

 private:
  GeometricState state;
  bool use_effective_center;
  ng_float_t D;
  std::unique_ptr<RVO::Agent> _RVOAgent;
  std::vector<std::unique_ptr<const RVO::Agent>> rvo_neighbors;
  std::vector<std::unique_ptr<const RVO::Obstacle>> rvo_obstacles;

  void add_line_obstacle(const LineSegment& line);
  void add_neighbor(const Neighbor& disc, float range, bool push_away = false,
                    ng_float_t epsilon = 2e-3);
  void add_obstacle(const Disc& disc, float range, bool push_away = false,
                    ng_float_t epsilon = 2e-3);
  void prepare_line_obstacles();
  void prepare(const Vector2& target_velocity, ng_float_t dt);

  Vector2 effective_position() const;

 private:
  const static std::string type;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_ORCA_H_
