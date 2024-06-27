/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_HL_H_
#define NAVGROUND_CORE_BEHAVIOR_HL_H_

#include <algorithm>
#include <memory>
#include <valarray>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/collision_computation.h"
#include "navground/core/property.h"
#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground_core_export.h"

// TODO(J): verify if behavior for tau < step is correct (non smooth)

namespace navground::core {

/**
 * @brief      Human-like obstacle avoidance behavior.
 *
 * The behavior inspired by how pedestrian move, has been originally presented
 * in
 *
 *     Guzzi, J.; Giusti, A.; Gambardella, L.M.; Theraulaz, G.; Di Caro, G.A.,
 *     "Human-friendly robot navigation in dynamic environments,"
 *     Robotics and Automation (ICRA), 2013 IEEE International Conference on,
 *     vol., no., pp.423,430, 6-10 May 2013
 *
 * *Registered properties*:
 *
 * - `tau` (float, \ref get_tau),
 *
 * - `eta` (float, \ref get_eta),
 *
 * - `aperture` (float, \ref get_aperture),
 *
 * - `resolution` (float, \ref get_resolution),
 *
 * - `epsilon` (float, \ref get_epsilon),
 *
 * - `barrier_angle` (float, \ref get_barrier_angle)
 *
 * *State*: \ref GeometricState
 */
class NAVGROUND_CORE_EXPORT HLBehavior : public Behavior {
 public:
  /**
   * Default \f$\eta\f$
   */
  static constexpr ng_float_t default_eta = 0.5;
  /**
   * Default \f$\tau\f$
   */
  static constexpr ng_float_t default_tau = 0.125;
  /**
   * Default aperture (full circular sector)
   */
  static constexpr ng_float_t default_aperture = M_PI;
  /**
   * Maximal resolution
   */
  static constexpr unsigned max_resolution = 361;
  /**
   * Default resolution. Should be less than \ref max_resolution
   */
  static constexpr unsigned default_resolution = 101;
  /**
   * Default epsilon.
   */
  static constexpr unsigned default_epsilon = 0;
  /**
   * Default barrier angle.
   */
  static constexpr ng_float_t default_barrier_angle = M_PI_2;

  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  HLBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
             ng_float_t radius = 0)
      : Behavior(kinematics, radius),
        effective_horizon(0),
        tau(default_tau),
        eta(default_eta),
        aperture(default_aperture),
        resolution(std::min(default_resolution, max_resolution)),
        epsilon(default_epsilon),
        barrier_angle(default_barrier_angle),
        collision_computation(),
        state(),
        cached_target_speed(0) {}
  ~HLBehavior() = default;

  // -------------------------- BEHAVIOR PARAMETERS

  /**
   * @brief      Gets the  time \f$\eta\f$ that the behavior keeps away from
   * collisions. Higher values lead to slower speeds.
   *
   * @return     \f$\eta\f$
   */
  ng_float_t get_eta() const { return eta; }
  /**
   * @brief      Sets the time \f$\eta\f$ that the behavior keeps away from
   * collisions. Higher values lead to slower speeds.
   *
   * @param[in]  value  A strict positive value.
   */
  void set_eta(ng_float_t value) { eta = value; }
  /**
   * @brief      Gets the relaxation time \f$\tau\f$. Higher values lead to
   * lower accelerations.
   *
   * @return     \f$\eta\f$
   */
  ng_float_t get_tau() const { return tau; }
  /**
   * @brief      Sets the relaxation time \f$\tau\f$. Higher values lead to
   * lower accelerations.
   *
   * @param[in]  value  A positive value. If zero, relaxation is disabled.
   */
  void set_tau(ng_float_t value) { tau = value; }
  /**
   * @brief      Gets the aperture \f$\alpha\f$: desired velocity is searched on
   * a circular sector \f$[-\alpha, \alpha]\f$.
   *
   * @return     The positive \f$\alpha\f$ in radians.
   */
  Radians get_aperture() const { return aperture; }
  /**
   * @brief      Sets the aperture, see \ref get_aperture
   *
   * @param[in]  value  A positive value
   */
  void set_aperture(Radians value) { aperture = value; }
  /**
   * @brief      Gets the number of subdivision of \f$[-\alpha, \alpha]\f$ to
   * search for optimal directions.
   *
   * @return     The resolution.
   */
  unsigned get_resolution() const { return resolution; }
  /**
   * @brief      Sets the number of subdivision of \f$[-\alpha, \alpha]\f$ to
   * search for optimal directions.
   *
   * @param[in]  value  A strict positive value. The larger the value, the more
   * precise the motion but also the more expensive the computations.
   */
  void set_resolution(unsigned value) {
    resolution = std::clamp<unsigned>(value, 1, max_resolution);
  }
  /**
   * @brief      Convenience method that return the size of an angular segment
   * in \f$[-\alpha, \alpha]\f$ to search for optimal directions.
   *
   * @return     The angular resolution for the optimal direction search in
   * radians.
   */
  Radians get_angular_resolution() const { return 2 * aperture / resolution; }

  /**
   * @brief      Gets the lowest margin to an obstacle or neighbor.
   *
   * Any obstacles nearer than this this value will be virtually pushing away
   * from the agent.
   *
   * @return     Epsilon.
   */
  ng_float_t get_epsilon() const { return epsilon; }
  /**
   * @brief      Sets the lowest margin to an obstacle or neighbor.
   *
   * Any obstacles nearer than this this value will be virtually pushing away
   * from the agent.
   *
   * @param[in]  value  Zero or negative values disable virtually pushing away
   * obstacles.
   */
  void set_epsilon(ng_float_t value) { epsilon = value; }

  /**
   * @brief      Gets the barrier angle, i.e., the minimal angle
   * with respect to a currently virtually colliding obstacle to ignore it.
   *
   * @return     Epsilon.
   */
  ng_float_t get_barrier_angle() const { return barrier_angle; }
  /**
   * @brief      Sets the barrier angle, i.e., the minimal angle
   * with respect to a currently virtually colliding obstacle to ignore it.
   *
   * @param[in]  value  A positive value. Higher values makes the agent more
   * cautious.
   */
  void set_barrier_angle(ng_float_t value) {
    barrier_angle = std::max<ng_float_t>(0, value);
  }

  /**
   * @brief      Gets the free distance to collision in \f$[-\alpha, \alpha]\f$
   * at regular intervals.
   *
   * @param[in]  assuming_static  If True, all obstacles are assumed static.
   * @param[in]  speed  The desired speed. Will be set to the last used target
   * speed if not specified.
   *
   * @return     A vector of distances of size \ref
   * get_resolution. Angles are in the agent frame.
   */
  std::valarray<ng_float_t> get_collision_distance(
      bool assuming_static = false,
      std::optional<ng_float_t> speed = std::nullopt);

  /**
   * @brief      Gets the angles in \f$[-\alpha, \alpha]\f$
   * at regular intervals used for collision checking
   *
   * @return     A vector of angles in the fixed frame of size \ref
   * get_resolution.
   */
  std::valarray<ng_float_t> get_collision_angles() const;

  /** @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * Properties: tau, eta, aperture, and resolution
   * @private
   */
  static const std::map<std::string, Property> properties;

  /** @private
   */
  virtual std::string get_type() const override { return type; }

  /** @private
   */
  EnvironmentState *get_environment_state() override { return &state; }

 protected:
  ng_float_t effective_horizon;
  ng_float_t tau;
  ng_float_t eta;
  Radians aperture;
  unsigned int resolution;
  ng_float_t epsilon;
  ng_float_t barrier_angle;
  CollisionComputation collision_computation;
  GeometricState state;
  ng_float_t cached_target_speed;

  /**
   * @brief      Override \ref Behavior::compute_cmd_internal adding target
   * velocity relaxation
   *
   * The target velocities (twist or wheel speeds, depending on the \ref
   * get_kinematics) are relaxed over time \f$\tau\f$ as \f$ \dot v = (v_t - v)
   * / \tau \f$, where \f$v_t\f$ is the instantaneous desired value computed by
   * \ref Behavior::compute_cmd.
   *
   * If \f$\tau=0\f$, no relaxation is performed and the desired target velocity
   * is returned.
   *
   */
  Twist2 compute_cmd_internal(ng_float_t time_step, Frame frame) override;

  Vector2 desired_velocity_towards_point(const Vector2 &value, ng_float_t speed,
                                         ng_float_t time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2 &value,
                                            ng_float_t time_step) override;

  void prepare(ng_float_t speed);
  Vector2 compute_repulsive_force(bool &inside_obstacle);
  Twist2 relax(const Twist2 &current_twist, const Twist2 &twist,
               ng_float_t dt) const;
  unsigned int index_of_relative_angle(Radians relative_angle);
  ng_float_t distance_to_segment(const LineSegment &line,
                                 Radians absolute_angle);
  ng_float_t dist_for_angle(const DiscCache *agent, Radians angle);
  ng_float_t compute_distance_to_collision_at_relative_angle(
      Radians relative_angle, ng_float_t *staticCache);
  ng_float_t feared_distance_to_collision_at_relative_angle(Radians angle);
  ng_float_t static_dist_for_angle(const DiscCache *agent, Radians angle);
  ng_float_t distance_to_collision_at_relative_angle(Radians angle);

  DiscCache make_neighbor_cache(const Neighbor &neighbor);
  DiscCache make_obstacle_cache(const Disc &obstacle);

 private:
  const static std::string type;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_HL_H_
