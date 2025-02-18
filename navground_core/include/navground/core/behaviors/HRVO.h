/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_HRVO_H_
#define NAVGROUND_CORE_BEHAVIOR_HRVO_H_

#include <memory>

#include "navground/core/behavior.h"
#include "navground/core/property.h"
#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground/core/export.h"

namespace HRVO {
class Agent;
}

namespace navground::core {

// DONE(J 2023): verify DIFFERENTIAL_DRIVE ->
// no need to change anything as we take care of it directly

/**
 * @brief      Hybrid Velocity Obstacle Behavior
 *
 * A wrapper of the open-source implementation from
 * http://gamma.cs.unc.edu/HRVO/
 *
 * *Registered properties*:
 *
 * - `uncertainty_offset` (float, \ref get_uncertainty_offset)
 * 
 * - `max_neighbors` (int, \ref get_max_number_of_neighbors)
 *
 * *State*: \ref GeometricState
 */
class NAVGROUND_CORE_EXPORT HRVOBehavior : public Behavior {
 public:
  static const std::string type;

  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  HRVOBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
               ng_float_t radius = 0);
  ~HRVOBehavior();

  /**
   * @private
   */
  EnvironmentState* get_environment_state() override { return &state; }

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
   * @brief      Gets the uncertainty offset.
   *
   * @return     The uncertainty offset.
   */
  ng_float_t get_uncertainty_offset() const;
  /**
   * @brief      Sets the uncertainty offset.
   *
   * @param[in]  value  The value
   */
  void set_uncertainty_offset(ng_float_t value);

 protected:
  Vector2 desired_velocity_towards_point(const Vector2& point, ng_float_t speed,
                                         ng_float_t time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2& velocity,
                                            ng_float_t time_step) override;

 private:
  GeometricState state;
  uint agentIndex;
  // float rangeSq;
  std::unique_ptr<HRVO::Agent> _HRVOAgent;
  void add_neighbor(const Neighbor& neighbor, float rangeSq,
                    bool push_away = false, ng_float_t epsilon = 2e-3);
  void add_obstacle(const Disc& disc, float rangeSq, bool push_away = false,
                    ng_float_t epsilon = 2e-3);
  void prepare_eval(const Vector2& target_velocity);
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_HRVO_H_
