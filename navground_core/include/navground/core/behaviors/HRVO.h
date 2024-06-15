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
#include "navground_core_export.h"

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
 * *State*: \ref GeometricState
 */
class NAVGROUND_CORE_EXPORT HRVOBehavior : public Behavior {
 public:
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
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  EnvironmentState* get_environment_state() override { return &state; }

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
  void prepare(const Vector2& target_velocity);

 private:
  const static std::string type;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_HRVO_H_
