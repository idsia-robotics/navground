/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_GEOMETRIC_BOUNDED_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_GEOMETRIC_BOUNDED_H_

#include <vector>

#include "navground/core/types.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

/**
 * @brief      Perfect state estimation within a range from the agent.
 *
 * *Registered properties*:
 *
 *   - `range` (float, \ref get_range), deprecated synonym `range_of_view`
 */
struct NAVGROUND_SIM_EXPORT BoundedStateEstimation : public StateEstimation {
  inline static const ng_float_t default_range = 1.0;
  inline static const bool default_update_static_obstacles = false;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range_  The range of view
   */
  BoundedStateEstimation(
      ng_float_t range_ = default_range,
      bool update_static_obstacles_ = default_update_static_obstacles)
      // float field_of_view_ = 0.0f,
      : StateEstimation(),
        // field_of_view(field_of_view_),
        range(range_),
        update_static_obstacles(update_static_obstacles_) {}

  virtual ~BoundedStateEstimation() = default;

  /**
   * @brief      Sets the maximal range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range(ng_float_t value) { range = value; }

  /**
   * @brief      Gets the maximal range of view.
   *
   * @return     The range of view.
   */
  ng_float_t get_range() const { return range; }

  /**
   * @brief      Sets whether to set the static obstacles
   * in ``prepare`` (ignoring the range) or in \ref update.
   *
   * @param[in]  value   True if static obstacles are set in \ref update
   */
  void set_update_static_obstacles(bool value) {
    update_static_obstacles = value;
  }

  /**
   * @brief      Gets whether to set the static obstacles
   * in ``prepare`` (ignoring the range) or in ``update``.
   *
   * @return     True if static obstacles are set in \ref update.
   */
  bool get_update_static_obstacles() const { return update_static_obstacles; }

  // void set_field_of_view(float v) { field_of_view = v; }

  // float get_field_of_view() const { return field_of_view; }

  /**
   * @private
   */
  virtual const Properties &get_properties() const override {
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
   * @brief      Gets the neighbors that lie within \ref get_range from the
   * agent.
   *
   * @param[in]  agent  The agent
   * @param[in]  world  The world the agent is part of.
   *
   * @return     A list of neighbors around the agent
   */
  virtual std::vector<Neighbor> neighbors_of_agent(const Agent *agent,
                                                   const World *world) const;

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) const override;

 protected:
#if 0
  BoundingBox bounding_box(const Agent *agent) const;

  virtual Neighbor perceive_neighbor(const Agent *agent,
                                     const Agent *neighbor) const;

  virtual bool visible(const Agent *agent, const Agent *neighbor) const;
#endif

  /**
   * @private
   */
  void prepare(Agent *agent, World *world) const override;

  /**
   * @brief      Gets the geometric state of an agent.
   *
   * @private
   *
   * @return     The geometric state or ``nullptr`` if the agent behavior does
   * not have a environment state that is a subclass of \ref
   * navground::core::GeometricState
   */
  GeometricState *get_geometric_state(Agent *agent) const {
    if (agent) {
      if (Behavior *behavior = agent->get_behavior()) {
        return dynamic_cast<GeometricState *>(
            behavior->get_environment_state());
      }
    }
    return nullptr;
  }

 private:
  // float field_of_view;
  ng_float_t range;
  bool update_static_obstacles;
  const static std::string type;
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_STATE_ESTIMATIONS_GEOMETRIC_BOUNDED_H_ */
