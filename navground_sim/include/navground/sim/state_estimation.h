/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATION_H_
#define NAVGROUND_SIM_STATE_ESTIMATION_H_

#include "navground/core/behavior.h"
#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/state.h"
#include "navground/sim/export.h"

using navground::core::Behavior;
using navground::core::EnvironmentState;
using navground::core::HasProperties;
using navground::core::HasRegister;

namespace navground::sim {

class Agent;
class World;

/**
 * @brief      This class describe a generic state estimation that should update
 * the environment state used by the agent \ref navground::core::Behavior.
 *
 * As the environment state is specialized by sub-classes of \ref
 * navground::core::Behavior like \ref navground::core::GeometricState, concrete
 * sub-classes have to target one or more of them.
 *
 * In particular, the agent should use a state estimation compatible with it's
 * state representation.
 *
 * \ref StateEstimation holds a pointer to the \ref World containing the agent,
 * which it queries to get the relevant entities (located nearby the agent).
 */
struct NAVGROUND_SIM_EXPORT StateEstimation
    : public virtual HasProperties,
      public virtual HasRegister<StateEstimation> {
  /**
   * @brief      Constructs a new instance.
   * @private
   */
  StateEstimation() {}
  virtual ~StateEstimation() = default;

  friend class Agent;
  friend class World;

  /**
   * @brief      Updates an environment state with respect to a given agent.
   * @param      agent  The agent owning the state estimation
   * @param[in]  world  The world that the agent is part of
   * @param      state  The environment state to be updated
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) {};

 protected:
  /**
   * @brief      Updates the state of a given agent \ref
   * navground::core::Behavior
   * @param      agent  The agent owning the state estimation
   * @param[in]  world  The world the agent is part of
   */
  void update(Agent *agent, World *world);

  /**
   * @brief      Setup the state estimation.
   * Called before starting a simulation.
   * @param      agent  The agent owning the state estimation
   * @param[in]  world    The world the agent is part of
   */
  virtual void prepare(Agent *agent, World *world) {};
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_STATE_ESTIMATION_H_ */
