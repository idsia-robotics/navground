/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASK_H_
#define NAVGROUND_SIM_TASK_H_

#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include <stdexcept>
#include <string>

using navground::core::HasProperties;
using navground::core::HasRegister;

namespace navground::sim {

class Agent;
class World;

/**
 * @brief      This class describe the high-level task control that provides
 * navigation goals.
 * 
 * Sub-classes are expected to override methods
 * 
 * - \ref prepare, which is called at the begin of the simulation to set it up.
 * - \ref update, which is called at every simulation step.
 * - \ref close, which is called at the end of the simulation.
 */
struct NAVGROUND_SIM_EXPORT Task : public virtual HasRegister<Task> {
  /**
   * The type of callbacks called when the task publishes data related to an
   * event.
   *
   * The task must publish data of the same size, see \ref get_log_size.
   *
   * @param[in] data The event payload
   */
  using TaskCallback = std::function<void(const std::vector<ng_float_t> &data)>;

  friend class Agent;
  friend class World;

  /**
   * @brief      Constructs a new instance.
   * @private
   */
  explicit Task() : callbacks() {}
  virtual ~Task() = default;

  /**
   * @brief      The size of the data passed to callbacks when events occur,
   *             see \ref TaskCallback and \ref add_callback.
   *
   * @return     The size of the data vector.
   */
  virtual size_t get_log_size() const { return 0; }

  /**
   * @brief      Adds a callback called to log task events.
   *
   * @param[in]  value  The desired callback
   */
  void add_callback(const TaskCallback &value) { callbacks.push_back(value); }

  /**
   * @brief      Remove any callbacks
   */
  void clear_callbacks() { callbacks.clear(); }

  /**
   * @brief      Returns whether the task is done.
   *
   * @return     True if the task has finished.
   */
  virtual bool done() const { return false; }

protected:
  /**
   * @brief      Tick the task, possibly updating the navigation goal of the
   * agent.
   *
   * @param      agent  The agent ticking the task
   * @param      world  The world the agent is part of
   * @param[in]  time   The simulation time
   */
  virtual void update(Agent *agent, World *world, ng_float_t time) {}

  /**
   * @brief      Setup the task.
   * 
   * Called before starting a simulation.
   * 
   * @param      agent  The agent owning the task
   * @param[in]  world  The world the agent is part of
   */
  virtual void prepare(Agent *agent, World *world) {};

  /**
   * @brief      Clean-up the task.
   * 
   * Called after finishing a simulation.
   */
  virtual void close() {};

  std::vector<TaskCallback> callbacks;

  /**
   * @brief      Log an event, forwarding data to registered callbacks
   *
   * @param[in]  log   The logged data
   */
  void log_event(const std::vector<ng_float_t> log) const {
    const unsigned u = get_log_size();
    if (u == log.size()) {
      for (const auto &cb : callbacks) {
        cb(log);
      }
    } else {
      throw std::runtime_error("Wrong log size: got " +
                               std::to_string(log.size()) + ", expected " +
                               std::to_string(u));
    }
  }
};
} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASK_H_ */
