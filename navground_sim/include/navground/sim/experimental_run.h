/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *  (Run)
 */

#ifndef NAVGROUND_SIM_EXPERIMENTAL_RUN_H_
#define NAVGROUND_SIM_EXPERIMENTAL_RUN_H_

#include <chrono>
#include <highfive/H5File.hpp>
#include <memory>

#include "navground/core/types.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

namespace navground::sim {

using Shape = std::vector<ssize_t>;

struct Experiment;

/**
 * @brief      Holds which data to record during an experimental run
 */
struct RecordConfig {
  /**
   * Whether to record the simulation time
   */
  bool time;
  /**
   * Whether to record the agents poses
   */
  bool pose;
  /**
   * Whether to record the agents twists
   */
  bool twist;
  /**
   * Whether to record the agents control commands
   */
  bool cmd;
  /**
   * Whether to record the agents targets
   */
  bool target;
  /**
   * Whether to record collisions
   */
  bool collisions;
  /**
   * Whether to record safety violations
   */
  bool safety_violation;
  /**
   * Whether to record data from task events
   */
  bool task_events;
  /**
   * Whether to record the time since when agents are stuck
   */
  bool deadlocks;
  /**
   * Whether to record efficacy (i.e., fraction of actual velocity vs
   * optimal velocity)
   */
  bool efficacy;

  /**
   * @brief      Sets all to record or not record.
   *
   * @return     The configuration.
   */
  static RecordConfig all(bool value) {
    return {value, value, value, value, value,
            value, value, value, value, value};
  }

  /**
   * @brief      Sets all to record or not record.
   *
   * @param[in]  value  The desired value
   */
  void set_all(bool value) { *this = all(value); }
};

/**
 * @brief      Group together parameters to run an experimental run
 */
struct RunConfig {
  /**
   * Simulation time step
   */
  ng_float_t time_step;
  /**
   * Maximal number of steps to perform
   */
  unsigned steps;
  /**
   * Whether to terminate when no progress is possible
   */
  bool terminate_when_all_idle_or_stuck;
};

/**
 * @brief      Simulates a world and collects data.
 */
class NAVGROUND_SIM_EXPORT ExperimentalRun {
  friend struct Experiment;

  /**
   * @brief      The state of the run
   *
   * @private
   */
  enum class State { init, running, finished };

 public:
  /**
   * @brief      Construct an \ref ExperimentalRun
   *
   * @param[in]  world          The world to simulate
   * @param[in]  run_config     The run configuration
   * @param[in]  record_config  The record configuration
   * @param[in]  seed           The seed used to initialize the world
   */
  ExperimentalRun(std::shared_ptr<World> world, const RunConfig& run_config,
                  const RecordConfig& record_config, unsigned seed = 0)
      : _state(State::init),
        _record_config(record_config),
        _run_config(run_config),
        _seed(seed),
        _world(world),
        _steps(0),
        _number(0),
        _record(false),
        _pose_data(),
        _twist_data(),
        _cmd_data(),
        _target_data(),
        _safety_violation_data(),
        _collisions_data(),
        _task_events_data(),
        _task_events(),
        _deadlock_data(),
        _efficacy_data(),
        _indices() {
    prepare();
  }

  /**
   * @brief     Construct an \ref ExperimentalRun
   *
   * @param[in]  world                             The world to simulate
   * @param[in]  time_step                         The duration of a simulation
   * step
   * @param[in]  max_steps                         The maximum number of steps
   * to perform
   * @param[in]  terminate_when_all_idle_or_stuck  Whether to terminate when all
   * agents idle or stuck
   * @param[in]  record_config                     The record configuration
   * @param[in]  seed                              The seed used to initialize
   * the world
   */
  ExperimentalRun(std::shared_ptr<World> world, ng_float_t time_step,
                  unsigned max_steps, bool terminate_when_all_idle_or_stuck,
                  const RecordConfig& record_config, unsigned seed = 0)
      : ExperimentalRun(
            world, {time_step, max_steps, terminate_when_all_idle_or_stuck},
            record_config, seed) {}

  /**
   * @brief      Gets the real-time duration of the run.
   *
   * @return     The duration in ns or 0 if the run is not yet finished.
   */
  std::chrono::nanoseconds get_duration_ns() const {
    if (has_finished()) {
      return _end - _begin;
    }
    return std::chrono::nanoseconds(0);
  }
  /**
   * @brief      Returns the simulated world.
   *
   * @return     The simulated world.
   */
  std::shared_ptr<World> get_world() const { return _world; }

  // void set_world(const std::shared_ptr<World>& value) { world = value; }

  /**
   * @brief      Whether the run is finished.
   *
   * @return     True if finished, False otherwise.
   */
  bool has_finished() const { return _state == State::finished; }

  /**
   * @brief      Gets the time step used for simulation.
   *
   * @return     The time step.
   */
  ng_float_t get_time_step() const { return _run_config.time_step; }
  /**
   * @brief      Gets the maximal number of steps to simulate.
   *
   * @return     The maximal number of steps.
   */
  ng_float_t get_maximal_steps() const { return _run_config.steps; }
  /**
   * @brief      Gets whether to terminate when all agents are idle or stuck.
   *
   * @return     Whether to terminate when all agents are idle or stuck.
   */
  bool get_terminate_when_all_idle_or_stuck() const {
    return _run_config.terminate_when_all_idle_or_stuck;
  }

  /**
   * @brief      Gets the recorded times.
   *
   * @return     The recorded times data.
   */
  const std::vector<ng_float_t>& get_time_data() const { return _time_data; }
  /**
   * @brief      Gets the shape of the recorded times.
   *
   * @return     The tensor shape.
   */
  Shape get_time_shape() const {
    if (_record_config.time) {
      return {_steps};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded poses.
   *
   * @return     The recorded poses data.
   */
  const std::vector<ng_float_t>& get_pose_data() const { return _pose_data; }
  /**
   * @brief      Gets the shape of the recorded poses.
   *
   * @return     The tensor shape.
   */
  Shape get_pose_shape() const {
    if (_record_config.pose) {
      return {_steps, _number, 3};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded twists.
   *
   * @return     The recorded twists data.
   */
  const std::vector<ng_float_t>& get_twist_data() const { return _twist_data; }
  /**
   * @brief      Gets the shape of the recorded twists.
   *
   * @return     The tensor shape.
   */
  Shape get_twist_shape() const {
    if (_record_config.twist) {
      return {_steps, _number, 3};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded commands.
   *
   * @return     The recorded commands data.
   */
  const std::vector<ng_float_t>& get_cmd_data() const { return _cmd_data; }
  /**
   * @brief      Gets the shape of the recorded commands.
   *
   * @return     The tensor shape.
   */
  Shape get_cmd_shape() const {
    if (_record_config.cmd) {
      return {_steps, _number, 3};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded targets.
   *
   * @return     The recorded targets data.
   */
  const std::vector<ng_float_t>& get_target_data() const {
    return _target_data;
  }
  /**
   * @brief      Gets the shape of the recorded targets.
   *
   * @return     The tensor shape.
   */
  Shape get_target_shape() const {
    if (_record_config.target) {
      return {_steps, _number, 3};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded safety margin violations.
   *
   * @return     The recorded safety margin violations data.
   */
  const std::vector<ng_float_t>& get_safety_violation_data() const {
    return _safety_violation_data;
  }
  /**
   * @brief      Gets the shape of the recorded safety margin violations.
   *
   * @return     The tensor shape.
   */
  Shape get_safety_violation_shape() const {
    if (_record_config.safety_violation) {
      return {_steps, _number};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded collisions.
   *
   * @return     The recorded collisions data.
   */
  const std::vector<unsigned>& get_collisions_data() const {
    return _collisions_data;
  }
  /**
   * @brief      Gets the shape of the recorded collisions.
   *
   * @return     The tensor shape.
   */
  Shape get_collisions_shape() const {
    if (_record_config.collisions) {
      return {static_cast<ssize_t>(_collisions_data.size()) / 3, 3};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded task logs.
   *
   * @return     The recorded task logs data.
   */
  const std::vector<ng_float_t>* get_task_data(const Agent* agent) const {
    const auto index = index_of_agent(agent);
    if (index) {
      return &_task_events_data.at(*index);
    }
    return nullptr;
  }
  /**
   * @brief      Gets the shape of the recorded task logs.
   *
   * @return     The tensor shape.
   */
  Shape get_task_shape(const Agent* agent) const {
    if (!_record_config.task_events) return {};
    const auto index = index_of_agent(agent);
    if (index) {
      const auto& data = _task_events_data.at(*index);
      const auto n = _task_events.at(*index);
      const ssize_t m = n ? data.size() / n : 0;
      return {n, m};
    }
    return {};
  }
  /**
   * @brief      Gets the recorded deadlocks.
   *
   * @return     The recorded deadlocks data.
   */
  const std::vector<ng_float_t>& get_deadlock_data() const {
    return _deadlock_data;
  }
  /**
   * @brief      Gets the shape of the recorded deadlocks.
   *
   * @return    The tensor shape.
   */
  Shape get_deadlock_shape() const {
    if (!_record_config.deadlocks) return {};
    return {static_cast<ssize_t>(_deadlock_data.size())};
  }
  /**
   * @brief      Gets the recorded efficacy.
   *
   * @return     The recorded efficacy data.
   */
  const std::vector<ng_float_t>& get_efficacy_data() const {
    return _efficacy_data;
  }
  /**
   * @brief      Gets the shape of the recorded efficacy.
   *
   * @return     The tensor shape.
   */
  Shape get_efficacy_shape() const {
    if (!_record_config.efficacy) return {};
    return {_steps, _number};
  }

  /**
   * @brief      Gets the number of recorded steps.
   *
   * @return     The recorded steps.
   */
  unsigned get_recorded_steps() const { return _steps; }
  /**
   * @brief      Gets the recorded number of agents.
   *
   * @return     The number of agents.
   */
  unsigned get_number_of_agents() const { return _number; }
  /**
   * @brief      Gets the seed used to initialize the simulation.
   *
   * @return     The seed.
   */
  unsigned get_seed() const { return _seed; }
  /**
   * @brief      Gets the record configuration.
   *
   * @return     The record configuration.
   */
  RecordConfig get_record_config() const { return _record_config; }

  /**
   * @brief      Associate an index to a given agent.
   *
   * Data related to agents is stored in this order.
   * For example, poses at a given time step are stored
   * as ``[[x_0, y_0, z_0], [x_1, y_1, z_1], ...]``
   *
   * where the pose of agent a is at the index returned by this function.
   *
   * @param[in]  agent  The agent
   *
   * @return     The index of data related to this agent.
   */
  std::optional<unsigned> index_of_agent(const Agent* agent) const {
    if (_indices.count(agent)) {
      return _indices.at(agent);
    }
    return std::nullopt;
  }

  /**
   * @brief      Signals that has simulation has started.
   *
   * Note that this won't neither execute the simulation nor record data.
   * It will just record the a time stamp and change the state to running.
   *
   * You don't need to call \ref start, \ref update, \ref stop, or \ref run
   * when running an \ref Experiment (see \ref
   * Experiment::run instead) but only if you are want to perform a run outside
   * of an \ref Experiment.
   *
   * Either call \ref start -> \ref update periodically after manually advancing
   * the simulation -> \ref stop, or \ref run once, which does perform all the
   * steps automatically.
   *
   */
  void start();
  /**
   * @brief      Signals that data needs to be updated
   *
   * Note that this won't execute the simulation.
   * Use it only between \ref start and \ref stop
   * calls to collect data about the current world state.
   *
   * Note that after the maximal number of steps has been recorded,
   * it won't record any more data.
   */
  void update();

  /**
   * @brief      Signals that has simulation has stopped.
   *
   * Call this after \ref start to stop the recording of a simulation.
   *
   * You don't need to call \ref start, \ref update, \ref stop, or \ref run
   * when running an \ref Experiment (see \ref
   * Experiment::run instead) but only if you are want
   * to perform a run outside of an \ref Experiment.
   *
   */
  void stop();

  /**
   * @brief      Runs a simulation and automatically record data step by
   * step
   *
   * You don't need to call \ref start, \ref update, \ref stop beside \ref run
   * as \ref run manages all the simulation and recording.
   *
   * You don't need to call \ref run when running an \ref Experiment (see \ref
   * Experiment::run instead) but only if you are want to perform a run outside
   * of an \ref Experiment.
   *
   */
  void run();

 private:
  State _state;
  /**
   * Which data to record
   */
  RecordConfig _record_config;

  /**
   * How to perform runs;
   */
  RunConfig _run_config;

  unsigned _seed;

  std::shared_ptr<World> _world;

  /**
   * The number of steps executed
   */
  unsigned _steps;
  /**
   * The number of agents recorded
   */
  unsigned _number;

  bool _record;
  std::vector<ng_float_t> _time_data;
  std::vector<ng_float_t> _pose_data;
  std::vector<ng_float_t> _twist_data;
  std::vector<ng_float_t> _cmd_data;
  std::vector<ng_float_t> _target_data;
  std::vector<ng_float_t> _safety_violation_data;
  std::vector<unsigned> _collisions_data;
  std::vector<std::vector<ng_float_t>>
      _task_events_data;  // one vector per agent
  std::vector<unsigned> _task_events;
  std::vector<ng_float_t> _deadlock_data;
  std::vector<ng_float_t> _efficacy_data;

  std::map<const Agent*, unsigned> _indices;
  std::chrono::time_point<std::chrono::steady_clock> _begin;
  std::chrono::time_point<std::chrono::steady_clock> _end;

  std::string _world_yaml;

  /**
   * @brief      Called before the simulation starts
   *
   */
  void prepare();

  /**
   * @brief      Called after the simulation has finished
   *
   */
  void finalize();

  /**
   * @brief      Save the run data to a hdf5 group
   *
   * @param      group  The dataset group where to store data
   */
  void save(HighFive::Group& group);
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_EXPERIMENTAL_RUN_H_ */
