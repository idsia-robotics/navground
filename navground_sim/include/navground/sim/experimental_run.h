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
#include "navground/sim/dataset.h"
#include "navground/sim/export.h"
#include "navground/sim/probe.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * The type of the data stored in a multi-dimensional buffer.
 */

struct Experiment;

/**
 * @brief  Holds the configuration to record sensing during an experimental run
 */
struct RecordSensingConfig {
  /**
   * The name of the dataset group
   */
  std::string name;
  /**
   * The sensor to record
   */
  std::shared_ptr<Sensor> sensor;
  /**
   * The indices of the agents to record
   */
  std::vector<unsigned> agent_indices;
};

/**
 * @brief  Holds the configuration to record neighbors during an experimental
 * run
 */
struct RecordNeighborsConfig {
  /**
   * Whether to record neighbors or not
   */
  bool enabled;
  /**
   * The number of neighbors to record. If more, it will record
   * the nearest `number` record. If fewer, it will pads with zeros.
   */
  int number;
  /**
   * Whether to use a frame relative to the agent or the absolute frame.
   */
  bool relative;
};

/**
 * @brief  Holds which data to record during an experimental run
 */
struct RecordConfig {
  /**
   * Whether to record the simulation time
   */
  bool time = false;
  /**
   * Whether to record the agents poses
   */
  bool pose = false;
  /**
   * Whether to record the agents twists
   */
  bool twist = false;
  /**
   * Whether to record the agents control commands
   */
  bool cmd;
  /**
   * Whether to record the agents actuated control commands
   */
  bool actuated_cmd;
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
   * Whether to record the initial world state (as YAML)
   */
  bool world;

  /**
   * Whether and how to record neighbors
   */
  RecordNeighborsConfig neighbors;

  /**
   * Whether to use uid as keys for sensing and task events.
   * Else it will use the agent index.
   */
  bool use_agent_uid_as_key;

  /**
   * The sensors to record
   */
  std::vector<RecordSensingConfig> sensing;

  /**
   * @brief      Sets all to record or not record.
   *
   * @return     The configuration.
   */
  static RecordConfig all(bool value) {
    return {value,
            value,
            value,
            value,
            value,
            value,
            value,
            value,
            value,
            value,
            value,
            value,
            {value, -1, false},
            true,
            {}};
  }

  /**
   * @brief      Sets all to record or not record.
   *
   * @param[in]  value  The desired value
   */
  void set_all(bool value) { *this = all(value); }

  /**
   * @brief      Add a sensor to be recorded
   *
   * @param[in]  name           The name of the group
   * @param[in]  sensor         The sensor to record
   * @param[in]  agent_indices  The agent indices whose sensing to record
   */
  void record_sensor(const std::string &name,
                     const std::shared_ptr<Sensor> &sensor,
                     const std::vector<unsigned> &agent_indices) {
    sensing.push_back({name, sensor, agent_indices});
  }
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

template <typename T>
inline std::set<std::string> _keys(const std::map<std::string, T> &m) {
  std::set<std::string> keys;
  for (auto const &[k, _] : m) {
    keys.insert(k);
  }
  return keys;
}

/**
 * @brief      Extracts collisions events, i.e.
 *             collisions separated by more
 *             than min_interval steps
 *
 * @param[in] collisions    A dataset of collisions, i.e. of tuples
 *                          <time_step, uid_1, uid_2> of type unsigned.
 * @param[in] min_interval  The minimal interval between collision among
 *                          the same pair to be considered a new event
 *
 * @return     A dataset with the same type and item shape.
 */
std::shared_ptr<Dataset> NAVGROUND_SIM_EXPORT extract_collision_events(
    const std::shared_ptr<Dataset> &collisions, unsigned min_interval = 1);

std::shared_ptr<Dataset> NAVGROUND_SIM_EXPORT extract_steps_to_collision(
    unsigned first_agent_id, unsigned last_agent_id, unsigned steps,
    const std::shared_ptr<Dataset> &collisions, unsigned min_interval = 1);

/**
 * @brief      Simulates a world and collects data.
 */
class NAVGROUND_SIM_EXPORT ExperimentalRun {
  friend struct Experiment;

public:
  /**
   * @brief      The state of the run
   *
   * @private
   */
  enum class State { init, running, finished };

  using tp = std::chrono::time_point<std::chrono::steady_clock>;

  /**
   * @private
   */
  ExperimentalRun(
      std::shared_ptr<World> world, const RunConfig &run_config,
      const RecordConfig &record_config, unsigned seed, State state,
      unsigned steps, tp begin, tp end, const std::string &world_yaml,
      const std::map<std::string, std::shared_ptr<Dataset>> &records)
      : _state(state), _record_config(record_config), _run_config(run_config),
        _seed(seed), _world(world), _steps(steps),
        // _indices(),
        _begin(begin), _end(end), _world_yaml(world_yaml), _records(records),
        _record_names(_keys(records)), _probes() {}

  /**
   * @brief      Construct an \ref ExperimentalRun
   *
   * @param[in]  world          The world to simulate
   * @param[in]  run_config     The run configuration
   * @param[in]  record_config  The record configuration
   * @param[in]  seed           The seed used to initialize the world
   */
  ExperimentalRun(std::shared_ptr<World> world, const RunConfig &run_config,
                  const RecordConfig &record_config, unsigned seed = 0)
      : _state(State::init), _record_config(record_config),
        _run_config(run_config), _seed(seed), _world(world), _steps(0),
        // _indices(),
        _world_yaml(), _records(), _record_names(), _probes() {}

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
                  const RecordConfig &record_config, unsigned seed = 0)
      : ExperimentalRun(
            world, {time_step, max_steps, terminate_when_all_idle_or_stuck},
            record_config, seed) {}

  // ExperimentalRun(const ExperimentalRun&) = delete;

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
   * @brief      Gets when the simulation started.
   *
   * @return     The begin stamp
   */
  tp get_begin() const { return _begin; }
  /**
   * @brief     Gets when the simulation finished.
   *
   * @return     The end stamp
   */
  tp get_end() const { return _end; }

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
   * @brief      Whether the run is running.
   *
   * @return     True if running, False otherwise.
   */
  bool is_running() const { return _state == State::running; }

  /**
   * @brief      Whether the run is running or has already finished.
   *
   * @return     True if started, False otherwise.
   */
  bool has_started() const { return _state != State::init; }

  /**
   * @brief      Gets the simulation state.
   *
   * @return     The state.
   */
  State get_state() const { return _state; }

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
  unsigned get_maximal_steps() const { return _run_config.steps; }
  /**
   * @brief      Gets whether to terminate when all agents are idle or stuck.
   *
   * @return     Whether to terminate when all agents are idle or stuck.
   */
  bool get_terminate_when_all_idle_or_stuck() const {
    return _run_config.terminate_when_all_idle_or_stuck;
  }

  // RunConfig get_run_config() const { return _run_config; }

  /**
   * @brief      Gets the record associated to a given key.
   *
   * @param[in]  key  The key
   *
   * @return     The record or null if none is found.
   */
  const std::shared_ptr<Dataset> get_record(const std::string &key) const {
    if (_records.count(key)) {
      return _records.at(key);
    }
    return nullptr;
  }

  /**
   * @brief      Gets the records.
   *
   * @param[in]  group   If specified, limits to records in a given group.
   *
   * @return     The records
   */
  const std::map<std::string, std::shared_ptr<Dataset>>
  get_records(const std::string &group = "") const {
    if (group.empty()) {
      return _records;
    }
    std::map<std::string, std::shared_ptr<Dataset>> records;
    for (auto &[key, full] : get_group(group)) {
      records[key] = get_record(full);
    }
    return records;
  }

  /**
   * @brief      Gets the recorded times.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_times() const {
    return get_record("times");
  }
  /**
   * @brief      Gets the recorded poses.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_poses() const {
    return get_record("poses");
  }
  /**
   * @brief      Gets the recorded twists.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_twists() const {
    return get_record("twists");
  }
  /**
   * @brief      Gets the recorded commands.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_cmds() const { return get_record("cmds"); }
  /**
   * @brief      Gets the recorded actuated commands.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_actuated_cmds() const {
    return get_record("actuated_cmds");
  }
  /**
   * @brief      Gets the recorded targets.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_targets() const {
    return get_record("targets");
  }
  /**
   * @brief      Gets the recorded safety margin violations.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_safety_violations() const {
    return get_record("safety_violations");
  }
  /**
   * @brief      Gets the recorded collisions.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_collisions() const {
    return get_record("collisions");
  }
  /**
   * @brief      Gets the recorded collisions events, i.e.
   *             collisions separated by more than min_interval steps
   *
   * @param[in] min_interval  The minimal interval between collision among
   *                          the same pair to be considered a new event
   *
   * @return     The record or none if not recorded.
   */
  std::shared_ptr<Dataset>
  get_collision_events(unsigned min_interval = 1) const {
    return extract_collision_events(get_collisions(), min_interval);
  }

  /**
   * @brief      Gets the steps to the next recorded collision 
   *             for each agent at each simulation step.
   *
   * @param[in]  min_interval  The minimal interval between collision among
   *                           the same pair to be considered a new event
   *
   * @return     A dataset of shape `{#steps, #agents}`.
   */
  std::shared_ptr<Dataset>
  get_steps_to_collision(unsigned min_interval = 1) const {
    const auto agents = get_world()->get_agents();
    const bool use_uid = get_record_config().use_agent_uid_as_key;
    unsigned i = 0;
    unsigned j = 0;
    if (agents.size()) {
      if (use_uid) {
        i = agents[0]->uid;
        j = agents.back()->uid;
      } else {
        j = agents.size() - 1;
      }
    }
    return extract_steps_to_collision(i, j, get_recorded_steps(),
                                      get_collisions(), min_interval);
  }

  /**
   * @brief      Gets the recorded task logs.
   *
   * @return     The records (empty if not recorded).
   */
  const std::map<unsigned, std::shared_ptr<Dataset>> get_task_events() const {
    std::map<unsigned, std::shared_ptr<Dataset>> records;
    for (auto &[key, full] : get_group("task_events")) {
      records[static_cast<unsigned>(std::stoul(key))] = get_record(full);
    }
    return records;
  }
  /**
   * @brief      Gets the recorded task logs.
   *
   * @param[in]  uid   The agent uid or index
   *                   (if \ref RecordConfig::use_agent_uid_as_key is not set)
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_task_events_for(unsigned uid) const {
    const std::string key = std::to_string(uid);
    auto g = get_group("task_events");
    if (g.count(key)) {
      return get_record(g.at(key));
    }
    return nullptr;
  }
  /**
   * @brief      Gets the recorded deadlocks.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_deadlocks() const {
    return get_record("deadlocks");
  }
  /**
   * @brief      Gets the recorded efficacy.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_efficacy() const {
    return get_record("efficacy");
  }
  /**
   * @brief      Gets the recorded neighbors.
   *
   * @return     The record or none if not recorded.
   */
  const std::shared_ptr<Dataset> get_neighbors() const {
    return get_record("neighbors");
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
  // unsigned get_number_of_agents() const { return _number; }
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

  // TODO(Jerome): Should remove it ... almost useless and adds state
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
  std::optional<unsigned> index_of_agent(const Agent *agent) const {
    if (_world) {
      return _world->index_of_agent(agent);
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

  /**
   * @brief      Determines if a record is present.
   *
   * @param[in]  key    The key
   * @param[in]  group  An optional group. If specified, the record with be
   *                    associated to the key ``<group>/<key>``.
   *
   * @return     True if a record is associated with the key, False otherwise.
   */
  bool has_record(std::string key, const std::string &group = "") const {
    if (!group.empty()) {
      key = group + "/" + key;
    }
    return _records.count(key) != 0;
  }

  /**
   * @brief      Adds a record.
   *
   * @param[in]  key    The key
   * @param[in]  group  An optional group. If specified, the record with be
   *                    associated to the key ``<group>/<key>``.
   * @param[in]  force  If specified, it will replace the record associated
   *                    with the key, if already present.
   *
   * @return     The created record
   */
  std::shared_ptr<Dataset> add_record(std::string key,
                                      const std::string &group = "",
                                      bool force = false) {
    if (!group.empty()) {
      key = group + "/" + key;
    }
    if (_records.count(key) && !force) {
      return _records[key];
    }
    _record_names.insert(key);
    _records[key] = std::make_shared<Dataset>();
    return _records[key];
  }

  /**
   * @brief      Add a record
   *
   * @param[in]  ds     The dataset
   * @param[in]  key    The key associated to the record
   * @param[in]  group  The group associated to the record.
   *                    If provided, the effective key will be ``<group>/<key>``
   * @param[in]  force  Whether to replace a record if already existing
   *                    for this key.
   */
  void insert_record(std::shared_ptr<Dataset> ds, std::string key,
                     const std::string &group = "", bool force = false) {
    if (!group.empty()) {
      key = group + "/" + key;
    }
    if (_records.count(key) && !force) {
      return;
    }
    _record_names.insert(key);
    _records[key] = ds;
  }

  /**
   * @brief      Adds a probe
   *
   * @param[in]  probe   The probe
   */
  void add_probe(const std::shared_ptr<Probe> &probe) {
    _probes.push_back(probe);
  }

  /**
   * @brief      Adds a record probe.
   *
   * @param[in]  key   The key of the record to be created
   *
   * @tparam     T     The probe class.
   */
  template <typename T>
  void add_record_probe(const std::string &key, bool force = false) {
    auto ds = add_record(key);
    ds->set_dtype<typename T::Type>();
    add_probe(std::dynamic_pointer_cast<Probe>(std::make_shared<T>(ds)));
  }

  /**
   * @brief      Adds a group record probe.
   *
   * @param[in]  key   The key of the group to be created
   *
   * @tparam     T     The probe class.
   */
  template <typename T> void add_group_record_probe(const std::string &key) {
    add_probe(std::dynamic_pointer_cast<Probe>(
        std::make_shared<T>([key, this](const std::string &sub_key) {
          auto ds = add_record(sub_key, key);
          ds->set_dtype<typename T::Type>();
          return ds;
        })));
  }

  /**
   * @brief      Gets the names of records.
   *
   * @param[in]  group  An optional group. If specified, the looks for record
   *                    associated to keys ``<group>/...``.
   *
   * @return     The record names (relative to the group if specified).
   */
  std::set<std::string> get_record_names(const std::string &group = "") const;

  /**
   * @brief      Gets the record keys in a group.
   *
   * @param[in]  name  The group name
   *
   * @return     A map of record keys ``{<key>: name/<key>}``
   *             indexed by keys relative to a group
   */
  std::map<std::string, std::string> get_group(const std::string &name) const;

  /**
   * @brief      Gets the YAML representation of the world at the begin of the
   * simulation..
   *
   * @return     The YAML string.
   */
  std::string get_world_yaml() const { return _world_yaml; }

  /**
   * @brief      Try to advance the world to a given recorded step.
   *
   * Depending if the data has been recorded, it will update:
   *
   * - poses
   * - twists
   * - cmds
   * - time
   * - collisions
   *
   * @param[in]  step  The step. Negative steps are interpreted as relative to
   * the last registered step, i.e., -1 is the last step.
   *
   * @param[in]  ignore_twists Whether to skip setting twists
   * @param[in]  ignore_cmds Whether to skip setting [last] commands
   * @param[in]  ignore_collisions Whether to skip setting collisions
   *
   * @return     True if the operation was possible and false otherwise.
   */
  bool go_to_step(int step, bool ignore_collisions = false,
                  bool ignore_twists = false, bool ignore_cmds = false);

  /**
   * @brief      Resets the run.
   *
   */
  void reset();

  /**
   * @brief      Gets recorded collisions at a given step.
   *
   * @param[in]  step  The step. Negative steps are interpreted as relative to
   * the last registered step, i.e., -1 is the last step.
   *
   * @return     The set of colliding entity pairs.
   */
  std::set<std::tuple<Entity *, Entity *>> get_collisions_at_step(int step);

  /**
   * @brief      Gets the last recorded simulation time
   *
   * @return     The simulation time
   */
  ng_float_t get_final_sim_time() const;

  /**
   * @brief      Gets the rectangle that contains the world during the whole
   * run.
   *
   * @return     The bounding box.
   */
  BoundingBox get_bounding_box() const;

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
  // unsigned _number;

  // std::map<const Agent *, unsigned> _indices;
  tp _begin;
  tp _end;
  std::string _world_yaml;
  std::map<std::string, std::shared_ptr<Dataset>> _records;
  std::set<std::string> _record_names;
  std::vector<std::shared_ptr<Probe>> _probes;

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
  void save(HighFive::Group &group) const;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_EXPERIMENTAL_RUN_H_ */
