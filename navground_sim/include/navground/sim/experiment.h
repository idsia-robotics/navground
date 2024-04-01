/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *  (Run)
 */

#ifndef NAVGROUND_SIM_EXPERIMENT_H_
#define NAVGROUND_SIM_EXPERIMENT_H_

#include <chrono>
#include <filesystem>
#include <highfive/H5File.hpp>
#include <memory>

#include "navground/core/types.h"
#include "navground/sim/experimental_run.h"
#include "navground/sim/scenario.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/scenario.h"
#include "navground_sim_export.h"

namespace navground::sim {

/**
 * @brief  An experiment supervises the execution of several runs of simulation.
 *
 * It initializes simulations sharing the same \ref Scenario and
 * run them while collecting the desired data through \ref ExperimentalRun.
 *
 * - use \ref run_once to perform a single run.
 *
 * - use \ref run to perform all runs, optionally saving the data to a HDF5
 * dataset.
 *
 * - use \ref start, \ref stop, \ref start_run, \ref stop_run, \ref update_run
 * to record data without launching a simulation, for instance if you are using
 * a different run-loop.
 *
 */
struct NAVGROUND_SIM_EXPORT Experiment {
  /**
   * @brief      The state
   *
   * @private
   */
  enum class State { init, running, finished };

  virtual ~Experiment() = default;

  // using Callback = std::function<void()>;

  /**
   * The type of callbacks called during an experiment
   */
  using RunCallback = std::function<void(ExperimentalRun*)>;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  time_step  The default simulation time step
   * @param[in]  steps      The default number of simulation steps
   */
  explicit Experiment(ng_float_t time_step = 0.1, unsigned steps = 1000)
      : record_config(),
        run_config({time_step, steps, true}),
        number_of_runs(1),
        save_directory(),
        runs(),
        name("experiment"),
        scenario(nullptr),
        run_index(0),
        state(State::init),
        // callbacks(),
        run_callbacks(),
        file(),
        file_path() {}

  /**
   * @brief      Gets the map of recorded runs.
   *
   * Runs are associated to the seed used to initialize them.
   *
   * @return     The recorded runs.
   */
  const std::map<unsigned, ExperimentalRun>& get_runs() const { return runs; }

  void add_run(unsigned seed, ExperimentalRun & run) {
    if (runs.count(seed) == 0) {
      runs.emplace(seed, run);
    }
  }

  /**
   * @brief      Determines if the experiment has finished.
   *
   * @return     True if finished, False otherwise.
   */
  bool has_finished() const { return state == State::finished; }

  /**
   * @brief      Determines if the experiment is running.
   *
   * @return     True if running, False otherwise.
   */
  bool is_running() const { return state == State::running; }

  /**
   * @brief      Perform a single run
   *
   * 1. it initializes a world from its scenario by calling
   *    \ref Scenario::init_world
   *
   * 2. it runs the simulation, step by step, collecting data in a \ref
   * ExperimentalRun by calling \ref ExperimentalRun::run
   *
   * @param[in]  seed  The index (and random seed) of the run
   *
   * @return     The recorded run.
   */
  ExperimentalRun& run_once(unsigned seed);

  /**
   * @brief      Perform several runs and optionally record the data in a HFD5
   * file.
   *
   * The number of runs is specified by the default \ref number_of_runs if not
   * specified.
   *
   * Runs will be indexed sequentially and their index used as a random seed.
   *
   * If \ref save_directory not empty but points to an existing directory,
   * it creates a HDF5 file ``<name>_<hash>_<timestamp>/data.h5`` with
   * attributes
   *
   * - ``begin_time`` [``string``], ISO 8601 formatted string of the time when
   *   the experiment is run, see \ref get_begin_time;
   *
   * - ``duration_ns`` [``unsigned``], total duration in nanoseconds, see
   *   \ref get_duration_ns.
   *
   * - ``experiment`` [``string``], YAML serialization of the experiment;
   *
   * Moreover, at the end of each run, it saves a group ``run_<index>`` with
   * attributes:
   *
   * - ``duration_ns`` [``unsigned``], total duration in nanoseconds, see
   *   \ref get_run_duration_ns;
   *
   * - ``seed`` [``unsigned``];
   *
   * - ``steps`` [``unsigned``], actual number of steps performed;
   *
   * - ``maximal_steps`` [``unsigned``], maximal number of steps that could have
   * been performed;
   *
   * - ``final_sim_time`` [``float``], the simulated time at the end of the run;
   *
   * - ``world`` [``string``], YAML serialization of the world at the begin of
   *   the experiment.
   *
   * datasets:
   *
   * - ``times`` [``float``] (if \ref RecordConfig::time is set);
   *
   * - ``poses`` [``float``] (if \ref RecordConfig::pose is set);
   *
   * - ``twists`` [``float``] (if \ref RecordConfig::twist is set);
   *
   * - ``cmds`` [``float``] (if \ref RecordConfig::cmd is set);
   *
   * - ``targets`` [``float``] (if \ref RecordConfig::target is set);
   *
   * - ``collisions`` [``unsigned``] (if \ref RecordConfig::collisions
   * is set);
   *
   * - ``deadlocks`` [``float``] (if \ref RecordConfig::deadlocks is
   * set);
   *
   * - ``efficacy`` [``float``] (if \ref RecordConfig::efficacy is
   * set);
   *
   * and groups:
   *
   * - ``task_events`` (if \ref RecordConfig::task_events is set),
   *   where each agents logs, in dataset ``<uid>`` [``float``], the
   *   events emitted by their task.
   *
   * Apart from saving data to the HDF5 file, each run is performed similarly
   * to \ref run_once.
   *
   * @param[in]  keep  Whether to keep runs in memory
   * @param[in]  number_of_threads  How many threads to use.
   * When more than one, runs will be distributed in parallel over the threads.
   * @param[in]  start_index  The index/seed of the first run.
   * If unspecified, it will use the experiment's \ref run_index
   * @param[in]  number_of_runs  The number of runs.
   * If unspecified, it will use the experiment's \ref number_of_runs
   * @param[in]  data_path  A path to set optionally as \ref file_path.
   * If set, it will save an HDF5 file, but no YAML, to this path.
   * If not set, \ref file_path will be automatically set to
   * ``<save_directory>/data.h5`` if \ref save_directory is set.
   */
  void run(bool keep = true, unsigned number_of_threads = 1,
           std::optional<unsigned> start_index = std::nullopt,
           std::optional<unsigned> number_of_runs = std::nullopt,
           std::optional<std::filesystem::path> data_path = std::nullopt);

  // void run_in_parallel(unsigned number_of_threads, bool keep = true,
  //                      std::optional<unsigned> start_index = std::nullopt,
  //                      std::optional<unsigned> number = std::nullopt);

  /**
   * @brief      Clear the recording
   */
  virtual void remove_all_runs() { runs.clear(); }

  /**
   * @brief      Removes a recorded run.
   *
   * @param[in]  seed  The seed/index of the run
   */
  virtual void remove_run(unsigned seed) { runs.erase(seed); }

  /**
   * @brief      Signal to start an experiment
   *
   * Note that this won't neither execute any simulation nor record data.
   * It will just record time stamp and change the state to running.
   *
   * This is only needed when using an external run-loop for the simulation.
   * In this case:
   *
   * 1. call \ref start
   *
   * 2. For each run
   *
   *    i. Call \ref init_run and \ref start_run
   *    ii. from your run-loop, call \ref update_run to record
   *        the current state of the simulated world
   *    iii. Call \ref stop_run
   *
   * 3. Call \ref stop
   *
   * Use \ref run instead to perform the simulations and record them all at
   * once.
   *
   * Calling \ref start is only effective once per experiment.
   *
   * @param[in]  path  A path to set optionally as \ref file_path.
   * If set, it will save an HDF5 file, but no YAML, to this path.
   * If not set, \ref file_path will be automatically set to
   * ``<save_directory>/data.h5`` if \ref save_directory is set.
   *
   */
  void start(std::optional<std::filesystem::path> path = std::nullopt);
  /**
   * @brief      Signal to stop an experiment
   * 
   * @param[in]  save_runs   Whether to save the runs before closing
   *
   * See \ref start. This is only need when using an external run-loop to
   * simulate.
   */
  void stop(bool save_runs = false);

  /**
   * @brief      Initializes a run
   *
   * @param[in]  seed   The random seed
   * @param[in]  world  The world to simulate. If null, it will initialize a new
   * world.
   *
   * See \ref start. This is only need when using an external run-loop to
   * simulate or when manually calling \ref ExperimentalRun::run later. Else use
   * \ref run_once (or \ref run for all runs) to initialize and run at once.
   *
   */
  virtual ExperimentalRun& init_run(int seed,
                                    std::shared_ptr<World> world = nullptr);

  /**
   * @brief      Start recording a run
   *
   * See \ref start. This is only need when using an external run-loop to
   * simulate.
   *
   * Call \ref start_run only once per run.
   *
   * @param      run   The run being recorded
   *
   */
  void start_run(ExperimentalRun& run);

  /**
   * @brief      { function_description }
   *
   * See \ref start. This is only need when using an external run-loop to
   * simulate.
   *
   * Call \ref update_run every time you need to record the current state of the
   * world, typically after each simulation step.
   *
   * @param      run   The run being recorded
   */
  void update_run(ExperimentalRun& run);

  /**
   * @brief      Stop the run recording
   *
   * See \ref start_run. This is only need to use an external run-loop.
   *
   * @param      run   The run being recorded
   */
  void stop_run(ExperimentalRun& run);

  /**
   * @brief      Adds a callback to be executed after each simulation step.
   *
   * @param[in]  value  The callback
   */
  // void add_callback(const Callback& value) { callbacks.push_back(value); }

  /**
   * @brief      Adds a callback to be executed after each run.
   *
   * @param[in]  value  The callback
   *
   * @param[in]  value  Whether the callback should be called when initializing
   * the run. If not set, it will be called at the completion of a run.
   */
  void add_run_callback(const RunCallback& value, bool at_init = false) {
    run_callbacks[at_init].push_back(value);
  }

  /**
   * @brief      Remove all run callbacks
   *
   * @param[in]  value  The callback
   */
  void clear_run_callbacks() { run_callbacks.clear(); }

  /**
   * @brief      Gets the path where the experimental data has been saved.
   *
   * @return     The path or none if the experiment has not been run yet.
   */
  std::optional<std::filesystem::path> get_path() const { return file_path; }

  /**
   * @brief      Gets the duration required to perform the whole experiment.
   *
   * @return     The duration in ns or 0 if the experiment has not finished.
   */
  std::chrono::nanoseconds get_duration_ns() const {
    if (state == State::finished) {
      return experiment_end - experiment_begin;
    }
    return std::chrono::nanoseconds(0);
  }

  /**
   * @brief      Gets the system time when the experiment began.
   *
   * @return     The time
   */
  std::chrono::time_point<std::chrono::system_clock> get_begin_time() const {
    return begin;
  }

  /**
   * @brief      Determines if it is recording data to a file or it has done it.
   *
   * @return     True if file, False otherwise.
   */
  bool has_file() const { return bool(file_path); }

  /**
   * @brief      Returns the YAML representation of the experiment.
   *
   * @private
   */
  virtual std::string dump() const;
  // virtual std::string dump() const { return YAML::dump<Experiment>(this); }

  /**
   * @brief      Gets the default time step used for simulation during each run
   *
   * @return     The time step.
   */
  ng_float_t get_time_step() const { return run_config.time_step; }
  /**
   * @brief      Sets the default time step used for simulation during each run
   *
   * @param[in]  value  The desired value
   */
  void set_time_step(ng_float_t value) { run_config.time_step = value; }
  /**
   * @brief      Gets the default maximal number of steps to simulate during
   * each run.
   *
   * @return     The maximal number of steps.
   */
  unsigned get_steps() const { return run_config.steps; }
  /**
   * @brief      Sets the default maximal number of steps to simulate during
   * each run.
   *
   * @param[in]  value  The desired value
   */
  void set_steps(ng_float_t value) { run_config.steps = value; }
  /**
   * @brief      Gets whether to terminate when all agents are idle or stuck.
   *
   * @return     Whether to terminate when all agents are idle or stuck.
   */
  bool get_terminate_when_all_idle_or_stuck() const {
    return run_config.terminate_when_all_idle_or_stuck;
  }
  /**
   * @brief      Sets whether to terminate when all agents are idle or stuck.
   *
   * @param[in]  value  The desired value
   */
  void set_terminate_when_all_idle_or_stuck(bool value) {
    run_config.terminate_when_all_idle_or_stuck = value;
  }

  /**
   * @brief      Register a probe to be added to all runs
   *
   * @param[in]  factory  A function that generate the probe.
   */
  void add_probe(const std::function<std::shared_ptr<Probe>()>& factory) {
    add_run_callback(
        [factory](ExperimentalRun* run) { run->add_probe(factory()); }, true);
  }

  /**
   * @brief      Register a probe to record data to during all runs.
   *
   * @param[in]  key   The name associated to the record
   *
   * @tparam     T     The class of the probe.
   *                   Must be subclass of \ref sim::RecordProbe
   */
  template <typename T>
  void add_record_probe(const std::string& key) {
    add_run_callback(
        [key](ExperimentalRun* run) { run->add_record_probe<T>(key); }, true);
  }

  /**
   * @brief      Register a probe to record a group of data to during all runs.
   *
   * @param[in]  key   The name associated to the group
   *
   * @tparam     T     The class of the probe. Must be subclass of \ref
   * sim::GroupRecordProbe
   */
  template <typename T>
  void add_group_record_probe(const std::string& key) {
    add_run_callback(
        [key](ExperimentalRun* run) { run->add_group_record_probe<T>(key); },
        true);
  }

  /**
   * Which data to record
   */
  RecordConfig record_config;

  /**
   * How to perform runs;
   */
  RunConfig run_config;

  /**
   * Default number of runs to perform
   */
  unsigned number_of_runs;

  /**
   * Where to save the results
   */
  std::filesystem::path save_directory;

  /**
   * All runs, ordered by seed
   */
  std::map<unsigned, ExperimentalRun> runs;

  /**
   * The name of the experiment
   */
  std::string name;

  /**
   * The scenario used to initialize a world
   */
  std::shared_ptr<Scenario> scenario;
  /**
   * The seed/index of the next run
   */
  unsigned run_index;

  /**
   * @brief      Save all recorded runs.
   *
   * @param[in]  directory  A path to set optionally as \ref save_directory
   * where to save YAML and HDF5 file.
   *
   * @param[in]  path  A path to set optionally as \ref file_path.
   * If set, it will save an HDF5 file, but no YAML, to this path.
   * If not set, \ref file_path will be automatically set to
   * ``<save_directory>/data.h5`` if \ref save_directory is set.
   */
  void save(std::optional<std::filesystem::path> directory = std::nullopt,
            std::optional<std::filesystem::path> path = std::nullopt);

 protected:
  State state;

  virtual std::shared_ptr<World> make_world() {
    return std::make_shared<World>();
  }

  void init_dataset(std::optional<std::filesystem::path> path = std::nullopt);
  void finalize_dataset();
  std::unique_ptr<HighFive::Group> init_dataset_run(unsigned index);
  void store_yaml(const std::string& yaml) const;
  void save_run(const ExperimentalRun& run);
  ExperimentalRun& _run_once(unsigned index);

  // std::vector<Callback> callbacks;
  std::map<bool, std::vector<RunCallback>> run_callbacks;

  std::shared_ptr<HighFive::File> file;

  std::chrono::time_point<std::chrono::steady_clock> experiment_begin;
  std::chrono::time_point<std::chrono::steady_clock> experiment_end;
  std::chrono::time_point<std::chrono::system_clock> begin;
  /**
   * Where results are saved
   */
  std::optional<std::filesystem::path> file_path;

  void run_in_sequence(
      bool keep = true, std::optional<unsigned> start_index = std::nullopt,
      std::optional<unsigned> number = std::nullopt,
      std::optional<std::filesystem::path> data_path = std::nullopt);

  virtual void run_in_parallel(
      unsigned number_of_threads, bool keep = true,
      std::optional<unsigned> start_index = std::nullopt,
      std::optional<unsigned> number = std::nullopt,
      std::optional<std::filesystem::path> data_path = std::nullopt);
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_EXPERIMENT_H_ */
