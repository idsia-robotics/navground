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

#include "navground/sim/scenario.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

namespace navground::sim {

struct Experiment;

/**
 * @brief      This class helps to collect the trajectories of the agents
 * during an experiment.
 */
struct NAVGROUND_SIM_EXPORT Trace {
  friend struct Experiment;

  /**
   * Whether it should record the agents poses
   */
  bool record_pose;
  /**
   * Whether it should record the agents twists
   */
  bool record_twist;
  /**
   * Whether it should record the agents control commands
   */
  bool record_cmd;
  /**
   * Whether it should record the agents targets
   */
  bool record_target;
  /**
   * Whether it should record collisions
   */
  bool record_collisions;
  /**
   * Whether it should record safety violations
   */
  bool record_safety_violation;
  /**
   * Whether it should record data from task events
   */
  bool record_task_events;
  /**
   * @brief      Constructs a new instance.
   */
  Trace()
      : record_pose(false),
        record_twist(false),
        record_cmd(false),
        record_target(false),
        record_collisions(false),
        record_safety_violation(false),
        record_task_events(false),
        pose_data(),
        twist_data(),
        cmd_data(),
        target_data(),
        safety_violation_data(),
        collisions_data(),
        task_events_data(),
        task_events(),
        record(false),
        indices() {}

  /**
   * The number of steps recorded
   */
  unsigned steps;
  /**
   * The number of agents recorded
   */
  unsigned number;
  std::vector<float> pose_data;
  std::vector<float> twist_data;
  std::vector<float> cmd_data;
  std::vector<float> target_data;
  std::vector<float> safety_violation_data;
  std::vector<unsigned> collisions_data;
  std::vector<std::vector<float>> task_events_data;  // one vector per agent
  std::vector<unsigned> task_events;

  std::optional<unsigned> index_of_agent(const Agent* agent) const {
    if (indices.count(agent)) {
      return indices.at(agent);
    }
    return std::nullopt;
  }

 private:
  bool record;
  float* pose_ptr;
  float* twist_ptr;
  float* cmd_ptr;
  float* target_ptr;
  float* safety_violation_ptr;

  std::map<const Agent*, unsigned> indices;

  /**
   * @brief      Init a trace recording
   *
   * @param[in]  world  The world
   * @param[in]  steps  The maximal number of steps that will be recorded
   */
  void init(const World& world, unsigned steps);
  /**
   * @brief      { function_description }
   *
   * @param[in]  world  The world
   * @param[in]  step   The simulation step index
   */
  void update(const World& world, unsigned step);

  /**
   * @brief      Called after the simulation has finished
   *
   * @param[in]  world  The world
   */
  void finalize(const World& world);

  /**
   * @brief      Save the trace data to a hdf5 group
   *
   * @param[in]  world  The world
   * @param      group  The dataset group where to store data
   */
  void save(const World& world, HighFive::Group& group);
};

/**
 * @brief      An experiment supervises the execution of a simulation.
 *
 * It initializes a simulation and run it while collecting the desired data with
 * \ref Trace.
 *
 * Use \ref run_once to perform a single run.
 *
 * Use \ref run to perform all runs, optionally saving the data to a HDF5
 * dataset.
 *
 */
struct NAVGROUND_SIM_EXPORT Experiment {
  virtual ~Experiment() = default;

  /**
   * Callback
   */
  using Callback = std::function<void()>;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  time_step  The simulation time step
   * @param[in]  steps      The number of simulation steps
   */
  explicit Experiment(float time_step = 0.1, int steps = 1000)
      : runs(1),
        time_step(time_step),
        steps(steps),
        save_directory(),
        trace(),
        name("experiment"),
        scenario(nullptr),
        terminate_when_all_idle(true),
        run_index(0),
        initialized(false),
        step(0),
        world(),
        callbacks(),
        file(),
        run_group(),
        file_path() {}

  /**
   * @brief      Perform a single run
   *
   * 1. it initializes a world from its scenario by calling
   *    \ref Scenario::init_world
   *
   * 2. it runs the simulation, step by step, collecting data in \ref trace.
   *
   * @param[in]  seed  The index (and random seed) of the run
   */
  void run_once(int seed);

  /**
   * @brief      Perform all runs
   *
   * The number of runs is specified by \ref runs.
   *
   * If \ref save_directory not empty but points to an existing directory,
   * it creates a HDF5 file ``<name>_<timestamp>/data.h5`` with attributes
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
   * - ``world`` [``string``], YAML serialization of the world at the begin of
   *   the experiment.
   *
   * datasets:
   *
   * - ``cmds`` [``float``] (if \ref Trace::record_cmd is set);
   *
   * - ``collisions`` [``unsigned``] (if \ref Trace::record_collisions is set);
   *
   * - ``poses`` [``float``] (if \ref Trace::record_pose is set);
   *
   * - ``targets`` [``float``] (if \ref Trace::record_target is set);
   *
   * - ``twists`` [``float``] (if \ref Trace::record_twist is set);
   *
   * and groups:
   *
   * - ``task_events`` (if \ref Trace::record_task_events is set),
   *   where each agents logs, in dataset ``agent_<uid>`` [``float``], the
   *   events emitted by their task.
   *
   * A part from saving data to the HDF5 file, each run is performed similarly
   * to \ref run_once.
   */
  void run();

  /**
   * @brief      Start recording
   * 
   * This is only need to use an external run-loop, where you
   * call \ref update explicitly to update the trace 
   * without updating the simulation itself. Use \ref run instead, to 
   * both run the simulation and record the trace.
   * 
   * Call \ref start only once per experiment.
   * 
   */
  void start();
  /**
   * @brief      Start recording
   * 
   * This is only need to use an external run-loop, where you
   * call \ref update explicitly to update the trace 
   * without updating the simulation itself. Call once at the start of each run.
   * 
   * @param[in]  seed  The random seed
   * @param[in]  init_world  Whether it should initialize a new world
   * 
   */
  void start_run(int seed, bool init_world = false);
  /**
   * @brief      Updates the trace.
   * 
   * See \ref start. Call from an external run-loop to update the trace 
   * without updating the simulation itself.
   */
  void update();
  /**
   * @brief      Stop the recording
   * 
   * See \ref start. This is only need to use an external run-loop.
   */
  void stop();
  /**
   * @brief      Stop the run recording
   * 
   * See \ref start_run. This is only need to use an external run-loop.
   */
  void stop_run();


  /**
   * @brief      Adds a callback to be executed after each simulation step.
   *
   * @param[in]  value  The callback
   */
  void add_callback(const Callback& value) { callbacks.push_back(value); }

  /**
   * @brief      Gets the path where the experimental data has been saved.
   *
   * @return     The path or none if the experiment has not been run yet.
   */
  std::optional<std::filesystem::path> get_path() const { return file_path; }

  std::shared_ptr<World> get_world() const { return world; }

  void set_world(const std::shared_ptr<World> & value) { 
    world = value; 
  }

  /**
   * @brief      Gets the duration required to perform the last run (excludes
   * initialization).
   *
   * @return     The duration in ns
   */
  std::chrono::nanoseconds get_run_duration_ns() const {
    return run_end - run_begin;
  }

  /**
   * @brief      Gets the duration required to perform the whole experiment.
   *
   * @return     The duration in ns
   */
  std::chrono::nanoseconds get_duration_ns() const {
    return experiment_end - experiment_begin;
  }

  /**
   * @brief      Gets the time when the experiment began.
   *
   * @return     The time
   */
  std::chrono::time_point<std::chrono::system_clock> get_begin_time() const {
    return begin;
  }

  /**
   * Number of runs to perform
   */
  unsigned runs;
  /**
   * Simulation time step
   */
  float time_step;
  /**
   * Maximal number of steps per run
   */
  unsigned steps;

  /**
   * Where to save the results
   */
  std::filesystem::path save_directory;

  /**
   * The current trace
   */
  Trace trace;

  /**
   * The name of the experiment
   */
  std::string name;

  /**
   * The scenario
   */
  std::shared_ptr<Scenario> scenario;

  bool terminate_when_all_idle;

  bool has_file() const {
    return bool(file_path);
  }

  /**
   * The index of the next run
   */
  unsigned run_index;

 protected:
  void run_run();
  void init_run(int seed);

  virtual std::shared_ptr<World> make_world() {
    return std::make_shared<World>();
  }

  virtual std::string dump();

  void init_dataset();
  void finalize_dataset();
  void init_dataset_run(unsigned index);
  void finalize_dataset_run();

  bool initialized;
  unsigned step;

  std::shared_ptr<World> world;
  
  std::vector<Callback> callbacks;
  
  std::shared_ptr<HighFive::File> file;
  std::shared_ptr<HighFive::Group> run_group;

  std::chrono::time_point<std::chrono::steady_clock> experiment_begin;
  std::chrono::time_point<std::chrono::steady_clock> experiment_end;
  std::chrono::time_point<std::chrono::system_clock> begin;
  std::chrono::time_point<std::chrono::steady_clock> run_begin;
  std::chrono::time_point<std::chrono::steady_clock> run_end;

  /**
   * Where results are saved
   */
  std::optional<std::filesystem::path> file_path;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_EXPERIMENT_H_ */
