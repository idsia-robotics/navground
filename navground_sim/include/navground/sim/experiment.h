/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *  (Run)
 */

#ifndef NAVGROUND_SIM_EXPERIMENT_H_
#define NAVGROUND_SIM_EXPERIMENT_H_

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
  std::vector<std::vector<float>> task_events_data; // one vector per agent
  std::vector<unsigned> task_events;

  std::optional<unsigned> index_of_agent(const Agent * agent) const {
    if (indices.count(agent)) {
      return indices.at(agent);
    }
    return std::nullopt;
  }

 private:
  bool record;
  float * pose_ptr;
  float * twist_ptr;
  float * cmd_ptr;
  float * target_ptr;
  float * safety_violation_ptr;

  std::map<const Agent *, unsigned> indices;

  /**
   * @brief      Init a trace recording
   *
   * @param[in]  world  The world
   * @param[in]  group  The dataset group where to store data
   * @param[in]  steps  The maximal number of steps that will be recorded
   */
  void init(const World& world, HighFive::Group * group, unsigned steps);
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
   * @param      group  The dataset group where to store data
   */
  void finalize(const World &world, HighFive::Group  * group);
};

/**
 * @brief      An experiment supervise the execution of a simulation.
 *
 * When performing one run using \ref run_once
 *
 * 1. it initializes a world from its scenario by calling \ref
 *    Scenario::init_world and setup the dataset.
 *
 * 2. in run the simulation, step by step, collecting and storing data about the
 *    agents in the dataset.
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
        initialized(false),
        step(0),
        world(),
        callbacks(),
        run_index(0),
        file(),
        run_group() {}

  /**
   * @brief      Perform a single run
   *
   * @param[in]  seed  The index (and random seed) of the run
   */
  void run_once(int seed);

  /**
   * @brief      Perform all runs
   */
  void run();

  /**
   * @brief      Adds a callback to be executed after each simulation step.
   *
   * @param[in]  value  The callback
   */
  void add_callback(const Callback& value) { callbacks.push_back(value); }

  /**
   * @brief      Gets the database path.
   *
   * @return     The path.
   */
  std::string get_path() const {
    if (file) {
      std::cerr << file->getPath() << " : " << file->getName() << std::endl;
      return file->getPath();
    }
    return "";
  }

  std::shared_ptr<World> get_world() const {
    return world;
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
   * Number of steps per run
   */
  unsigned steps;

  /**
   * Where to save the results
   */
  std::filesystem::path save_directory;

  /**
   * The trace
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

 protected:
  void init_run(int seed);

  virtual std::shared_ptr<World> make_world() {
    return std::make_shared<World>();
  }

  virtual std::string dump();

  void init_dataset();
  void init_dataset_run(unsigned index);

  bool initialized;
  unsigned step;
  std::shared_ptr<World> world;
  std::vector<Callback> callbacks;
  unsigned run_index;
  std::shared_ptr<HighFive::File> file;
  std::shared_ptr<HighFive::Group> run_group;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_EXPERIMENT_H_ */
