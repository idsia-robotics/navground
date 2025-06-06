#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>

#include "docstrings.h"
#include "navground/core/attribute.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/build_info.h"
#include "navground/core/kinematics.h"
#include "navground/core/types.h"
#include "navground/core/version.h"
#include "navground/core/yaml/yaml.h"
#include "navground/core_py/behavior_modulation.h"
#include "navground/core_py/collection.h"
#include "navground/core_py/pickle.h"
#include "navground/core_py/register.h"
#include "navground/core_py/version.h"
#include "navground/core_py/yaml.h"
#include "navground/sim/build_info.h"
#include "navground/sim/dataset.h"
#include "navground/sim/experiment.h"
#include "navground/sim/experimental_run.h"
#include "navground/sim/probe.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/scenario.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/corridor.h"
#include "navground/sim/scenarios/cross.h"
#include "navground/sim/scenarios/cross_torus.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/state_estimations/geometric_bounded.h"
#include "navground/sim/state_estimations/gridmap_state_estimation.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/state_estimations/sensor_boundary.h"
#include "navground/sim/state_estimations/sensor_combination.h"
#include "navground/sim/state_estimations/sensor_discs.h"
#include "navground/sim/state_estimations/sensor_lidar.h"
#include "navground/sim/state_estimations/sensor_marker.h"
#include "navground/sim/state_estimations/sensor_odometry.h"
#include "navground/sim/task.h"
#include "navground/sim/tasks/direction.h"
#include "navground/sim/tasks/go_to_pose.h"
#include "navground/sim/tasks/waypoints.h"
#include "navground/sim/version.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/schema.h"
#include "navground/sim/yaml/schema_sim.h"
#include "navground/sim/yaml/world.h"
#include "version.h"

using namespace navground::core;
using namespace navground::sim;

std::string sampler_to_string(const PropertySampler &value) {
  return "<Sampler: " + value.type_name += ">";
}

static navground::core::BuildDependencies build_dependencies_sim_py() {
  py::module_ core = py::module_::import("navground.core");
  const auto bi =
      core.attr("get_build_info")().cast<navground::core::BuildInfo>();
  return {
      {"core",
       {navground::core::build_info(), navground::core::get_build_info()}},
      {"sim", {navground::sim::build_info(), navground::sim::get_build_info()}},
      {"core_py", {navground::core_py::build_info(), bi}}};
}

namespace py = pybind11;

// PYBIND11_MAKE_OPAQUE(std::map<unsigned, ExperimentalRun>);

// PYBIND11_MAKE_OPAQUE(std::vector<Scenario::Group>);

void set_dataset_type_py(Dataset &dataset, const py::object &obj);
void set_dataset_data_py(Dataset &dataset, const py::array &obj,
                         bool append = false,
                         std::optional<py::dtype> = std::nullopt);

template <typename T> struct get<T, py::object> {
  static typename T::Native *ptr(const py::object &c) {
    return c.cast<typename T::Native *>();
  }
};

template <> struct navground::sim::add_modulation<py::object, py::object> {
  static void call(py::object &behavior, py::object modulation) {
    return add_modulation_py(behavior, modulation);
  }
};

struct PyBehavior : public Behavior {
  using C = py::object;
  using Native = Behavior;

  static py::object make_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("Behavior").attr("make_type")(type);
  }

  static bool has_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("Behavior").attr("has_type")(type).cast<bool>();
  }

  // Should cache
  static std::map<std::string, Properties> type_properties() {
    py::module_ nav = py::module_::import("navground.core");
    auto value = nav.attr("Behavior").attr("type_properties");
    return value.cast<std::map<std::string, Properties>>();
  }
};

struct PyKinematics : public Kinematics {
  using C = py::object;
  using Native = Kinematics;

  static py::object make_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("Kinematics").attr("make_type")(type);
  }

  // Should cache
  static std::map<std::string, Properties> type_properties() {
    py::module_ nav = py::module_::import("navground.core");
    auto value = nav.attr("Kinematics").attr("type_properties");
    return value.cast<std::map<std::string, Properties>>();
  }

  static bool has_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("Kinematics").attr("has_type")(type).cast<bool>();
  }
};

struct PyBehaviorModulation : public BehaviorModulation {
  using C = py::object;
  using Native = BehaviorModulation;

  static py::object make_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("BehaviorModulation").attr("make_type")(type);
  }

  static bool has_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("BehaviorModulation").attr("has_type")(type).cast<bool>();
  }

  // Should cache
  static std::map<std::string, Properties> type_properties() {
    py::module_ nav = py::module_::import("navground.core");
    auto value = nav.attr("BehaviorModulation").attr("type_properties");
    return value.cast<std::map<std::string, Properties>>();
  }
};

struct PyTask : public Task, public PyHasRegister<Task> {
  /* Inherit the constructors */
  using Task::Task;
  using PyHasRegister<Task>::C;
  // using PyHasRegister<Task>::get_type;
  // using PyHasRegister<Task>::decode;
  // using PyHasRegister<Task>::encode;
  using Native = Task;
  using Task::log_event;

  void update(Agent *agent, World *world, ng_float_t time) override {
    PYBIND11_OVERRIDE(void, Task, update, agent, world, time);
  }

  void prepare(Agent *agent, World *world) override {
    PYBIND11_OVERRIDE(void, Task, prepare, agent, world);
  }

  void close() override { PYBIND11_OVERRIDE(void, Task, close); }

  size_t get_log_size() const override {
    PYBIND11_OVERRIDE(size_t, Task, get_log_size);
  }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE

  bool done() const override { PYBIND11_OVERRIDE(bool, Task, done); }
};

struct PyStateEstimation : public StateEstimation,
                           public PyHasRegister<StateEstimation> {
  /* Inherit the constructors */
  using StateEstimation::StateEstimation;
  using PyHasRegister<StateEstimation>::C;
  // using PyHasRegister<StateEstimation>::get_type;
  // using PyHasRegister<StateEstimation>::decode;
  // using PyHasRegister<StateEstimation>::encode;
  using Native = StateEstimation;

  void update(Agent *agent, World *world, EnvironmentState *state) override {
    PYBIND11_OVERRIDE(void, StateEstimation, update, agent, world, state);
  }

  void prepare(Agent *agent, World *world) override {
    PYBIND11_OVERRIDE(void, StateEstimation, prepare, agent, world);
  }

  void close() override { PYBIND11_OVERRIDE(void, StateEstimation, close); }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE
};

struct PySensor : public Sensor, public PyStateEstimation {
  // using PyStateEstimation::get_type;
  // using PyStateEstimation::decode;
  // using PyStateEstimation::encode;

  using Sensor::Sensor;

  Sensor::Description get_description() const override {
    PYBIND11_OVERRIDE_PURE(Sensor::Description, Sensor, get_description);
  }

  void update(Agent *agent, World *world, EnvironmentState *state) override {
    PYBIND11_OVERRIDE(void, Sensor, update, agent, world, state);
  }

  void prepare(Agent *agent, World *world) override {
    PYBIND11_OVERRIDE(void, Sensor, prepare, agent, world);
  }

  void prepare_state(navground::core::SensingState &state) const override {
    PYBIND11_OVERRIDE(void, Sensor, prepare_state, state);
  }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE

  static const inline std::string type =
      register_abstract_type<PySensor>("Sensor", Sensor::properties, nullptr);
};

class PyAgent : public Agent {
public:
  using B = PyBehavior;
  using M = PyBehaviorModulation;
  using K = PyKinematics;
  using T = PyTask;
  using S = PyStateEstimation;
  using Native = PyAgent;

  virtual ~PyAgent() = default;

  using C = py::object;

  explicit PyAgent(ng_float_t radius = 0,
                   const py::object &behavior = py::none(),
                   const py::object &kinematics = py::none(),
                   const py::object &task = py::none(),
                   const std::vector<py::object> &estimations = {},
                   ng_float_t control_period = 0, unsigned id = 0)
      : Agent(radius, nullptr, nullptr, nullptr,
              std::vector<std::shared_ptr<StateEstimation>>{}, control_period,
              id) {
    set_kinematics(kinematics);
    set_behavior(behavior);
    set_state_estimations(estimations);
    set_task(task);
  }

  static py::object make(ng_float_t radius = 0,
                         const py::object &behavior = py::none(),
                         const py::object &kinematics = py::none(),
                         const py::object &task = py::none(),
                         const std::vector<py::object> &estimations = {},
                         ng_float_t control_period = 0, unsigned id = 0) {
    auto a = std::make_shared<PyAgent>(radius, behavior, kinematics, task,
                                       estimations, control_period, id);
#if 0
    auto a = std::make_shared<PyAgent>(radius, nullptr, nullptr, nullptr,
                                       nullptr, control_period, id);
    a->set_kinematics(kinematics);
    a->set_behavior(behavior);
    a->set_state_estimation(estimation);
    a->set_task(task);
#endif
    return py::cast(a);
  }

  void set_kinematics(const py::object &obj) {
    py_kinematics = obj;
    Agent::set_kinematics(obj.cast<std::shared_ptr<Kinematics>>());
  }

  void set_behavior(const py::object &obj) {
    py_behavior = obj;
    Agent::set_behavior(obj.cast<std::shared_ptr<Behavior>>());
  }

  void set_state_estimation(const py::object &obj) {
    py_state_estimations = {obj};
    Agent::set_state_estimation(obj.cast<std::shared_ptr<StateEstimation>>());
  }

  void set_state_estimations(const std::vector<py::object> &objs) {
    py_state_estimations = objs;
    std::vector<std::shared_ptr<StateEstimation>> ses(objs.size());
    std::transform(objs.cbegin(), objs.cend(), ses.begin(),
                   [](const py::object &obj) {
                     return obj.cast<std::shared_ptr<StateEstimation>>();
                   });
    Agent::set_state_estimations(ses);
  }

  void set_task(const py::object &obj) {
    py_task = obj;
    Agent::set_task(obj.cast<std::shared_ptr<Task>>());
  }

  // GeometricState *get_geometric_state() const override {
  //   try {
  //     return py_behavior.cast<GeometricState *>();
  //   } catch (const py::cast_error &e) {
  //     return nullptr;
  //   }
  // }

private:
  py::object py_kinematics;
  py::object py_behavior;
  std::vector<py::object> py_state_estimations;
  py::object py_task;
};

struct PyWorld : public World {
  /* Inherit the constructors */
  using World::World;
  using A = PyAgent;
  using Native = World;

  ~PyWorld() = default;

  std::vector<py::object> py_agents;
  std::optional<py::object> numpy_rng;

  void add_agent(const py::object &value) {
    py_agents.push_back(value);
    std::shared_ptr<Agent> agent = value.cast<std::shared_ptr<Agent>>();
    World::add_agent(agent);
  }

  void set_seed(unsigned seed) {
    if (seed != get_seed()) {
      py::module_ np = py::module_::import("numpy");
      numpy_rng = np.attr("random").attr("default_rng")(seed);
    }
    World::set_seed(seed);
  }

  void set_random_generator(py::object value) { numpy_rng = value; }

  py::object get_random_generator() {
    if (!numpy_rng) {
      py::module_ np = py::module_::import("numpy");
      numpy_rng = np.attr("random").attr("default_rng")(get_seed());
    }
    return *numpy_rng;
  }

  void copy_random_generator(PyWorld &world) {
    World::set_random_generator(
        static_cast<World &>(world).get_random_generator());
    set_random_generator(world.get_random_generator());
  }
};

struct PyGroup : public Scenario::Group {
  /* Inherit the constructors */
  PyGroup() {};

  void add_to_world(World *world,
                    std::optional<unsigned> seed = std::nullopt) override {
    PYBIND11_OVERRIDE_PURE(void, Scenario::Group, add_to_world, world, seed);
  }
};

void add_group_py(const py::object &obj, const py::object value) {
  add_py_item(obj, value, "groups");
  std::shared_ptr<Scenario::Group> g;
  try {
    g = value.cast<std::shared_ptr<AgentSampler<PyWorld>>>();
  } catch (py::cast_error &) {
    g = value.cast<std::shared_ptr<Scenario::Group>>();
  }
  obj.cast<Scenario &>().add_group(g);
}

void remove_group_py(const py::object &obj, const py::object value) {
  obj.cast<Scenario &>().remove_group(
      value.cast<std::shared_ptr<Scenario::Group>>());
  remove_py_item(obj, value, "groups");
}

void remove_group_at_index_py(const py::object &obj, size_t index) {
  auto &scenario = obj.cast<Scenario &>();
  const auto group = scenario.get_group(index);
  if (group) {
    scenario.remove_group_at_index(index);
    remove_py_item(obj, py::cast(group), "groups");
  }
}

void clear_groups_py(const py::object &obj) {
  obj.cast<Scenario &>().clear_groups();
  clear_collection_py(obj, "groups");
}

struct PyScenario : public Scenario, public PyHasRegister<Scenario> {
  /* Inherit the constructors */
  using Scenario::Scenario;
  using Native = Scenario;

  void init_world(World *world,
                  std::optional<int> seed = std::nullopt) override {
    PYBIND11_OVERRIDE(void, Scenario, init_world, world, seed);
  }

  std::shared_ptr<World> make_world(std::optional<int> seed = std::nullopt) {
    auto world = std::make_shared<PyWorld>();
    // NOTE: creating the py::object before calling
    // `init_world`, let Python initializer store
    // (a reference) to the world.
    py::object py_world = py::cast(world);
    init_world(world.get(), seed);
    apply_inits(world.get());
    return world;
  }

  py::object make_world_py(std::optional<int> seed = std::nullopt) {
    auto world = std::make_shared<PyWorld>();
    py::object py_world = py::cast(world);
    init_world(world.get(), seed);
    apply_inits(world.get());
    return py_world;
  }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE
};

struct PyProbe : public Probe {
  using Probe::Probe;

  void update(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, Probe, update, run);
  }

  void prepare(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, Probe, prepare, run);
  }

  void finalize(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, Probe, finalize, run);
  }
};

struct PyRecordProbe : public RecordProbe {
  using RecordProbe::RecordProbe;

  void update(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, RecordProbe, update, run);
  }

  void prepare(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, RecordProbe, prepare, run);
  }

  void finalize(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, RecordProbe, finalize, run);
  }

  Dataset::Shape get_shape(const World &world) const override {
    PYBIND11_OVERRIDE(Dataset::Shape, RecordProbe, get_shape, world);
  }
};

struct PyGroupRecordProbe : public GroupRecordProbe {
  using GroupRecordProbe::GroupRecordProbe;
  using GroupRecordProbe::ShapeMap;

  void update(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, GroupRecordProbe, update, run);
  }

  void prepare(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, GroupRecordProbe, prepare, run);
  }

  void finalize(ExperimentalRun *run) override {
    PYBIND11_OVERRIDE(void, GroupRecordProbe, finalize, run);
  }

  ShapeMap get_shapes(const World &world,
                      bool use_agent_uid_as_key) const override {
    PYBIND11_OVERRIDE(ShapeMap, GroupRecordProbe, get_shapes, world,
                      use_agent_uid_as_key);
  }
};

static py::object add_record_probe_py(const std::string &name,
                                      const py::object &probe_factory,
                                      ExperimentalRun &run) {
  auto ds = run.add_record(name);
  auto obj = probe_factory.attr("__call__")(ds);
  const auto dtype = obj.attr("dtype");
  set_dataset_type_py(*ds, dtype);
  auto probe = obj.cast<std::shared_ptr<Probe>>();
  run.add_probe(probe);
  return obj;
}

static py::object add_group_record_probe_py(const std::string &name,
                                            const py::object &probe_factory,
                                            ExperimentalRun &run) {
  auto probe_py = probe_factory.attr("__call__")();
  const auto dtype = probe_py.attr("dtype");
  const auto factory = [name, dtype, &run](const std::string &sub_key) {
    auto ds = run.add_record(sub_key, name);
    set_dataset_type_py(*ds, dtype);
    return ds;
  };
  auto probe_cpp = probe_py.cast<std::shared_ptr<GroupRecordProbe>>();
  probe_cpp->set_factory(factory);
  run.add_probe(probe_cpp);
  return probe_py;
}

struct PyExperiment : public Experiment {
  /* Inherit the constructors */
  using Experiment::Experiment;

  // using Native = Experiment;
  using Native = PyExperiment;

  ~PyExperiment() = default;

  // virtual ~PyExperiment() = default;

  py::object py_scenario;
  // Store a python object to allow the simulation (scenario, ...) to set new
  // python attributes
  py::object py_world;

  std::map<unsigned, std::vector<py::object>> _py_probes;
  std::vector<py::object> _py_probe_factories;
  std::map<std::string, py::object> _py_record_probe_factories;
  std::map<std::string, py::object> _py_group_record_probe_factories;
  std::map<bool, std::vector<py::object>> _py_run_callbacks;

  std::shared_ptr<World> make_world() override {
    auto world = std::make_shared<PyWorld>();
    py_world = py::cast(world);
    return world;
  }

  std::string dump() const override { return YAML::dump<PyExperiment>(this); }

  void set_scenario(const py::object &value) {
    py_scenario = value;
    scenario = value.cast<std::shared_ptr<Scenario>>();
  }

  void add_run_callback_py(const py::object &value, bool at_init = false) {
    _py_run_callbacks[at_init].push_back(value);
    add_run_callback(value.cast<RunCallback>(), at_init);
  }

  /**
   * @brief      Register a probe to record data during experimental runs.
   *
   * @param[in]  name   The name to assign to probe.
   * It will be used to define HDF5 groups or datasets when saving data.
   *
   * @param[in]  probe  A callable that generate probes
   */
  void add_probe_py(const py::object &probe_cls) {
    _py_probe_factories.push_back(probe_cls);
  }

  void add_record_probe_py(const std::string &name,
                           const py::object &probe_cls) {
    _py_record_probe_factories[name] = probe_cls;
  }

  void add_group_record_probe_py(const std::string &name,
                                 const py::object &probe_cls) {
    _py_group_record_probe_factories[name] = probe_cls;
  }

  ExperimentalRun &init_run(int seed,
                            std::shared_ptr<World> world = nullptr) override {
    auto &run = Experiment::init_run(seed, world);
    for (const auto &cls : _py_probe_factories) {
      instantiate_probe(cls, run);
    }
    for (const auto &[name, cls] : _py_record_probe_factories) {
      instantiate_record_probe(name, cls, run);
    }
    for (const auto &[name, cls] : _py_group_record_probe_factories) {
      instantiate_group_record_probe(name, cls, run);
    }
    return run;
  }

  void remove_all_runs() override {
    Experiment::remove_all_runs();
    // !! We need to keep the python objects alive for the lifetime of the run
    _py_probes.clear();
  }

  void remove_run(unsigned seed) override {
    Experiment::remove_run(seed);
    // !! We need to keep the python objects alive for the lifetime of the run
    _py_probes.erase(seed);
  }

  void run_in_parallel(
      unsigned number_of_threads, bool keep = true,
      std::optional<unsigned> start_index = std::nullopt,
      std::optional<unsigned> number = std::nullopt,
      std::optional<std::filesystem::path> data_path = std::nullopt) override {
    // std::cout << "run_in_parallel on " << number_of_threads << " threads"
    //           << std::endl;
    start(data_path);
    std::queue<unsigned> indices;
    const unsigned max_index =
        start_index.value_or(run_index) + number.value_or(number_of_runs);
    for (unsigned i = start_index.value_or(run_index); i < max_index; i++) {
      indices.push(i);
    }

    std::mutex mutex;

    auto f = [this, &indices, &mutex, keep]() {
      while (true) {
        mutex.lock();
        if (indices.empty()) {
          mutex.unlock();
          break;
        }
        unsigned i = indices.front();
        indices.pop();
        if (runs.count(i)) {
          mutex.unlock();
          continue;
        }
        ExperimentalRun *sim_run;
        {
          // may call Python => get the GIL
          py::gil_scoped_acquire acquire;
          // may call Python
          sim_run = &(init_run(i));
        }
        mutex.unlock();
        if (!sim_run)
          continue;
        // Run is parallelized!!
        // Only works if there are no Python classes used
        // as Behavior, Kinematics, StateEstimation, or Task
        sim_run->run();
        mutex.lock();
        save_run(*sim_run);
        {
          // may call Python => get the GIL
          py::gil_scoped_acquire acquire;
          for (const auto &cb : run_callbacks[false]) {
            // may call Python
            cb(sim_run);
          }
          if (!keep) {
            // will call Python (world destructor)
            remove_run(i);
          }
        }
        mutex.unlock();
      }
    };

    py::gil_scoped_release release;

    std::vector<std::thread> threads;
    for (size_t i = 0; i < number_of_threads - 1; i++) {
      threads.emplace_back(std::thread(f));
    }
    f();
    for (auto &t : threads) {
      if (t.joinable()) {
        t.join();
      }
    }
    stop();
  }

private:
  void instantiate_probe(const py::object cls, ExperimentalRun &run) {
    auto obj = cls.attr("__call__")();
    auto probe = obj.cast<std::shared_ptr<Probe>>();
    run.add_probe(probe);
    // !! We need to keep the python objects alive for the lifetime of the run
    _py_probes[run.get_seed()].push_back(obj);
  }

  void instantiate_record_probe(const std::string &name,
                                const py::object factory,
                                ExperimentalRun &run) {
    auto obj = ::add_record_probe_py(name, factory, run);
    _py_probes[run.get_seed()].push_back(obj);
  }

  void instantiate_group_record_probe(const std::string &name,
                                      const py::object &probe_factory,
                                      ExperimentalRun &run) {
    auto obj = ::add_group_record_probe_py(name, probe_factory, run);
    _py_probes[run.get_seed()].push_back(obj);
  }
};

namespace YAML {

inline std::shared_ptr<PropertySampler>
load_property_sampler(const std::string &value, const std::string &type_name) {
  YAML::Node node;
  try {
    node = YAML::Load(value);
  } catch (const YAML::ParserException &e) {
    std::cerr << e.what() << std::endl;
    return nullptr;
  }
  auto p = property_sampler(node, type_name);
  if (p && p->valid()) {
    return p;
  }
  return nullptr;
}

template <> struct convert<PyAgent> {
  static Node encode(const PyAgent &rhs) {
    return convert<Agent>::encode(static_cast<const Agent &>(rhs));
  }
  static bool decode(const Node &node, PyAgent &rhs) {
    if (convert<Agent>::decode(node, static_cast<Agent &>(rhs))) {
      if (!rhs.get_behavior() && node["behavior"]) {
        auto value = load_node_py<PyBehavior>(node["behavior"]);
        rhs.set_behavior(value);
      }
      if (!rhs.get_kinematics() && node["kinematics"]) {
        auto value = load_node_py<PyKinematics>(node["kinematics"]);
        rhs.set_kinematics(value);
      }
      if (!rhs.get_task() && node["task"]) {
        auto value = load_node_py<PyTask>(node["task"]);
        rhs.set_task(value);
      }
      if (rhs.get_state_estimations().size() == 0) {
        if (node["state_estimation"]) {
          auto value =
              load_node_py<PyStateEstimation>(node["state_estimation"]);
          rhs.set_state_estimation(value);
        }
        if (node["state_estimations"]) {
          std::vector<py::object> ses;
          for (const auto &c : node["state_estimations"]) {
            ses.push_back(load_node_py<PyStateEstimation>(c));
          }
          rhs.set_state_estimations(ses);
        }
      }
      return true;
    }
    return false;
  }
};

template <> struct convert<std::shared_ptr<PyAgent>> {
  static Node encode(const std::shared_ptr<PyAgent> &rhs) {
    if (rhs) {
      return convert<PyAgent>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node &node, std::shared_ptr<PyAgent> &rhs) {
    rhs = std::make_shared<PyAgent>();
    if (convert<PyAgent>::decode(node, *rhs)) {
      return true;
    }
    rhs = nullptr;
    return false;
  }
};

template <> py::object load_node_py<PyAgent>(const Node &node) {
  return py::cast(node.as<PyAgent>());
}

// PyWorld load_world(const Node &node) {
//   PyWorld world;
//   convert_world<PyAgent>::decode(node, world);
//   return world;
// }

template <> py::object load_node_py<PyWorld>(const Node &node) {
  auto world = std::make_shared<PyWorld>();
  convert_world<PyAgent>::decode(node, *world);
  return py::cast(world);
}

template <> py::object load_node_py<PyScenario>(const Node &node) {
  auto obj = make_type_from_yaml_py<PyScenario>(node);
  if (obj.is_none()) {
    auto ws = std::make_shared<Scenario>();
    obj = py::cast(ws);
  }
  convert_scenario<PyWorld>::decode(node, obj.cast<Scenario &>());
  return obj;
}

template <> py::object load_node_py<AgentSampler<PyWorld>>(const Node &node) {
  auto group = std::make_shared<AgentSampler<PyWorld>>();
  convert<AgentSampler<PyWorld>>::decode(node, *group);
  return py::cast(group);
}

void update_scenario(Scenario &scenario, const Node &node) {
  convert_scenario<PyWorld>::decode(node, scenario);
}

template <> std::string dump(const Scenario *sampler) {
  if (!sampler)
    return "";
  auto node = convert_scenario<PyWorld>::encode(*sampler);
  Emitter out;
  out << node;
  return std::string(out.c_str());
}

// TODO(move to PyExperiment that creates a PyWorld instead of a World)
// Experiment load_experiment(const Node &node) {
//   Experiment experiment;
//   convert_experiment<PyAgent, PyBehavior, PyKinematics, PyTask,
//                      PyStateEstimation, PyWorld>::decode(node, experiment);
//   return experiment;
// };

// std::string dump_experiment(const Experiment *experiment) {
//   if (!experiment) return "";
//   // const auto node = convert_experiment<PyAgent, PyBehavior,
//   PyKinematics,
//   // PyTask,
//   //                    PyStateEstimation,
//   //                    PyWorld>::encode(*experiment);
//   Node node;
//   Emitter out;
//   out << node;
//   return std::string(out.c_str());
// };

template <> struct convert<PyExperiment> {
  static Node encode(const PyExperiment &rhs) {
    Node node = convert_experiment::encode(rhs);
    if (rhs.scenario) {
      node["scenario"] = convert_scenario<PyWorld>::encode(*rhs.scenario);
    }
    return node;
  }
  static bool decode(const Node &node, PyExperiment &rhs) {
    if (convert_experiment::decode(node, rhs)) {
      if (node["scenario"]) {
        rhs.set_scenario(load_node_py<PyScenario>(node["scenario"]));
      }
      return true;
    }
    return false;
  }
};

template <> py::object load_node_py<PyExperiment>(const Node &node) {
  return py::cast(node.as<PyExperiment>());
}

// template <>
// std::string dump(const Experiment *experiment) {
//   if (!experiment) return "";
//   auto node = YAML::Node(*experiment);
//   if (experiment->scenario) {
//     node["scenario"] =
//     convert_scenario<PyWorld>::encode(*(experiment->scenario));
//   }
//   Emitter out;
//   out << node;
//   return std::string(out.c_str());
// };

} // namespace YAML

#if 0
static py::memoryview empty_unsigned_view() {
  static unsigned empty_unsigned_buffer;
  return py::memoryview::from_memory(&empty_unsigned_buffer, 0, true);
}

static py::memoryview empty_float_view() {
  static float empty_float_buffer;
  return py::memoryview::from_memory(&empty_float_buffer, 0, true);
}

// static py::memoryview empty_float_view = py::memoryview::from_memory(
//     &empty_float_buffer, {0}, {static_cast<unsigned>(sizeof(float))});

static py::memoryview run_view(const ExperimentalRun *run, const float *data) {
  const std::array<ssize_t, 3> shape{run->steps, run->number, 3};
  const std::array<ssize_t, 3> strides{
      static_cast<ssize_t>(sizeof(float) * 3 * run->number),
      3 * sizeof(float), sizeof(float)};
  return py::memoryview::from_buffer(data, shape, strides);
}
#endif

static std::vector<ssize_t> convert(const std::vector<size_t> &value) {
  std::vector<ssize_t> r;
  std::transform(value.cbegin(), value.cend(), std::back_inserter(r),
                 [](size_t c) { return static_cast<ssize_t>(c); });
  return r;
}

static py::array as_array(const Dataset &dataset) {
  // auto size = dataset.size();
  auto shape = convert(dataset.get_shape());
  return std::visit(
      [&shape](auto &&arg) {
        using T = std::remove_reference_t<decltype(arg[0])>;
        py::array_t _arr = py::array_t<T>();
        py::detail::array_proxy(_arr.ptr())->flags &=
            ~py::detail::npy_api::NPY_ARRAY_WRITEABLE_;
        return py::array(shape, arg.data(), _arr);
      },
      dataset.get_data());
}

#if 0
static std::map<std::string, py::array> as_dict_array(
    const std::map<std::string, std::shared_ptr<Dataset>> &records) {
  std::map<std::string, py::array> m;
  for (const auto &[key, ds] : records) {
    m.emplace(key, as_array(*ds));
  }
  return m;
}
#endif

template <typename T> static py::array make_empty_array() {
  return py::array(0, static_cast<const T *>(nullptr));
}

void set_dataset_type_py(Dataset &dataset, const py::object &obj) {
  py::module_ np = py::module_::import("numpy");
  py::dtype dtype = np.attr("dtype")(obj);
  if (dtype.equal(py::dtype::of<int8_t>())) {
    dataset.set_dtype<int8_t>();
  } else if (dtype.equal(py::dtype::of<int16_t>())) {
    dataset.set_dtype<int16_t>();
  } else if (dtype.equal(py::dtype::of<int32_t>())) {
    dataset.set_dtype<int32_t>();
  } else if (dtype.equal(py::dtype::of<int64_t>())) {
    dataset.set_dtype<int64_t>();
  } else if (dtype.equal(py::dtype::of<uint8_t>()) ||
             dtype.equal(py::dtype::of<bool>())) {
    dataset.set_dtype<uint8_t>();
  } else if (dtype.equal(py::dtype::of<uint16_t>())) {
    dataset.set_dtype<uint16_t>();
  } else if (dtype.equal(py::dtype::of<uint32_t>())) {
    dataset.set_dtype<uint32_t>();
  } else if (dtype.equal(py::dtype::of<uint64_t>())) {
    dataset.set_dtype<uint64_t>();
  } else if (dtype.equal(py::dtype::of<float>())) {
    dataset.set_dtype<float>();
  } else if (dtype.equal(py::dtype::of<double>())) {
    dataset.set_dtype<double>();
  } else {
    py::print("Type unknown", dtype);
  }
}

Dataset::Data data_of_type(py::dtype dtype, void *ptr, const size_t size) {
  if (dtype.equal(py::dtype::of<int8_t>())) {
    auto begin = reinterpret_cast<int8_t *>(ptr);
    return std::vector<int8_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<int16_t>())) {
    auto begin = reinterpret_cast<int16_t *>(ptr);
    return std::vector<int16_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<int32_t>())) {
    auto begin = reinterpret_cast<int32_t *>(ptr);
    return std::vector<int32_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<int64_t>())) {
    auto begin = reinterpret_cast<int64_t *>(ptr);
    return std::vector<int64_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<uint8_t>()) ||
             dtype.equal(py::dtype::of<bool>())) {
    auto begin = reinterpret_cast<uint8_t *>(ptr);
    return std::vector<uint8_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<uint16_t>())) {
    auto begin = reinterpret_cast<uint16_t *>(ptr);
    return std::vector<uint16_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<uint32_t>())) {
    auto begin = reinterpret_cast<uint32_t *>(ptr);
    return std::vector<uint32_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<uint64_t>())) {
    auto begin = reinterpret_cast<uint64_t *>(ptr);
    return std::vector<uint64_t>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<float>())) {
    auto begin = reinterpret_cast<float *>(ptr);
    return std::vector<float>(begin, begin + size);
  } else if (dtype.equal(py::dtype::of<double>())) {
    auto begin = reinterpret_cast<double *>(ptr);
    return std::vector<double>(begin, begin + size);
  }
  throw(py::type_error("Unsupported dtype " +
                       py::cast<std::string>(py::str(dtype))));
  // std::cerr << "Unknown type" << std::endl;
  // return std::vector<int8_t>();
}

void set_dataset_data_py(Dataset &dataset, const py::array &value, bool append,
                         std::optional<py::dtype> dtype) {
  auto sshape = value.request().shape;
  // So it supports scalars too
  if (sshape.size() == 0) {
    sshape.push_back(1);
  }
  const Dataset::Shape shape(sshape.begin(), sshape.end());
  const Dataset::Shape item_shape(sshape.begin() + 1, sshape.end());
  void *ptr = value.request().ptr;
  const auto count = std::accumulate(std::begin(shape), std::end(shape), 1,
                                     std::multiplies<>{});
  if (!dtype) {
    dtype = value.dtype();
  }
  const auto data = data_of_type(*dtype, ptr, count);
  if (append) {
    dataset.append(data);
  } else {
    dataset.set_data(data);
    dataset.set_item_shape(item_shape);
  }
}

void dataset_push_py(Dataset &dataset, const py::object &value) {
  const auto arr = py::array::ensure(value);
  const auto data = data_of_type(arr.dtype(), arr.request().ptr, 1);
  dataset.append(data);
}

py::dtype get_dataset_type_py(const Dataset &dataset) {
  return std::visit(
      [](auto &&arg) {
        using T = std::remove_reference_t<decltype(arg[0])>;
        return py::dtype::of<T>();
      },
      dataset.get_data());
}

static void init_scenario(Scenario *scenario, py::dict *state) {
  if (state->contains("__py_inits")) {
    const auto ds = (*state)["__py_inits"].cast<py::dict>();
    for (const auto &item : ds) {
      const auto key = item.first.cast<std::string>();
      const auto value = item.second.cast<Scenario::Init>();
      scenario->set_init(key, value);
    }
  }
}

PYBIND11_MODULE(_navground_sim, m) {
  py::options options;
#if PYBIND11_VERSION_MAJOR >= 2 && PYBIND11_VERSION_MINOR >= 10
  options.disable_enum_members_docstring();
#endif

  declare_register<StateEstimation>(m, "StateEstimation");
  declare_register<Task>(m, "Task");
  declare_register<Scenario>(m, "Scenario");
  //  declare_register<PScenario>(m, "Scenario");

  py::class_<Entity, std::shared_ptr<Entity>>(m, "Entity",
                                              DOC(navground, sim, Entity))
      .def_readonly("_uid", &Entity::uid, DOC(navground, sim, Entity, uid))
      .def_property("last_collision_time", &Entity::get_last_collision_time,
                    nullptr,
                    DOC(navground, sim, Entity, property, last_collision_time))
      .def("has_been_in_collision_since", &Entity::has_been_in_collision_since,
           py::arg("time"),
           DOC(navground, sim, Entity, has_been_in_collision_since));

  py::class_<Wall, Entity, std::shared_ptr<Wall>>(m, "Wall",
                                                  DOC(navground, sim, Wall))
      .def(py::init<Vector2, Vector2>(), py::arg("p1"), py::arg("p2"),
           DOC(navground, sim, Wall, Wall))
      .def(py::init<LineSegment>(), py::arg("line"),
           DOC(navground, sim, Wall, Wall, 3))
      .def_readwrite("line", &Wall::line, DOC(navground, sim, Wall, line))
      .def_static("schema", &YAML::schema_py<Wall>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_unique_py<Wall>, py::arg("value"),
                  YAML::load_string_py_doc("wall", "Wall").c_str())
      .def("dump", &YAML::dump<Wall>, YAML::dump_doc());

  py::class_<Obstacle, Entity, std::shared_ptr<Obstacle>>(
      m, "Obstacle", DOC(navground, sim, Obstacle))
      .def(py::init<Vector2, ng_float_t>(), py::arg("position"),
           py::arg("radius"), DOC(navground, sim, Obstacle, Obstacle))
      .def(py::init<Disc>(), py::arg("disc"),
           DOC(navground, sim, Obstacle, Obstacle, 3))
      .def_readwrite("disc", &Obstacle::disc,
                     DOC(navground, sim, Obstacle, disc))
      .def_static("schema", &YAML::schema_py<Obstacle>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_unique_py<Obstacle>,
                  py::arg("value"),
                  YAML::load_string_py_doc("obstacle", "Obstacle").c_str())
      .def("dump", &YAML::dump<Obstacle>, YAML::dump_doc());

  py::class_<BoundingBox>(m, "BoundingBox", "A rectangular region")
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t>(),
           py::arg("min_x"), py::arg("max_x"), py::arg("min_y"),
           py::arg("max_y"),
           R"doc(
Creates a rectangular region

:param min_x:
    Minimal x coordinates
:param max_x:
    Maximal x coordinate
:param min_y:
    Minimal y coordinate
:param max_y:
    Maximal y coordinate
           )doc")
      .def(py::init([](const Vector2 &p1, const Vector2 &p2) {
             return BoundingBox{p1[0], p2[0], p1[1], p2[1]};
           }),
           py::arg("p1"), py::arg("p2"),
           R"doc(
Creates a rectangular region

:param p1: Bottom-left corner
:param p2: Top-right corner
           )doc")
      .def(
          "__eq__",
          [](const BoundingBox &bb, const py::object &obj) {
            try {
              auto other = obj.cast<BoundingBox>();
              return bb.equals(&other);
            } catch (py::cast_error &) {
              return false;
            }
          },
          py::arg("other"))
      .def("__hash__", &BoundingBox::hashCode)
      .def("__repr__",
           [](const BoundingBox &bb) -> py::str {
             py::str r("BoundingBox(min_x=");
             r += py::str(py::cast(bb.getMinX()));
             r += py::str(", max_x=") + py::str(py::cast(bb.getMaxX()));
             r += py::str(", min_y=") + py::str(py::cast(bb.getMinY()));
             r += py::str(", max_y=") + py::str(py::cast(bb.getMaxY()));
             r += py::str(")");
             return r;
           })
      .def_property("min_x", &BoundingBox::getMinX, nullptr)
      .def_property("min_y", &BoundingBox::getMinY, nullptr)
      .def_property("max_x", &BoundingBox::getMaxX, nullptr)
      .def_property("max_y", &BoundingBox::getMaxY, nullptr)
      .def(
          "set_min_x",
          [](const BoundingBox &bb, ng_float_t value) {
            return bb_set_min_x(bb, value);
          },
          py::arg("value"), DOC(navground, sim, bb_set_min_x))
      .def(
          "set_min_y",
          [](const BoundingBox &bb, ng_float_t value) {
            return bb_set_min_y(bb, value);
          },
          py::arg("value"), DOC(navground, sim, bb_set_min_y))
      .def(
          "set_max_x",
          [](const BoundingBox &bb, ng_float_t value) {
            return bb_set_max_x(bb, value);
          },
          py::arg("value"), DOC(navground, sim, bb_set_max_x))
      .def(
          "set_max_y",
          [](const BoundingBox &bb, ng_float_t value) {
            return bb_set_max_y(bb, value);
          },
          py::arg("value"), DOC(navground, sim, bb_set_max_y))
      .def(
          "to_tuple", [](const BoundingBox &bb) { return bb_to_tuple(bb); },
          DOC(navground, sim, bb_to_tuple))
      .def_static(
          "from_tuple",
          [](const py::tuple &t) {
            const auto v = t.cast<
                std::tuple<ng_float_t, ng_float_t, ng_float_t, ng_float_t>>();
            return bb_from_tuple(v);
          },
          DOC(navground, sim, bb_to_tuple))
      .def_static("envelop", &envelop, py::arg("position"), py::arg("radius"),
                  DOC(navground, sim, envelop))
      .def_property("width", &BoundingBox::getWidth, nullptr)
      .def_property("height", &BoundingBox::getHeight, nullptr)
      .def_property(
          "p1",
          [](const BoundingBox &bb) {
            return Vector2(bb.getMinX(), bb.getMinY());
          },
          nullptr, R"doc(
Returns the bottom-left corner
           )doc")
      .def_property(
          "p2",
          [](const BoundingBox &bb) {
            return Vector2(bb.getMaxX(), bb.getMaxY());
          },
          nullptr, R"doc(
Returns the top-right corner
           )doc")
      .def("distance",
           py::overload_cast<const BoundingBox &>(&BoundingBox::distance,
                                                  py::const_),
           py::arg("other"), R"doc(
Computes the distance to another bounding box

:param other: The other bounding box
:type other: :py:class:`navground.sim.BoundingBox`
:returns: The distance
          )doc")
      .def(
          "intersection",
          [](const BoundingBox &bb,
             const BoundingBox &other) -> std::optional<BoundingBox> {
            BoundingBox r;
            if (bb.intersection(other, r)) {
              return r;
            }
            return std::nullopt;
          },
          py::arg("other"), R"doc(
Computes the intersection with another bounding box

:param other: The other bounding box
:type other: :py:class:`navground.sim.BoundingBox`
:returns: The intersection or ``None`` if the bounding boxes do not overlap.
                 )doc")
      .def("intersects",
           py::overload_cast<const BoundingBox &>(&BoundingBox::intersects,
                                                  py::const_),
           py::arg("other"), R"doc(
Checks whether it intersects with another bounding box

:param other: The other bounding box
:type other: :py:class:`navground.sim.BoundingBox`
:returns: ``True`` if the two bounding boxes overlaps
                 )doc")
      .def("covers",
           py::overload_cast<const BoundingBox &>(&BoundingBox::covers,
                                                  py::const_),
           py::arg("other"), R"doc(
Checks whether the bounding box covers another bounding box

:param other: The other bounding box
:type other: :py:class:`navground.sim.BoundingBox`
:returns: ``True`` if the bounding box covers the other bounding box
                 )doc")
      .def(
          "expand_to_include",
          py::overload_cast<const BoundingBox &>(&BoundingBox::expandToInclude),
          py::arg("other"), R"doc(
Expands the bounding box to include another bounding box

:param other: The other bounding box
:type other: :py:class:`navground.sim.BoundingBox`
           )doc")
      .def(
          "contains",
          [](const BoundingBox &bb, const Vector2 &p) {
            return bb.contains(p[0], p[1]);
          },
          py::arg("point"), R"doc(
Checks whether the bounding box includes a point

:param point: The point
:type other: :py:class:`navground.core.Vector2`
:returns: ``True`` if the bounding box contains the point.
           )doc")
      .def(
          "translate",
          [](BoundingBox &bb, const Vector2 &p) { bb.translate(p[0], p[1]); },
          py::arg("delta"), R"doc(
Translates the bounding box by a vector

:param delta: The displacement vector
:type delta: :py:class:`navground.core.Vector2`
           )doc")
      .def(
          "expand_by",
          [](BoundingBox &bb, const Vector2 &p) { bb.expandBy(p[0], p[1]); },
          py::arg("delta"), R"doc(
Expands the bounding box by a vector

:param delta: The expansion vector
:type delta: :py:class:`navground.core.Vector2`
           )doc")
      .def_static("schema", &YAML::schema_py<BoundingBox>, R"doc(
Returns the YAML schema.

:returns: The YAML schema
           )doc")
      .def(py::pickle(
          [](const BoundingBox &value) -> py::tuple {
            return py::cast(bb_to_tuple(value));
          },
          [](py::tuple v) { // __setstate__
            const auto t = v.cast<
                std::tuple<ng_float_t, ng_float_t, ng_float_t, ng_float_t>>();
            return bb_from_tuple(t);
          }));

  py::class_<StateEstimation, PyStateEstimation, HasRegister<StateEstimation>,
             HasProperties, std::shared_ptr<StateEstimation>>
      se(m, "StateEstimation", DOC(navground, sim, StateEstimation));

  py::class_<Task, PyTask, HasRegister<Task>, HasProperties,
             std::shared_ptr<Task>>
      task(m, "Task", DOC(navground, sim, Task));

  py::class_<Agent, Entity, HasAttributes, std::shared_ptr<Agent>>(
      m, "NativeAgent", DOC(navground, sim, Agent))
      .def_readwrite("id", &Agent::id, DOC(navground, sim, Agent, id))
      .def_readwrite("type", &Agent::type, DOC(navground, sim, Agent, type))
      .def_readwrite("color", &Agent::color, DOC(navground, sim, Agent, color))
      .def_property("enabled", &Agent::get_enabled, &Agent::set_enabled,
                    DOC(navground, sim, Agent, property_enabled))
      .def_readwrite("radius", &Agent::radius,
                     DOC(navground, sim, Agent, radius))
      .def_readwrite("control_period", &Agent::control_period,
                     DOC(navground, sim, Agent, control_period))
      .def_property("pose", &Agent::get_pose, &Agent::set_pose,
                    DOC(navground, sim, Agent, property_pose))
      .def_property("twist", &Agent::get_twist, &Agent::set_twist,
                    DOC(navground, sim, Agent, property_twist))
      .def_property("actuated_cmd", &Agent::get_actuated_cmd, nullptr,
                    DOC(navground, sim, Agent, property_actuated_cmd))
      .def_property(
          "last_cmd", py::overload_cast<>(&Agent::get_last_cmd, py::const_),
          &Agent::set_last_cmd, DOC(navground, sim, Agent, property_last_cmd))
      .def("get_last_cmd",
           py::overload_cast<navground::core::Frame>(&Agent::get_last_cmd,
                                                     py::const_),
           py::arg("frame"), DOC(navground, sim, Agent, get_last_cmd))
      .def_readwrite("tags", &Agent::tags, DOC(navground, sim, Agent, tags))
      .def("add_tag", &Agent::add_tag, py::arg("tag"),
           DOC(navground, sim, Agent, add_tag))
      .def("remove_tag", &Agent::remove_tag, py::arg("tag"),
           DOC(navground, sim, Agent, remove_tag))
      .def_property(
          "position", [](const Agent *agent) { return agent->pose.position; },
          [](Agent *agent, const Vector2 &value) {
            agent->pose.position = value;
          },
          "Position")
      .def_property(
          "orientation",
          [](const Agent *agent) { return agent->pose.orientation; },
          [](Agent *agent, const ng_float_t value) {
            agent->pose.orientation = value;
          },
          "Orientation")
      .def_property(
          "velocity", [](const Agent *agent) { return agent->twist.velocity; },
          [](Agent *agent, const Vector2 &value) {
            agent->twist.velocity = value;
          },
          "Velocity")
      .def_property(
          "angular_speed",
          [](const Agent *agent) { return agent->twist.angular_speed; },
          [](Agent *agent, const ng_float_t value) {
            agent->twist.angular_speed = value;
          },
          "Angular speed")
      .def_property("controller", py::overload_cast<>(&Agent::get_controller),
                    nullptr, py::return_value_policy::reference,
                    DOC(navground, sim, Agent, property_controller))
      // .def_readwrite("task", &Agent::task)
      .def_property("task",
                    py::cpp_function(&Agent::get_task,
                                     py::return_value_policy::reference),
                    py::cpp_function(&Agent::set_task, py::keep_alive<1, 2>()),
                    DOC(navground, sim, Agent, property_task))
      // .def_readwrite("state_estimation", &Agent::state_estimation)
      .def_property("state_estimation",
                    py::cpp_function(&Agent::get_state_estimation,
                                     py::return_value_policy::reference),
                    py::cpp_function(&Agent::set_state_estimation,
                                     py::keep_alive<1, 2>()),
                    DOC(navground, sim, Agent, property_state_estimation))
      .def_property("state_estimations",
                    py::cpp_function(&Agent::get_state_estimations,
                                     py::return_value_policy::reference),
                    py::cpp_function(&Agent::set_state_estimations,
                                     py::keep_alive<1, 2>()),
                    DOC(navground, sim, Agent, property_state_estimations))
      // .def_readwrite("behavior", &Agent::behavior)
      .def_property(
          "behavior",
          py::cpp_function(&Agent::get_behavior,
                           py::return_value_policy::reference),
          py::cpp_function(&Agent::set_behavior, py::keep_alive<1, 2>()),
          DOC(navground, sim, Agent, property_behavior))
      // .def_readwrite("kinematics", &Agent::kinematics)
      .def_property(
          "kinematics",
          py::cpp_function(&Agent::get_kinematics,
                           py::return_value_policy::reference),
          py::cpp_function(&Agent::set_kinematics, py::keep_alive<1, 2>()),
          DOC(navground, sim, Agent, property_kinematics))
      .def_property("idle", &Agent::idle, nullptr,
                    DOC(navground, sim, Agent, idle))
      .def("actuate",
           py::overload_cast<const Twist2 &, ng_float_t>(&Agent::actuate),
           py::arg("cmd"), py::arg("time_step"),
           DOC(navground, sim, Agent, actuate, 2))
      .def("has_been_stuck_since", &PyAgent::has_been_stuck_since,
           py::arg("time"), DOC(navground, sim, Agent, has_been_stuck_since))
      .def_property("time_since_stuck", &PyAgent::get_time_since_stuck, nullptr,
                    DOC(navground, sim, Agent, property_time_since_stuck));

  py::class_<PyAgent, Agent, Entity, std::shared_ptr<PyAgent>> agent(
      m, "Agent", py::dynamic_attr(), DOC(navground, sim, Agent));
  agent
      .def(py::init<ng_float_t, const py::object &, const py::object &,
                    const py::object &, const std::vector<py::object> &,
                    ng_float_t, unsigned>(),
           py::arg("radius") = 0, py::arg("behavior") = py::none(),
           py::arg("kinematics") = py::none(), py::arg("task") = py::none(),
           py::arg("state_estimations") = std::vector<py::object>(),
           py::arg("control_period") = 0, py::arg("id") = 0,
           DOC(navground, sim, Agent, Agent))
#if 0
      .def(py::init<float, std::shared_ptr<Behavior>,
                    std::shared_ptr<Kinematics>, std::shared_ptr<Task>,
                    std::shared_ptr<StateEstimation>, float, unsigned>(),
           py::arg("radius") = 0, py::arg("behavior") = nullptr,
           py::arg("kinematics") = nullptr, py::arg("task") = nullptr,
           py::arg("state_estimation") = nullptr,
           py::arg("control_period") = 0, py::arg("id") = 0)
#endif
      .def_property("task", &PyAgent::get_task, &PyAgent::set_task,
                    DOC(navground, sim, Agent, property, task))
      .def_property("state_estimation", &PyAgent::get_state_estimation,
                    &PyAgent::set_state_estimation,
                    DOC(navground, sim, Agent, property, state_estimation))
      .def_property("state_estimations", &PyAgent::get_state_estimations,
                    &PyAgent::set_state_estimations,
                    DOC(navground, sim, Agent, property, state_estimations))
      .def_property("behavior", &PyAgent::get_behavior, &PyAgent::set_behavior,
                    DOC(navground, sim, Agent, property, behavior))
      .def_property("kinematics", &PyAgent::get_kinematics,
                    &PyAgent::set_kinematics,
                    DOC(navground, sim, Agent, property, kinematics))
      .def_static("schema", &YAML::schema_py<Agent>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_py<PyAgent>, py::arg("value"),
                  YAML::load_string_py_doc("agent", "Agent").c_str())
      .def("dump", &YAML::dump<Agent>, YAML::dump_doc());

#if 0 // TODO(Jerome): why?
      .def_property("controller", &PyAgent::get_controller, nullptr,
                    py::return_value_policy::reference,
                    DOC(navground, sim, Agent, property, controller));
#endif

  py::class_<World, HasAttributes, std::shared_ptr<World>>(
      m, "NativeWorld", DOC(navground, sim, World, 2))
      .def(py::init<>(), DOC(navground, sim, World, World))
      .def("add_callback", &World::add_callback, py::arg("callback"),
           py::keep_alive<1, 2>(), DOC(navground, sim, World, add_callback))
      .def("reset_callbacks", &World::reset_callbacks,
           DOC(navground, sim, World, reset_callbacks))
      .def("snap_twists_to_zero", &World::snap_twists_to_zero,
           py::arg("epsilon") = 1e-6,
           DOC(navground, sim, World, snap_twists_to_zero))
      .def("update", &World::update, py::arg("time_step"),
           DOC(navground, sim, World, update))
      .def("set_termination_condition", &World::set_termination_condition,
           py::arg("condition"),
           DOC(navground, sim, World, set_termination_condition))
      .def_property("has_termination_condition",
                    &World::has_termination_condition, nullptr,
                    DOC(navground, sim, World, has_termination_condition))
      .def("run", &World::run, py::arg("steps"), py::arg("time_step"),
           DOC(navground, sim, World, run))
      .def("run_until", &World::run_until, py::arg("condition"),
           py::arg("time_step"), DOC(navground, sim, World, run_until))
      .def("update_dry", &World::update_dry, py::arg("time_step"),
           py::arg("advance_time") = true,
           DOC(navground, sim, World, update_dry))
      .def("actuate", &World::actuate, py::arg("time_step"),
           DOC(navground, sim, World, actuate))
      .def_property("time", &World::get_time, &World::set_time,
                    DOC(navground, sim, World, property_time))
      .def_property("step", &World::get_step, &World::set_step,
                    DOC(navground, sim, World, property_step))
      .def("get_lattice", &World::get_lattice, py::arg("axis"),
           DOC(navground, sim, World, get_lattice))
      .def("set_lattice", &World::set_lattice, py::arg("axis"),
           py::arg("value"), DOC(navground, sim, World, set_lattice))
      .def("add_agent", &World::add_agent, py::keep_alive<1, 2>(),
           py::arg("agent"), DOC(navground, sim, World, add_agent))
      .def("add_obstacle",
           py::overload_cast<const Disc &>(&World::add_obstacle),
           py::arg("disc"), DOC(navground, sim, World, add_obstacle))
      .def("add_obstacle",
           py::overload_cast<const Obstacle &>(&World::add_obstacle),
           py::arg("obstacle"), DOC(navground, sim, World, add_obstacle, 2))
      .def("add_wall", py::overload_cast<const LineSegment &>(&World::add_wall),
           py::arg("line"), DOC(navground, sim, World, add_wall))
      .def("add_wall", py::overload_cast<const Wall &>(&World::add_wall),
           py::arg("wall"), DOC(navground, sim, World, add_wall, 2))
      .def("index_of_agent", &World::index_of_agent, py::arg("agent"),
           DOC(navground, sim, World, index_of_agent))
      .def_property("agents", &World::get_agents, nullptr,
                    DOC(navground, sim, World, property_agents))
      .def_property("walls", &World::get_walls, nullptr,
                    DOC(navground, sim, World, property_walls))
      .def_property("obstacles", &World::get_obstacles, nullptr,
                    DOC(navground, sim, World, property_obstacles))
      .def_property(
          "discs", [](const World &world) { return world.get_discs(); },
          nullptr, DOC(navground, sim, World, property_discs))
      .def_property("line_obstacles", &World::get_line_obstacles, nullptr,
                    DOC(navground, sim, World, property_line_obstacles))
      .def("subdivide_bounding_box", &World::subdivide_bounding_box,
           py::arg("bounding_box"), py::arg("ignore_lattice") = false)
      .def("get_agents_in_region", &World::get_agents_in_region,
           py::arg("bounding_box"), py::return_value_policy::reference_internal,
           DOC(navground, sim, World, get_agents_in_region))
      .def("get_obstacles_in_region", &World::get_obstacles_in_region,
           py::arg("bounding_box"), py::return_value_policy::reference_internal,
           DOC(navground, sim, World, get_obstacles_in_region))
      .def("get_discs_in_region", &World::get_discs_in_region,
           py::arg("bounding_box"), py::arg("ignore_lattice") = false,
           DOC(navground, sim, World, get_discs_in_region))
      .def("get_line_obstacles_in_region", &World::get_line_obstacles_in_region,
           py::arg("bounding_box"),
           DOC(navground, sim, World, get_line_obstacles_in_region))
      .def("get_neighbors", &World::get_neighbors, py::arg("agent"),
           py::arg("distance"), py::arg("ignore_lattice") = false,
           DOC(navground, sim, World, get_neighbors))
      .def_property("collisions", &World::get_collisions,
                    &World::set_collisions,
                    DOC(navground, sim, World, property_collisions))
      .def("compute_safety_violation", &World::compute_safety_violation,
           py::arg("agent"), py::arg("safety_margin") = py::none(),
           DOC(navground, sim, World, compute_safety_violation))
      .def("get_agents_in_collision", &World::get_agents_in_collision,
           py::arg("duration") = 0.0,
           DOC(navground, sim, World, get_agents_in_collision))
      .def("get_agents_in_deadlock", &World::get_agents_in_deadlock,
           py::arg("duration") = 0.0,
           DOC(navground, sim, World, get_agents_in_deadlock))
      .def("agents_are_idle", &World::agents_are_idle,
           DOC(navground, sim, World, agents_are_idle))
      .def("space_agents_apart", &World::space_agents_apart,
           py::arg("minimal_distance") = 0,
           py::arg("with_safety_margin") = false,
           py::arg("max_iterations") = 10,
           DOC(navground, sim, World, space_agents_apart))
      .def_property("seed", &World::get_seed, &World::set_seed,
                    DOC(navground, sim, World, property_seed))
      .def("get_entity", &World::get_entity, py::arg("uid"),
           py::return_value_policy::reference,
           DOC(navground, sim, World, get_entity))
      .def("_prepare", &World::prepare)
      .def("_close", &World::close)
      .def("agents_are_idle_or_stuck", &World::agents_are_idle_or_stuck,
           DOC(navground, sim, World, agents_are_idle_or_stuck))
      .def("in_collision", &World::in_collision, py::arg("e1"), py::arg("e2"),
           DOC(navground, sim, World, in_collision))
      .def("record_collision", &World::record_collision, py::arg("e1"),
           py::arg("e2"), DOC(navground, sim, World, record_collision))
      .def("clear_collisions", &World::clear_collisions,
           DOC(navground, sim, World, clear_collisions))
      .def_property("minimal_bounding_box", &World::get_minimal_bounding_box,
                    nullptr,
                    DOC(navground, sim, World, property_minimal_bounding_box))
      .def_property("bounding_box", &World::get_bounding_box,
                    &World::set_bounding_box,
                    DOC(navground, sim, World, property_bounding_box))
      .def("copy_random_generator", &World::copy_random_generator,
           py::arg("world"), DOC(navground, sim, World, copy_random_generator))
      .def("add_random_obstacles", &World::add_random_obstacles,
           py::arg("number"), py::arg("min_radius"), py::arg("max_radius"),
           py::arg("margin") = 0.0, py::arg("max_tries") = 1000,
           DOC(navground, sim, World, add_random_obstacles))
      .def("get_lattice_grid", &World::get_lattice_grid,
           py::arg("include_zero") = true, py::arg("c8") = true,
           DOC(navground, sim, World, get_lattice_grid))
      .def("should_terminate", &World::should_terminate,
           DOC(navground, sim, World, should_terminate))
      .def_static("schema", &YAML::schema_py<World>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_py<PyWorld>, py::arg("value"),
                  YAML::load_string_py_doc("world", "World").c_str())
      .def("dump", &YAML::dump<World>, YAML::dump_doc());

  py::class_<PyWorld, World, HasAttributes, std::shared_ptr<PyWorld>> world(
      m, "World", py::dynamic_attr(), DOC(navground, sim, World));
  world.def(py::init<>(), DOC(navground, sim, World, World))
      .def("add_agent", &PyWorld::add_agent, py::arg("agent"),
           DOC(navground, sim, World, add_agent))
      .def_property("seed", &PyWorld::get_seed, &PyWorld::set_seed,
                    DOC(navground, sim, World, property_seed))
      .def_property("random_generator", &PyWorld::get_random_generator,
                    &PyWorld::set_random_generator, R"doc(
The random generator.

:rtype: :py:class:`numpy.random.Generator`

)doc")
      .def("copy_random_generator", &PyWorld::copy_random_generator,
           py::arg("world"), DOC(navground, sim, World, copy_random_generator));

  se.def(py::init<>(), DOC(navground, sim, StateEstimation, StateEstimation))
      .def("update",
           py::overload_cast<Agent *, World *, EnvironmentState *>(
               &StateEstimation::update),
           py::arg("agent"), py::arg("world"), py::arg("state"),
           DOC(navground, sim, StateEstimation, update))
      .def(
          "prepare",
          [](PyStateEstimation *se, Agent *agent, World *world) {
            se->prepare(agent, world);
          },
          py::arg("agent"), py::arg("world"),
          DOC(navground, sim, StateEstimation, prepare))
      .def(
          "close", [](PyStateEstimation *se) { se->close(); },
          DOC(navground, sim, StateEstimation, close))
      .def_static(
          "load", &YAML::load_string_py<PyStateEstimation>, py::arg("value"),
          YAML::load_string_py_doc("state estimation", "StateEstimation")
              .c_str());

  py::class_<BoundedStateEstimation, StateEstimation,
             std::shared_ptr<BoundedStateEstimation>>
      bse(m, "BoundedStateEstimation",
          DOC(navground, sim, BoundedStateEstimation));
  bse.def(py::init<ng_float_t, bool>(),
          // py::arg("field_of_view") = 0.0,
          py::arg("range") = 0.0, py::arg("update_static_obstacles") = false,
          DOC(navground, sim, BoundedStateEstimation, BoundedStateEstimation))
      // .def_property("field_of_view",
      // &BoundedStateEstimation::get_field_of_view,
      //               &BoundedStateEstimation::set_field_of_view)
      .def_property("range", &BoundedStateEstimation::get_range,
                    &BoundedStateEstimation::set_range,
                    DOC(navground, sim, BoundedStateEstimation, property_range))
      .def_property("update_static_obstacles",
                    &BoundedStateEstimation::get_update_static_obstacles,
                    &BoundedStateEstimation::set_update_static_obstacles,
                    DOC(navground, sim, BoundedStateEstimation,
                        property_update_static_obstacles))
      .def("_neighbors_of_agent", &BoundedStateEstimation::neighbors_of_agent,
           py::arg("agent"), py::arg("world"),
           DOC(navground, sim, BoundedStateEstimation, neighbors_of_agent));

  py::class_<Sensor, PySensor, StateEstimation, std::shared_ptr<Sensor>> sse(
      m, "Sensor", DOC(navground, sim, Sensor));
  sse.def(py::init<const std::string &>(), py::arg("name") = "",
          DOC(navground, sim, Sensor, Sensor))
      .def("get_description", &Sensor::get_description,
           DOC(navground, sim, Sensor, get_description))
      .def_property("description", &Sensor::get_description, nullptr,
                    DOC(navground, sim, Sensor, property_description))
      .def_property("name", &Sensor::get_name, &Sensor::set_name,
                    DOC(navground, sim, Sensor, property_name))
      .def("prepare_state", &Sensor::prepare_state, py::arg("state"),
           DOC(navground, sim, Sensor, prepare_state))
      .def(
          "get_field_name",
          [](const Sensor &se, const std::string &field) {
            return se.get_field_name(field);
          },
          py::arg("field"), DOC(navground, sim, Sensor, get_field_name))
      .def("get_or_init_buffer", &Sensor::get_or_init_buffer, py::arg("state"),
           py::arg("field"), py::return_value_policy::reference,
           DOC(navground, sim, Sensor, get_or_init_buffer))
      .def_static("load", &YAML::load_string_py<PyStateEstimation, Sensor>,
                  py::arg("value"),
                  YAML::load_string_py_doc("sensor", "Sensor").c_str());

  py::class_<LidarStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<LidarStateEstimation>>
      lse(m, "LidarStateEstimation", DOC(navground, sim, LidarStateEstimation));

  py::class_<LidarStateEstimation::Scan>(
      lse, "Scan", DOC(navground, sim, LidarStateEstimation, Scan))
      .def_property(
          "ranges",
          [](const LidarStateEstimation::Scan &scan) { return scan.ranges; },
          nullptr, DOC(navground, sim, LidarStateEstimation, Scan, ranges))
      .def_readwrite(
          "start_angle", &LidarStateEstimation::Scan::start_angle,
          DOC(navground, sim, LidarStateEstimation, Scan, start_angle))
      .def_readwrite("fov", &LidarStateEstimation::Scan::fov,
                     DOC(navground, sim, LidarStateEstimation, Scan, fov))
      .def_readwrite("max_range", &LidarStateEstimation::Scan::max_range,
                     DOC(navground, sim, LidarStateEstimation, Scan, max_range))
      .def_property(
          "angular_increment",
          &LidarStateEstimation::Scan::get_angular_increment, nullptr,
          DOC(navground, sim, LidarStateEstimation, Scan, angular_increment))
      .def_property("angles", &LidarStateEstimation::Scan::get_angles, nullptr,
                    DOC(navground, sim, LidarStateEstimation, Scan, angles));

  lse.def(
         py::init<ng_float_t, ng_float_t, ng_float_t, unsigned, const Vector2 &,
                  ng_float_t, ng_float_t, const std::string &>(),
         py::arg("range") = 0.0, py::arg("start_angle") = -PI,
         py::arg("field_of_view") = TWO_PI, py::arg("resolution") = 100,
         py::arg("position") = Vector2::Zero(), py::arg("error_bias") = 0,
         py::arg("error_std_dev") = 0, py::arg("name") = "",
         DOC(navground, sim, LidarStateEstimation, LidarStateEstimation))
      .def_property("range", &LidarStateEstimation::get_range,
                    &LidarStateEstimation::set_range,
                    DOC(navground, sim, LidarStateEstimation, property_range))
      .def_property(
          "start_angle", &LidarStateEstimation::get_start_angle,
          &LidarStateEstimation::set_start_angle,
          DOC(navground, sim, LidarStateEstimation, property_start_angle))
      .def_property(
          "field_of_view", &LidarStateEstimation::get_field_of_view,
          &LidarStateEstimation::set_field_of_view,
          DOC(navground, sim, LidarStateEstimation, property_field_of_view))
      .def_property(
          "resolution", &LidarStateEstimation::get_resolution,
          &LidarStateEstimation::set_resolution,
          DOC(navground, sim, LidarStateEstimation, property_resolution))
      .def_property(
          "position", &LidarStateEstimation::get_position,
          &LidarStateEstimation::set_position,
          DOC(navground, sim, LidarStateEstimation, property_position))
      .def_property(
          "error_bias", &LidarStateEstimation::get_error_bias,
          &LidarStateEstimation::set_error_bias,
          DOC(navground, sim, LidarStateEstimation, property_error_bias))
      .def_property(
          "error_std_dev", &LidarStateEstimation::get_error_std_dev,
          &LidarStateEstimation::set_error_std_dev,
          DOC(navground, sim, LidarStateEstimation, property_error_std_dev))
      .def_property(
          "angular_increment", &LidarStateEstimation::get_angular_increment,
          nullptr,
          DOC(navground, sim, LidarStateEstimation, property_angular_increment))
      .def_property("angles", &LidarStateEstimation::get_angles, nullptr,
                    DOC(navground, sim, LidarStateEstimation, property_angles))
      .def_static(
          "read_scan_with_name", &LidarStateEstimation::read_scan_with_name,
          py::arg("state"), py::arg("name"),
          DOC(navground, sim, LidarStateEstimation, read_scan_with_name))
      .def("read_scan", &LidarStateEstimation::read_scan, py::arg("state"),
           DOC(navground, sim, LidarStateEstimation, read_scan))
      .def_static(
          "read_ranges_with_name", &LidarStateEstimation::read_ranges_with_name,
          py::arg("state"), py::arg("name"),
          DOC(navground, sim, LidarStateEstimation, read_ranges_with_name))
      .def("read_ranges", &LidarStateEstimation::read_ranges, py::arg("state"),
           DOC(navground, sim, LidarStateEstimation, read_ranges));

  py::class_<OdometryStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<OdometryStateEstimation>>
      ose(m, "OdometryStateEstimation",
          DOC(navground, sim, OdometryStateEstimation));
  ose.def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, ng_float_t,
                   ng_float_t, bool, bool, const std::string &>(),
          py::arg("longitudinal_speed_bias") = 0,
          py::arg("longitudinal_speed_std_dev") = 0,
          py::arg("transversal_speed_bias") = 0,
          py::arg("transversal_speed_std_dev") = 0,
          py::arg("angular_speed_bias") = 0,
          py::arg("angular_speed_std_dev") = 0,
          py::arg("update_sensing_state") = true,
          py::arg("update_ego_state") = false, py::arg("name") = "",
          DOC(navground, sim, OdometryStateEstimation, OdometryStateEstimation))
      .def_property("longitudinal_speed_bias",
                    &OdometryStateEstimation::get_longitudinal_speed_bias,
                    &OdometryStateEstimation::set_longitudinal_speed_bias,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_longitudinal_speed_bias))
      .def_property("longitudinal_speed_std_dev",
                    &OdometryStateEstimation::get_longitudinal_speed_std_dev,
                    &OdometryStateEstimation::set_longitudinal_speed_std_dev,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_longitudinal_speed_std_dev))
      .def_property("transversal_speed_bias",
                    &OdometryStateEstimation::get_transversal_speed_bias,
                    &OdometryStateEstimation::set_transversal_speed_bias,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_transversal_speed_bias))
      .def_property("transversal_speed_std_dev",
                    &OdometryStateEstimation::get_transversal_speed_std_dev,
                    &OdometryStateEstimation::set_transversal_speed_std_dev,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_transversal_speed_std_dev))
      .def_property("angular_speed_bias",
                    &OdometryStateEstimation::get_angular_speed_bias,
                    &OdometryStateEstimation::set_angular_speed_bias,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_angular_speed_bias))
      .def_property("angular_speed_std_dev",
                    &OdometryStateEstimation::get_angular_speed_std_dev,
                    &OdometryStateEstimation::set_angular_speed_std_dev,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_angular_speed_std_dev))
      .def_property("update_sensing_state",
                    &OdometryStateEstimation::get_update_sensing_state,
                    &OdometryStateEstimation::set_update_sensing_state,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_update_sensing_state))
      .def_property("update_ego_state",
                    &OdometryStateEstimation::get_update_ego_state,
                    &OdometryStateEstimation::set_update_ego_state,
                    DOC(navground, sim, OdometryStateEstimation,
                        property_update_ego_state))
      .def_property("pose", &OdometryStateEstimation::get_pose, nullptr,
                    DOC(navground, sim, OdometryStateEstimation, property_pose))
      .def_property(
          "twist", &OdometryStateEstimation::get_twist, nullptr,
          DOC(navground, sim, OdometryStateEstimation, property_twist))
      .def_static(
          "read_pose_with_name", &OdometryStateEstimation::read_pose_with_name,
          py::arg("state"), py::arg("name"),
          DOC(navground, sim, OdometryStateEstimation, read_pose_with_name))
      .def("read_pose", &OdometryStateEstimation::read_pose, py::arg("state"),
           DOC(navground, sim, OdometryStateEstimation, read_pose));

  py::class_<MarkerStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<MarkerStateEstimation>>
      marker_se(m, "MarkerStateEstimation",
                DOC(navground, sim, MarkerStateEstimation));

  py::enum_<MarkerStateEstimation::ReferenceOrientation>(
      marker_se, "ReferenceOrientation",
      DOC(navground, core, MarkerStateEstimation, ReferenceOrientation))
      .value("agent", MarkerStateEstimation::ReferenceOrientation::agent,
             DOC(navground, core, MarkerStateEstimation, ReferenceOrientation,
                 rectangular))
      .value("world", MarkerStateEstimation::ReferenceOrientation::world,
             DOC(navground, core, MarkerStateEstimation, ReferenceOrientation,
                 circular))
      .value("target_direction",
             MarkerStateEstimation::ReferenceOrientation::target_direction,
             DOC(navground, core, MarkerStateEstimation, ReferenceOrientation,
                 none));

  marker_se
      .def(
          py::init<const Vector2 &, MarkerStateEstimation::ReferenceOrientation,
                   ng_float_t, ng_float_t, ng_float_t, ng_float_t, bool, bool,
                   const std::string &>(),
          py::arg("marker_position") = Vector2::Zero(),
          py::arg("reference_orientation") =
              MarkerStateEstimation::ReferenceOrientation::agent,
          py::arg("min_x") = -std::numeric_limits<ng_float_t>::infinity(),
          py::arg("min_y") = -std::numeric_limits<ng_float_t>::infinity(),
          py::arg("max_x") = std::numeric_limits<ng_float_t>::infinity(),
          py::arg("max_y") = std::numeric_limits<ng_float_t>::infinity(),
          py::arg("include_x") = true, py::arg("include_y") = true,
          py::arg("name") = "",
          DOC(navground, sim, MarkerStateEstimation, MarkerStateEstimation))
      .def_property(
          "marker_position", &MarkerStateEstimation::get_marker_position,
          &MarkerStateEstimation::set_marker_position,
          DOC(navground, sim, MarkerStateEstimation, property_marker_position))
      .def_property("reference_orientation",
                    &MarkerStateEstimation::get_reference_orientation,
                    &MarkerStateEstimation::set_reference_orientation,
                    DOC(navground, sim, MarkerStateEstimation,
                        property_reference_orientation))
      .def_property("min_x", &MarkerStateEstimation::get_min_x,
                    &MarkerStateEstimation::set_min_x,
                    DOC(navground, sim, MarkerStateEstimation, property_min_x))
      .def_property("min_y", &MarkerStateEstimation::get_min_y,
                    &MarkerStateEstimation::set_min_y,
                    DOC(navground, sim, MarkerStateEstimation, property_min_y))
      .def_property("max_x", &MarkerStateEstimation::get_max_x,
                    &MarkerStateEstimation::set_max_x,
                    DOC(navground, sim, MarkerStateEstimation, property_max_x))
      .def_property("max_y", &MarkerStateEstimation::get_max_y,
                    &MarkerStateEstimation::set_max_y,
                    DOC(navground, sim, MarkerStateEstimation, property_max_y))
      .def_property(
          "bounding_box", &MarkerStateEstimation::get_bounding_box,
          &MarkerStateEstimation::set_bounding_box,
          DOC(navground, sim, MarkerStateEstimation, property_bounding_box))
      .def_property(
          "include_x", &MarkerStateEstimation::get_include_x,
          &MarkerStateEstimation::set_include_x,
          DOC(navground, sim, MarkerStateEstimation, property_include_x))
      .def_property(
          "include_y", &MarkerStateEstimation::get_include_y,
          &MarkerStateEstimation::set_include_y,
          DOC(navground, sim, MarkerStateEstimation, property_include_y))
      .def("update_marker", &MarkerStateEstimation::update_marker,
           py::arg("agent"), py::arg("world"),
           DOC(navground, sim, MarkerStateEstimation, update_marker))
      .def_property("measured_marker_position",
                    &MarkerStateEstimation::get_measured_marker_position,
                    nullptr,
                    DOC(navground, sim, MarkerStateEstimation,
                        property_measured_marker_position))
      .def_static("read_marker_position_with_name",
                  &MarkerStateEstimation::read_marker_position_with_name,
                  py::arg("state"), py::arg("name"),
                  DOC(navground, sim, MarkerStateEstimation,
                      read_marker_position_with_name))
      .def("read_marker_position", &MarkerStateEstimation::read_marker_position,
           py::arg("state"),
           DOC(navground, sim, MarkerStateEstimation, read_marker_position));

  py::class_<LocalGridMapStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<LocalGridMapStateEstimation>>
      gmse(m, "LocalGridMapStateEstimation",
           DOC(navground, sim, LocalGridMapStateEstimation));

  py::enum_<LocalGridMapStateEstimation::FootprintType>(
      gmse, "FootprintType",
      DOC(navground, core, LocalGridMapStateEstimation, FootprintType))
      .value("rectangular",
             LocalGridMapStateEstimation::FootprintType::rectangular,
             DOC(navground, core, LocalGridMapStateEstimation, FootprintType,
                 rectangular))
      .value("circular", LocalGridMapStateEstimation::FootprintType::circular,
             DOC(navground, core, LocalGridMapStateEstimation, FootprintType,
                 circular))
      .value("none", LocalGridMapStateEstimation::FootprintType::none,
             DOC(navground, core, LocalGridMapStateEstimation, FootprintType,
                 none));

  gmse.def(py::init<const std::vector<std::shared_ptr<LidarStateEstimation>> &,
                    const std::vector<std::string> &,
                    const std::shared_ptr<OdometryStateEstimation> &,
                    const std::string &, unsigned, unsigned, ng_float_t, bool,
                    LocalGridMapStateEstimation::FootprintType,
                    const std::string &>(),
           py::arg("lidars") =
               std::vector<std::shared_ptr<LidarStateEstimation>>(),
           py::arg("external_lidars") = std::vector<std::string>(),
           py::arg("odometry") = nullptr,
           py::arg("external_odometry") = std::string(),
           py::arg("width") = LocalGridMapStateEstimation::default_width,
           py::arg("height") = LocalGridMapStateEstimation::default_height,
           py::arg("resolution") =
               LocalGridMapStateEstimation::default_resolution,
           py::arg("include_transformation") = false,
           py::arg("footprint") =
               LocalGridMapStateEstimation::FootprintType::rectangular,
           py::arg("name") = "",
           DOC(navground, sim, LocalGridMapStateEstimation,
               LocalGridMapStateEstimation))
      .def_property(
          "lidars", &LocalGridMapStateEstimation::get_lidars,
          &LocalGridMapStateEstimation::set_lidars,
          DOC(navground, sim, LocalGridMapStateEstimation, property_lidars))
      .def_property("external_lidars",
                    &LocalGridMapStateEstimation::get_external_lidars,
                    &LocalGridMapStateEstimation::set_external_lidars,
                    DOC(navground, sim, LocalGridMapStateEstimation,
                        property_external_lidars))
      .def_property(
          "odometry", &LocalGridMapStateEstimation::get_odometry,
          &LocalGridMapStateEstimation::set_odometry,
          DOC(navground, sim, LocalGridMapStateEstimation, property_odometry))
      .def_property("external_odometry",
                    &LocalGridMapStateEstimation::get_external_odometry,
                    &LocalGridMapStateEstimation::set_external_odometry,
                    DOC(navground, sim, LocalGridMapStateEstimation,
                        property_external_odometry))
      .def_property(
          "width", &LocalGridMapStateEstimation::get_width,
          &LocalGridMapStateEstimation::set_width,
          DOC(navground, sim, LocalGridMapStateEstimation, property_width))
      .def_property(
          "height", &LocalGridMapStateEstimation::get_height,
          &LocalGridMapStateEstimation::set_height,
          DOC(navground, sim, LocalGridMapStateEstimation, property_height))
      .def_property(
          "resolution", &LocalGridMapStateEstimation::get_resolution,
          &LocalGridMapStateEstimation::set_resolution,
          DOC(navground, sim, LocalGridMapStateEstimation, property_resolution))
      .def_property(
          "footprint", &LocalGridMapStateEstimation::get_footprint,
          &LocalGridMapStateEstimation::set_footprint,
          DOC(navground, sim, LocalGridMapStateEstimation, property_footprint))
      .def_property("include_transformation",
                    &LocalGridMapStateEstimation::get_include_transformation,
                    &LocalGridMapStateEstimation::set_include_transformation,
                    DOC(navground, sim, LocalGridMapStateEstimation,
                        property_include_transformation))
      .def_static("read_transform_with_name",
                  &LocalGridMapStateEstimation::read_transform_with_name,
                  py::arg("state"), py::arg("name"),
                  DOC(navground, sim, LocalGridMapStateEstimation,
                      read_transform_with_name))
      .def("read_transform", &LocalGridMapStateEstimation::read_transform,
           py::arg("state"),
           DOC(navground, sim, LocalGridMapStateEstimation, read_transform))
      .def_static("read_gridmap_with_name",
                  &LocalGridMapStateEstimation::read_gridmap_with_name,
                  py::arg("state"), py::arg("name"),
                  DOC(navground, sim, LocalGridMapStateEstimation,
                      read_gridmap_with_name))
      .def("read_gridmap", &LocalGridMapStateEstimation::read_gridmap,
           py::arg("state"),
           DOC(navground, sim, LocalGridMapStateEstimation, read_gridmap));

  py::class_<DiscsStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<DiscsStateEstimation>>
      dse(m, "DiscsStateEstimation", DOC(navground, sim, DiscsStateEstimation));
  dse.def(py::init<ng_float_t, unsigned, ng_float_t, ng_float_t, bool, bool,
                   unsigned, bool, bool, const std::string &>(),
          py::arg("range") = 1.0, py::arg("number") = 1,
          py::arg("max_radius") = 0, py::arg("max_speed") = 0,
          py::arg("include_valid") = true, py::arg("use_nearest_point") = true,
          py::arg("max_id") = 0, py::arg("include_x") = true,
          py::arg("include_y") = true, py::arg("name") = "",
          DOC(navground, sim, DiscsStateEstimation, DiscsStateEstimation))
      .def_property("range", &DiscsStateEstimation::get_range,
                    &DiscsStateEstimation::set_range,
                    DOC(navground, sim, DiscsStateEstimation, property_range))
      .def_property("number", &DiscsStateEstimation::get_number,
                    &DiscsStateEstimation::set_number,
                    DOC(navground, sim, DiscsStateEstimation, property_number))
      .def_property(
          "max_radius", &DiscsStateEstimation::get_max_radius,
          &DiscsStateEstimation::set_max_radius,
          DOC(navground, sim, DiscsStateEstimation, property_max_radius))
      .def_property(
          "max_speed", &DiscsStateEstimation::get_max_speed,
          &DiscsStateEstimation::set_max_speed,
          DOC(navground, sim, DiscsStateEstimation, property_max_speed))
      .def_property(
          "include_valid", &DiscsStateEstimation::get_include_valid,
          &DiscsStateEstimation::set_include_valid,
          DOC(navground, sim, DiscsStateEstimation, property_include_valid))
      .def_property("max_id", &DiscsStateEstimation::get_max_id,
                    &DiscsStateEstimation::set_max_id,
                    DOC(navground, sim, DiscsStateEstimation, property_max_id))
      .def_property(
          "use_nearest_point", &DiscsStateEstimation::get_use_nearest_point,
          &DiscsStateEstimation::set_use_nearest_point,
          DOC(navground, sim, DiscsStateEstimation, property_use_nearest_point))
      .def_property(
          "include_x", &DiscsStateEstimation::get_include_x,
          &DiscsStateEstimation::set_include_x,
          DOC(navground, sim, DiscsStateEstimation, property_include_x))
      .def_property(
          "include_y", &DiscsStateEstimation::get_include_y,
          &DiscsStateEstimation::set_include_y,
          DOC(navground, sim, DiscsStateEstimation, property_include_y));

  py::class_<BoundarySensor, Sensor, StateEstimation,
             std::shared_ptr<BoundarySensor>>
      boundary_sensor(m, "BoundarySensor", DOC(navground, sim, BoundarySensor));
  boundary_sensor
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, ng_float_t,
                    const std::string &>(),
           py::arg("range") = 1.0, py::arg("min_x") = BoundarySensor::low,
           py::arg("max_x") = BoundarySensor::high,
           py::arg("min_y") = BoundarySensor::low,
           py::arg("max_y") = BoundarySensor::high, py::arg("name") = "",
           DOC(navground, sim, BoundarySensor, BoundarySensor))
      .def_property("range", &BoundarySensor::get_range,
                    &BoundarySensor::set_range,
                    DOC(navground, sim, BoundarySensor, property_range))
      .def_property("min_x", &BoundarySensor::get_min_x,
                    &BoundarySensor::set_min_x,
                    DOC(navground, sim, BoundarySensor, property_min_x))
      .def_property("max_x", &BoundarySensor::get_max_x,
                    &BoundarySensor::set_max_x,
                    DOC(navground, sim, BoundarySensor, property_max_x))
      .def_property("min_y", &BoundarySensor::get_min_y,
                    &BoundarySensor::set_min_y,
                    DOC(navground, sim, BoundarySensor, property_min_y))
      .def_property("max_y", &BoundarySensor::get_max_y,
                    &BoundarySensor::set_max_y,
                    DOC(navground, sim, BoundarySensor, property_max_y));

  py::class_<SensorCombination, Sensor, StateEstimation,
             std::shared_ptr<SensorCombination>>
      cse(m, "SensorCombination", DOC(navground, sim, SensorCombination));
  cse.def(py::init<std::vector<std::shared_ptr<Sensor>>>(),
          py::arg("sensors") = std::vector<std::shared_ptr<Sensor>>(),
          DOC(navground, sim, SensorCombination, SensorCombination))
      .def_property("sensors", &SensorCombination::get_sensors,
                    &SensorCombination::set_sensors,
                    DOC(navground, sim, SensorCombination, property_sensors));

  task.def(py::init<>())
      // .def("update", &Task::update)
      .def(
          "prepare",
          [](PyTask *task, Agent *agent, World *world) {
            task->prepare(agent, world);
          },
          py::arg("agent"), py::arg("world"),
          DOC(navground, sim, Task, prepare))
      .def(
          "close", [](PyTask *task) { task->close(); },
          DOC(navground, sim, Task, close))
      .def(
          "update",
          [](PyTask *task, Agent *agent, World *world, ng_float_t time) {
            task->update(agent, world, time);
          },
          py::arg("agent"), py::arg("world"), py::arg("time"),
          DOC(navground, sim, Task, update))
      .def("done", &Task::done, DOC(navground, sim, Task, done))
      .def("get_log_size", &Task::get_log_size,
           DOC(navground, sim, Task, get_log_size))
      .def_property("log_size", &Task::get_log_size, nullptr,
                    DOC(navground, sim, Task, property, log_size))
      .def(
          "log_event",
          [](const PyTask &task, const std::vector<ng_float_t> &data) {
            task.log_event(data);
          },
          py::arg("log"), DOC(navground, sim, Task, log_event))
      .def("add_callback", &Task::add_callback, py::arg("callback"),
           DOC(navground, sim, Task, add_callback))
      .def_static("load", &YAML::load_string_py<PyTask>, py::arg("value"),
                  YAML::load_string_py_doc("task", "Task").c_str());

  py::class_<DirectionTask, Task, std::shared_ptr<DirectionTask>> direction(
      m, "DirectionTask", DOC(navground, sim, DirectionTask));
  direction
      .def(py::init<Vector2>(), py::arg("direction") = Vector2(1, 0),
           DOC(navground, sim, DirectionTask, DirectionTask))
      .def_property("direction", &DirectionTask::get_direction,
                    &DirectionTask::set_direction,
                    DOC(navground, sim, DirectionTask, property_direction));

  py::class_<WaypointsTask, Task, std::shared_ptr<WaypointsTask>> waypoints(
      m, "WaypointsTask", DOC(navground, sim, WaypointsTask));
  waypoints
      .def(py::init<const Waypoints &, bool, ng_float_t, bool,
                    const std::vector<ng_float_t> &,
                    const std::vector<ng_float_t> &, ng_float_t,
                    const std::vector<ng_float_t> &, ng_float_t,
                    const std::vector<ng_float_t> &>(),
           py::arg("waypoints") = Waypoints{},
           py::arg("loop") = WaypointsTask::default_loop,
           py::arg("tolerance") = WaypointsTask::default_tolerance,
           py::arg("random") = WaypointsTask::default_random,
           py::arg("tolerances") = std::vector<ng_float_t>(),
           py::arg("orientations") = std::vector<ng_float_t>(),
           py::arg("angular_tolerance") =
               WaypointsTask::default_angular_tolerance,
           py::arg("angular_tolerances") = std::vector<ng_float_t>(),
           py::arg("wait_time") = 0,
           py::arg("wait_times") = std::vector<ng_float_t>(),
           DOC(navground, sim, WaypointsTask, WaypointsTask))
      .def_property("log_size", &WaypointsTask::get_log_size, nullptr,
                    DOC(navground, sim, WaypointsTask, property, log_size))
      .def_property("waypoints", &WaypointsTask::get_waypoints,
                    &WaypointsTask::set_waypoints,
                    DOC(navground, sim, WaypointsTask, property_waypoints))
      .def_property("orientations", &WaypointsTask::get_orientations,
                    &WaypointsTask::set_orientations,
                    DOC(navground, sim, WaypointsTask, property_orientations))
      .def("get_effective_tolerance", &WaypointsTask::get_effective_tolerance,
           py::arg("index"),
           DOC(navground, sim, WaypointsTask, get_effective_tolerance))
      .def("get_effective_angular_tolerance",
           &WaypointsTask::get_effective_angular_tolerance, py::arg("index"),
           DOC(navground, sim, WaypointsTask, get_effective_angular_tolerance))
      .def_property("tolerance", &WaypointsTask::get_tolerance,
                    &WaypointsTask::set_tolerance,
                    DOC(navground, sim, WaypointsTask, property_tolerance))
      .def_property(
          "angular_tolerance", &WaypointsTask::get_angular_tolerance,
          &WaypointsTask::set_angular_tolerance,
          DOC(navground, sim, WaypointsTask, property_angular_tolerance))
      .def_property("tolerances", &WaypointsTask::get_tolerances,
                    &WaypointsTask::set_tolerances,
                    DOC(navground, sim, WaypointsTask, property_tolerances))
      .def_property(
          "angular_tolerances", &WaypointsTask::get_angular_tolerances,
          &WaypointsTask::set_angular_tolerances,
          DOC(navground, sim, WaypointsTask, property_angular_tolerances))
      .def_property("wait_time", &WaypointsTask::get_wait_time,
                    &WaypointsTask::set_wait_time,
                    DOC(navground, sim, WaypointsTask, property_wait_time))
      .def_property("wait_times", &WaypointsTask::get_wait_times,
                    &WaypointsTask::set_wait_times,
                    DOC(navground, sim, WaypointsTask, property_wait_times))
      .def("get_effective_wait_time", &WaypointsTask::get_effective_wait_time,
           py::arg("index"),
           DOC(navground, sim, WaypointsTask, get_effective_wait_time))
      .def_property("random", &WaypointsTask::get_random,
                    &WaypointsTask::set_random,
                    DOC(navground, sim, WaypointsTask, property_random))
      .def_property("loop", &WaypointsTask::get_loop, &WaypointsTask::set_loop,
                    DOC(navground, sim, WaypointsTask, property_loop));

  py::class_<GoToPoseTask, WaypointsTask, Task, std::shared_ptr<GoToPoseTask>>
      gotopose(m, "GoToPoseTask", DOC(navground, sim, GoToPoseTask));
  gotopose
      .def(py::init<const Vector2 &, ng_float_t, ng_float_t, ng_float_t>(),
           py::arg("point") = GoToPoseTask::default_point,
           py::arg("orientation") = 0,
           py::arg("tolerance") = GoToPoseTask::default_tolerance,
           py::arg("angular_tolerance") =
               GoToPoseTask::default_angular_tolerance,
           DOC(navground, sim, GoToPoseTask, GoToPoseTask))
      .def_property("point", &GoToPoseTask::get_point, &GoToPoseTask::set_point,
                    DOC(navground, sim, GoToPoseTask, property_point))
      .def_property("orientation", &GoToPoseTask::get_orientation,
                    &GoToPoseTask::set_orientation,
                    DOC(navground, sim, GoToPoseTask, property_orientation));

  py::class_<RecordNeighborsConfig>(m, "RecordNeighborsConfig",
                                    DOC(navground, sim, RecordNeighborsConfig))
      .def(py::init([](bool enabled, int number, bool relative) {
             return new RecordNeighborsConfig{enabled, number, relative};
           }),
           py::arg("enabled") = false, py::arg("number") = 0,
           py::arg("relative") = false)
      .def_readwrite("enabled", &RecordNeighborsConfig::enabled,
                     DOC(navground, sim, RecordNeighborsConfig, enabled))
      .def_readwrite("number", &RecordNeighborsConfig::number,
                     DOC(navground, sim, RecordNeighborsConfig, number))
      .def_readwrite("relative", &RecordNeighborsConfig::relative,
                     DOC(navground, sim, RecordNeighborsConfig, relative))
      .def("__repr__",
           [](const RecordNeighborsConfig &value) -> py::str {
             py::str r("RecordNeighborsConfig(enabled=");
             r += py::str(py::cast(value.enabled));
             r += py::str(", number=") + py::str(py::cast(value.number));
             r += py::str(", relative=") + py::str(py::cast(value.relative)) +
                  py::str(")");
             return r;
           })
      .def(py::pickle(
          [](const RecordNeighborsConfig &value) {
            return py::make_tuple(value.enabled, value.number, value.relative);
          },
          [](py::tuple v) { // __setstate__
            return RecordNeighborsConfig{py::cast<bool>(v[0]),
                                         py::cast<int>(v[1]),
                                         py::cast<bool>(v[2])};
          }))
      .def_static("schema", &YAML::schema_py<RecordNeighborsConfig>);

  py::class_<RecordSensingConfig>(m, "RecordSensingConfig",
                                  DOC(navground, sim, RecordSensingConfig))
      .def(
          py::init([](std::string name, std::shared_ptr<StateEstimation> sensor,
                      std::vector<unsigned> agent_indices) {
            return new RecordSensingConfig{
                name, std::dynamic_pointer_cast<Sensor>(sensor), agent_indices};
          }),
          py::arg("name") = "", py::arg("sensor") = nullptr,
          py::arg("agent_indices") = std::vector<unsigned>{})
      .def_readwrite("name", &RecordSensingConfig::name,
                     DOC(navground, sim, RecordSensingConfig, name))
      .def_readwrite("sensor", &RecordSensingConfig::sensor,
                     DOC(navground, sim, RecordSensingConfig, sensor))
      .def_readwrite("agent_indices", &RecordSensingConfig::agent_indices,
                     DOC(navground, sim, RecordSensingConfig, agent_indices))
      .def("__repr__",
           [](const RecordSensingConfig &value) -> py::str {
             py::str r("RecordSensingConfig(name='");
             r += py::str(py::cast(value.name));
             r += py::str("', sensor=") + py::str(py::cast(value.sensor));
             r += py::str(", agent_indices=") +
                  py::str(py::cast(value.agent_indices)) + py::str(")");
             return r;
           })
      .def(py::pickle(
          [](const RecordSensingConfig &value) {
            return py::make_tuple(value.name, value.sensor,
                                  value.agent_indices);
          },
          [](py::tuple v) { // __setstate__
            return RecordSensingConfig{py::cast<std::string>(v[0]),
                                       py::cast<std::shared_ptr<Sensor>>(v[1]),
                                       py::cast<std::vector<unsigned>>(v[2])};
          }))
      .def_static("schema", &YAML::schema_py<RecordSensingConfig>);

  py::class_<RunConfig>(m, "RunConfig", DOC(navground, sim, RunConfig))
      .def(py::init([](ng_float_t time_step = 0, unsigned steps = 0,
                       bool terminate_when_all_idle_or_stuck = false) {
             return new RunConfig{time_step, steps,
                                  terminate_when_all_idle_or_stuck};
           }),
           py::arg("time_step") = 0, py::arg("steps") = 0,
           py::arg("terminate_when_all_idle_or_stuck") = false)
      .def_readwrite("time_step", &RunConfig::time_step,
                     DOC(navground, sim, RunConfig, time_step))
      .def_readwrite("steps", &RunConfig::steps,
                     DOC(navground, sim, RunConfig, steps))
      .def_readwrite(
          "terminate_when_all_idle_or_stuck",
          &RunConfig::terminate_when_all_idle_or_stuck,
          DOC(navground, sim, RunConfig, terminate_when_all_idle_or_stuck))
      .def("__repr__",
           [](const RunConfig &value) -> py::str {
             py::str r("RunConfig(time_step=");
             r += py::str(py::cast(value.time_step));
             r += py::str(", steps=") + py::str(py::cast(value.steps));
             r += py::str(", terminate_when_all_idle_or_stuck=") +
                  py::str(py::cast(value.terminate_when_all_idle_or_stuck));
             r += py::str(")");
             return r;
           })
      .def(py::pickle(
          [](const RunConfig &value) {
            return py::make_tuple(value.time_step, value.steps,
                                  value.terminate_when_all_idle_or_stuck);
          },
          [](py::tuple v) { // __setstate__
            return RunConfig{py::cast<ng_float_t>(v[0]),
                             py::cast<unsigned>(v[1]), py::cast<bool>(v[2])};
          }));

  py::class_<RecordConfig>(m, "RecordConfig", DOC(navground, sim, RecordConfig))
      .def(py::init(
               [](bool time, bool pose, bool twist, bool cmd, bool actuated_cmd,
                  bool target, bool collisions, bool safety_violation,
                  bool task_events, bool deadlocks, bool efficacy, bool world,
                  RecordNeighborsConfig neighbors, bool use_agent_uid_as_key,
                  std::vector<RecordSensingConfig> sensing) {
                 return new RecordConfig{time,         pose,
                                         twist,        cmd,
                                         actuated_cmd, target,
                                         collisions,   safety_violation,
                                         task_events,  deadlocks,
                                         efficacy,     world,
                                         neighbors,    use_agent_uid_as_key,
                                         sensing};
               }),
           py::arg("time") = false, py::arg("pose") = false,
           py::arg("twist") = false, py::arg("cmd") = false,
           py::arg("actuated_cmd") = false, py::arg("target") = false,
           py::arg("collisions") = false, py::arg("safety_violation") = false,
           py::arg("task_events") = false, py::arg("deadlocks") = false,
           py::arg("efficacy") = false, py::arg("world") = false,
           py::arg("neighbors") = RecordNeighborsConfig{false, 0, false},
           py::arg("use_agent_uid_as_key") = true,
           py::arg("sensing") = std::vector<RecordSensingConfig>{})
      // DOC(navground, sim, RecordConfig, RecordConfig)
      .def_readwrite("time", &RecordConfig::time,
                     DOC(navground, sim, RecordConfig, time))
      .def_readwrite("pose", &RecordConfig::pose,
                     DOC(navground, sim, RecordConfig, pose))
      .def_readwrite("twist", &RecordConfig::twist,
                     DOC(navground, sim, RecordConfig, twist))
      .def_readwrite("cmd", &RecordConfig::cmd,
                     DOC(navground, sim, RecordConfig, cmd))
      .def_readwrite("actuated_cmd", &RecordConfig::actuated_cmd,
                     DOC(navground, sim, RecordConfig, actuated_cmd))
      .def_readwrite("target", &RecordConfig::target,
                     DOC(navground, sim, RecordConfig, target))
      .def_readwrite("safety_violation", &RecordConfig::safety_violation,
                     DOC(navground, sim, RecordConfig, safety_violation))
      .def_readwrite("collisions", &RecordConfig::collisions,
                     DOC(navground, sim, RecordConfig, collisions))
      .def_readwrite("task_events", &RecordConfig::task_events,
                     DOC(navground, sim, RecordConfig, task_events))
      .def_readwrite("deadlocks", &RecordConfig::deadlocks,
                     DOC(navground, sim, RecordConfig, deadlocks))
      .def_readwrite("efficacy", &RecordConfig::efficacy,
                     DOC(navground, sim, RecordConfig, efficacy))
      .def_readwrite("world", &RecordConfig::world,
                     DOC(navground, sim, RecordConfig, world))
      .def_readwrite("neighbors", &RecordConfig::neighbors,
                     DOC(navground, sim, RecordConfig, neighbors))
      .def_readwrite("use_agent_uid_as_key",
                     &RecordConfig::use_agent_uid_as_key,
                     DOC(navground, sim, RecordConfig, use_agent_uid_as_key))
      .def_readwrite("sensing", &RecordConfig::sensing,
                     DOC(navground, sim, RecordConfig, sensing))
      .def_static("all", &RecordConfig::all, py::arg("value"),
                  DOC(navground, sim, RecordConfig, all))
      .def("set_all", &RecordConfig::set_all, py::arg("value"),
           DOC(navground, sim, RecordConfig, set_all))
      .def("record_sensor", &RecordConfig::record_sensor, py::arg("name"),
           py::arg("sensor"), py::arg("agent_indices"),
           DOC(navground, sim, RecordConfig, record_sensor))
      .def("__repr__",
           [](const RecordConfig &value) -> py::str {
             py::str r("RecordConfig(time=");
             r += py::str(py::cast(value.time));
             r += py::str(", pose=") + py::str(py::cast(value.pose));
             r += py::str(", twist=") + py::str(py::cast(value.twist));
             r += py::str(", cmd=") + py::str(py::cast(value.cmd));
             r += py::str(", actuated_cmd=") +
                  py::str(py::cast(value.actuated_cmd));
             r += py::str(", target=") + py::str(py::cast(value.target));
             r +=
                 py::str(", collisions=") + py::str(py::cast(value.collisions));
             r += py::str(", safety_violation=") +
                  py::str(py::cast(value.safety_violation));
             r += py::str(", task_events=") +
                  py::str(py::cast(value.task_events));
             r += py::str(", deadlocks=") + py::str(py::cast(value.deadlocks));
             r += py::str(", efficacy=") + py::str(py::cast(value.efficacy));
             r += py::str(", world=") + py::str(py::cast(value.world));
             r += py::str(", neighbors=") + py::str(py::cast(value.neighbors));
             r += py::str(", sensing=") + py::str(py::cast(value.sensing));
             r += py::str(", use_agent_uid_as_key=") +
                  py::str(py::cast(value.use_agent_uid_as_key)) + py::str(")");
             return r;
           })
      .def(py::pickle(
          [](const RecordConfig &value) {
            return py::make_tuple(value.time, value.pose, value.twist,
                                  value.cmd, value.actuated_cmd, value.target,
                                  value.collisions, value.safety_violation,
                                  value.task_events, value.deadlocks,
                                  value.efficacy, value.world, value.neighbors,
                                  value.use_agent_uid_as_key, value.sensing);
          },
          [](py::tuple v) { // __setstate__
            return RecordConfig{
                py::cast<bool>(v[0]),
                py::cast<bool>(v[1]),
                py::cast<bool>(v[2]),
                py::cast<bool>(v[3]),
                py::cast<bool>(v[4]),
                py::cast<bool>(v[5]),
                py::cast<bool>(v[6]),
                py::cast<bool>(v[7]),
                py::cast<bool>(v[8]),
                py::cast<bool>(v[9]),
                py::cast<bool>(v[10]),
                py::cast<bool>(v[11]),
                py::cast<RecordNeighborsConfig>(v[12]),
                py::cast<bool>(v[13]),
                py::cast<std::vector<RecordSensingConfig>>(v[14])};
          }));

  py::class_<Dataset, std::shared_ptr<Dataset>>(
      m, "Dataset", py::buffer_protocol(), DOC(navground, sim, Dataset))
      .def(py::init([](const py::object &data, const py::object &dtype,
                       const std::vector<size_t> &item_shape) {
             Dataset ds;
             if (!data.is_none()) {
               auto arr = py::array::ensure(data);
               set_dataset_data_py(ds, arr);
             } else {
               ds.set_item_shape(item_shape);
               if (!dtype.is_none()) {
                 set_dataset_type_py(ds, dtype);
               }
             }
             return ds;
           }),
           py::arg("data") = py::none(), py::arg("dtype") = py::none(),
           py::arg("item_shape") = std::vector<size_t>(),
           R"doc(
Instantiate a dataset with an object convertible to a numpy array 
of one of the types of :cpp:type:`navground::sim::Dataset::Scalar`.

:param data: Copies shape, data and dtype from this numpy array (if specified)
:type data: :py:data:`numpy.typing.ArrayLike` | None
:param dtype: The type of data to store (if specified and if ``data`` is None)
:type dtype: :py:data:`numpy.typing.DTypeLike` | None.
:param item_shape: Set the shape of all axis except the first (only if ``data`` is None ).
                   Set to an empty list to instantiate a flat dataset.
:type item_shape:  list[int]

)doc")
#if 0
      .def(
          py::init([](py::object dtype, const std::vector<size_t> &item_shape) {
            Dataset ds(item_shape);
            set_dataset_type_py(ds, dtype);
            return ds;
          }),
          ,
          R"doc(
Instantiate a dataset with a type and the shape of an item.

:param dtype: The type of data to store
:type dtype: Any object that is convertible to a :py:data:`numpy.typing.DTypeLike`.

:param item_shape: The shape of all axis except the first.
                   Leave empty to instantiate a flat dataset.
:type item_shape:  list[int]

)doc")
#endif
      .def("__repr__",
           [](const Dataset &ds) -> py::str {
             py::str r("<Dataset: shape ");
             r += py::str(py::tuple(py::cast(ds.get_shape())));
             r += py::str(", dtype ") + py::str(get_dataset_type_py(ds));
             r += py::str(">");
             return r;
           })
      .def_buffer([](const Dataset &ds) { return as_array(ds).request(); })
      .def(
          "append",
          [](Dataset &ds, const py::object b, bool reset) {
            py::array arr = py::array::ensure(b);
            set_dataset_data_py(ds, arr, !reset);
          },
          py::arg("values"), py::arg("reset") = false, R"doc(
Append items from (objects convertible to) numpy arrays of 
one of the types of :cpp:type:`navground::sim::Dataset::Scalar`.

:param values: Append data and dtype from this numpy array
:type values: :py:data:`numpy.typing.ArrayLike`
:param reset: Whether to replace the data instead of appending. 
:type reset: bool

)doc")
#if 0
      .def(
          "append",
          [](Dataset &ds, const Dataset::Data &values) { ds.append(values); },
          py::arg("values"),
          // DOC(navground, sim, Dataset, append)
          R"doc(
Add items.

The items will be implicitly converted to the current data type, see :py:meth:`dtype`.

:param values: The values to add
:type values: list[int] | list[float]
)doc")
#endif
#if 0
      .def(
          "push",
          [](Dataset &ds, const Dataset::Scalar &value) { ds.push(value); },
          py::arg("value"),
          // DOC(navground, sim, Dataset, push)
          R"doc(
Add an item.

The item will be implicitly converted to the current data type, see :py:meth:`dtype`.

:param value: The values to add
:type value: int | float
)doc")
#endif
      .def(
          "push",
          [](Dataset &ds, const py::object &value) {
            dataset_push_py(ds, value);
          },
          py::arg("value"),
          // DOC(navground, sim, Dataset, push)
          R"doc(
Add an item of one of the types of :cpp:type:`navground::sim::Dataset::Scalar`.

The item will be converted to the current data type, see :py:meth:`dtype`.

:param value: The value to add
:type value: :py:class:`numbers.Real`
)doc")
      .def("reset", &Dataset::reset, DOC(navground, sim, Dataset, reset))
      .def_property("size", &Dataset::size, nullptr,
                    DOC(navground, sim, Dataset, property, size))
      .def_property("is_valid", &Dataset::is_valid, nullptr,
                    DOC(navground, sim, Dataset, property, is_valid))
      .def_property("item_shape", &Dataset::get_item_shape,
                    &Dataset::set_item_shape,
                    DOC(navground, sim, Dataset, property, item_shape))
      .def_property("shape", &Dataset::get_shape, nullptr,
                    DOC(navground, sim, Dataset, property, shape))
      .def_property("dtype", &get_dataset_type_py, &set_dataset_type_py,
                    R"doc(
The type of the dataset.

Can be set to any object that is convertible to :py:class:`numpy.dtype`.

)doc")
      .def(py::pickle(
          // HACK(Jerome):
          // For some reason I need to pass a dtype, because the arr.dtype()
          // returns a different type.
          [](const Dataset &ds) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(as_array(ds),
                                  get_dataset_type_py(ds).attr("str"));
          },
          [](py::tuple t) { // __setstate__
            if (t.size() != 2)
              throw std::runtime_error("Invalid state!");
            Dataset ds;
            auto arr = py::cast<py::array>(t[0]);
            auto dt = py::dtype(py::cast<std::string>(t[1]));
            set_dataset_data_py(ds, arr, false, dt);
            return ds;
          }))
      .def("write_buffer", &Dataset::write_buffer, py::arg("buffer"),
           py::arg("index"), DOC(navground, sim, Dataset, write_buffer))
      .def("get_buffer", &Dataset::get_buffer, py::arg("index"),
           DOC(navground, sim, Dataset, get_buffer))
      .def_property("number_of_items", &Dataset::get_number_of_items, nullptr,
                    DOC(navground, sim, Dataset, property_number_of_items))
      .def("__len__",
           [](Dataset &ds) -> size_t {
             const auto shape = ds.get_shape();
             if (shape.size()) {
               return shape[0];
             }
             return 0;
           })
      .def("__iter__",
           [](Dataset &ds) { return as_array(ds).attr("__iter__")(); });

  py::class_<ExperimentalRun, std::shared_ptr<ExperimentalRun>>
      experimental_run(m, "ExperimentalRun", py::dynamic_attr(),
                       DOC(navground, sim, ExperimentalRun));

  py::class_<Probe, PyProbe, std::shared_ptr<Probe>>(m, "Probe",
                                                     DOC(navground, sim, Probe))
      .def(py::init<>(), DOC(navground, sim, Probe, Probe))
      .def("prepare", &Probe::prepare, py::arg("run"),
           DOC(navground, sim, Probe, prepare))
      .def("update", &Probe::update, py::arg("run"),
           DOC(navground, sim, Probe, update))
      .def("finalize", &Probe::finalize, py::arg("run"),
           DOC(navground, sim, Probe, finalize));

  py::class_<RecordProbe, Probe, PyRecordProbe, std::shared_ptr<RecordProbe>>(
      m, "RecordProbe", DOC(navground, sim, RecordProbe))
      .def(py::init<std::shared_ptr<Dataset>>(), py::arg("record") = nullptr,
           DOC(navground, sim, RecordProbe, RecordProbe))
      .def("get_shape", &RecordProbe::get_shape, py::arg("world"),
           DOC(navground, sim, RecordProbe, get_shape))
      .def_property("data", &RecordProbe::get_data, nullptr,
                    DOC(navground, sim, RecordProbe, property_data));

  py::class_<GroupRecordProbe, Probe, PyGroupRecordProbe,
             std::shared_ptr<GroupRecordProbe>>(
      m, "GroupRecordProbe", DOC(navground, sim, GroupRecordProbe))
      .def(py::init<std::optional<GroupRecordProbe::Factory>>(),
           py::arg("factory") = py::none(),
           DOC(navground, sim, GroupRecordProbe, GroupRecordProbe))
      .def("get_shapes", &GroupRecordProbe::get_shapes, py::arg("world"),
           py::arg("use_agent_uid_as_key"),
           DOC(navground, sim, GroupRecordProbe, get_shapes))
      .def("get_data", &GroupRecordProbe::get_data, py::arg("key"),
           DOC(navground, sim, GroupRecordProbe, get_data));

  py::class_<SensingProbe, Probe, std::shared_ptr<SensingProbe>>(
      m, "SensingProbe", DOC(navground, sim, SensingProbe))
      .def(py::init<std::string, std::shared_ptr<Sensor>,
                    std::vector<unsigned>>(),
           py::arg("name") = "sensing", py::arg("sensor") = nullptr,
           py::arg("agent_indices") = std::vector<unsigned>{},
           DOC(navground, sim, SensingProbe, SensingProbe))
      .def("get_data", &SensingProbe::get_data,
           //            R"doc(
           // The stored sensor readings, indexed by UID or index

           // :rtype: typing.Dict[str, navground.sim.Dataset]

           // )doc"
           DOC(navground, sim, SensingProbe, get_data));

  experimental_run
      .def(py::init<std::shared_ptr<World>, ng_float_t, int, bool,
                    const RecordConfig &, int>(),
           py::arg("world"), py::arg("time_step") = 0.1,
           py::arg("steps") = 1000,
           py::arg("terminate_when_all_idle_or_stuck") = true,
           py::arg("record_config") = RecordConfig(), py::arg("seed") = 0,
           DOC(navground, sim, ExperimentalRun, ExperimentalRun))
      .def("index_of_agent", &ExperimentalRun::index_of_agent, py::arg("agent"),
           DOC(navground, sim, ExperimentalRun, index_of_agent))
      .def(
          "add_record",
          [](ExperimentalRun &run, const std::string &key,
             const py::object &data, const py::object &dtype,
             const std::vector<size_t> &item_shape) {
            auto ds = run.add_record(key);
            if (!data.is_none()) {
              auto arr = py::array::ensure(data);
              set_dataset_data_py(*ds, arr);
            } else {
              ds->set_item_shape(item_shape);
              if (!dtype.is_none()) {
                set_dataset_type_py(*ds, dtype);
              }
            }
            return ds;
          },
          py::arg("key"), py::arg("data") = py::none(),
          py::arg("dtype") = py::none(),
          py::arg("item_shape") = std::vector<size_t>(), R"doc(
Adds a record.

:param key: the record key
:type key: str

:param data: Copies shape, data and dtype from this numpy array (if specified)
:type data: :py:data:`numpy.typing.ArrayLike` | None
:param dtype: The type of data to store (if specified and if ``data`` is None)
:type dtype: :py:data:`numpy.typing.DTypeLike` | None.
:param item_shape: Set the shape of all axis except the first (only if ``data`` is None ).
                   Set to an empty list to instantiate a flat dataset.
:type item_shape:  list[int]

)doc")
#if 0
      .def(
          "add_record",
          [](ExperimentalRun &run, const std::string &key, py::array data) {
            auto ds = run.add_record(key);
            set_dataset_data_py(*ds, data);
            return ds;
          },
          py::arg("key"), py::arg("data"), R"doc(
Adds a record.

:param key: the record key
:type key: str

:param data: Copies shape, data and dtype from this numpy array
:type data: :py:data:`numpy.typing.ArrayLike`

)doc")
      .def(
          "add_record",
          [](ExperimentalRun &run, const std::string &key, py::object dtype,
             const std::vector<size_t> &item_shape) {
            auto ds = run.add_record(key);
            ds->set_item_shape(item_shape);
            set_dataset_type_py(*ds, dtype);
            return ds;
          },
          py::arg("key"), py::arg("dtype"),
          py::arg("item_shape") = std::vector<int>(), R"doc(
Adds a record.

:param key: The record key
:type key: str

:param dtype: The type of data to store
:type dtype: :py:class:`numpy.typing.DTypeLike`.

:param item_shape: The shape of all axis except the first.
                   Leave empty to instantiate a flat dataset.
:type item_shape:  list[int]

)doc")
#endif
      .def("add_probe", &ExperimentalRun::add_probe, py::keep_alive<1, 2>(),
           py::arg("probe"), DOC(navground, sim, ExperimentalRun, add_probe))
      .def(
          "add_record_probe",
          [](py::object &run, const std::string &name,
             const py::object &probe) {
            ExperimentalRun &_run = py::cast<ExperimentalRun &>(run);
            auto obj = add_record_probe_py(name, probe, _run);
            add_py_item(run, obj, "probes");
            return obj;
          },
          py::arg("key"), py::arg("probe"), R"doc(
Adds a record probe.

:param key: the key of the record to be created
:type key: str

:param probe: the probe factory like a RecordProbe class.
:type key: typing.Callable[[navground.sim.Dataset], sim.RecordProbe]

)doc")
      .def(
          "add_group_record_probe",
          [](py::object &run, const std::string &name,
             const py::object &probe) {
            ExperimentalRun &_run = py::cast<ExperimentalRun &>(run);
            auto obj = add_group_record_probe_py(name, probe, _run);
            add_py_item(run, obj, "probes");
            return obj;
          },
          py::arg("key"), py::arg("probe"), R"doc(
Adds a group record probe.

:param key: the key of the group to be created
:type key: str

:param probe: the probe factory like a GroupRecordProbe class.
:type key: typing.Callable[[], sim.GroupRecordProbe]

)doc")
      .def_property(
          "record_names",
          [](const ExperimentalRun &run) { return run.get_record_names(); },
          nullptr, DOC(navground, sim, ExperimentalRun, property_record_names))
      .def_property(
          "records",
          [](const ExperimentalRun &run) { return run.get_records(); }, nullptr,
          DOC(navground, sim, ExperimentalRun, property_records))
      .def("get_record_names", &ExperimentalRun::get_record_names,
           py::arg("group") = "",
           DOC(navground, sim, ExperimentalRun, get_record_names))
      .def("get_records", &ExperimentalRun::get_records, py::arg("group") = "",
           DOC(navground, sim, ExperimentalRun, get_records))
      .def("get_record", &ExperimentalRun::get_record, py::arg("key") = "",
           DOC(navground, sim, ExperimentalRun, get_record))
#if 0
      .def(
          "get_records",
          [](const ExperimentalRun *run, const std::string &key) {
            return as_dict_array(run->get_records(key));
          },
          py::arg("group") = "", R"doc(
      Gets the records.
      
      :param group: if specified, limits to records in a given group.
      :type group: str
      :return:  read-only recorded data arrays indexed by their key
      (relative to the group if specified).
      :rtype: typing.Dict[str, np.ndarray]
      )doc")
      .def(
          "get_record",
          [](const ExperimentalRun *run, const std::string &key) -> py::object {
            auto record = run->get_record(key);
            if (record) return as_array(*record);
            return py::none();
          },
          py::arg("key"), R"doc(
Gets recorded data.

:param key: the name of the record
:type key: str
:return: read-only recorded data array or None if no data has been recorded for the given key
:rtype: typing.Optional[np.ndarray]
)doc")
#endif
      .def_property(
          "times",
          [](const ExperimentalRun *run) {
            auto record = run->get_times();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded simulation times as a numpy array of shape
``(simulation steps)`` and dtype ``float``::

  [t_0, t_1, ...]
  
The array is empty if times have not been recorded in the run.
)doc")
      .def_property(
          "poses",
          [](const ExperimentalRun *run) {
            auto record = run->get_poses();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded poses of the agents as a numpy array of shape 
``(simulation steps, number of agents, 3)`` and dtype ``float``::

  [[[x_0, y_0, theta_0], 
    [x_1, y_1, theta_1], 
    ...], 
   ...]
  
The array is empty if poses have not been recorded in the run.
)doc")
      .def_property(
          "twists",
          [](const ExperimentalRun *run) {
            auto record = run->get_twists();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded twists of the agents as a numpy array of shape 
``(simulation steps, number of agents, 3)`` and dtype ``float``::

  [[[vx_0, vy_0, omega_0], 
    [vx_1, vy_1, omega_1], 
    ...], 
   ...]
  
The array is empty if twist have not been recorded in the run.
)doc")
      .def_property(
          "targets",
          [](const ExperimentalRun *run) {
            auto record = run->get_targets();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded targets of the agents as a numpy array of shape 
``(simulation steps, number of agents, 16)`` and dtype ``float``::

  [[[position?, position[0], position[1], orientation?, orientation, 
     speed?, speed, direction?, direction[0], direction[1],
     angular_speed?, angular_speed, angular_direction?, angular_direction,
     position_tol, orientation_tol], 
     ...] 
   ...]

where <x>? is 1.0 if the field is defined with value  <x>,
else <x>? is 0.0 and value <x> is set to zero.
  
The array is empty if targets have not been recorded in the run.
)doc")
      .def_property(
          "commands",
          [](const ExperimentalRun *run) {
            auto record = run->get_cmds();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded commands of the agents as a numpy array of shape 
``(simulation steps, number of agents, 3)`` and dtype ``float``::

  [[[vx_0, vy_0, omega_0], 
    [vx_1, vy_1, omega_1], 
    ...], 
   ...]
  
The array is empty if commands have not been recorded in the run.
)doc")
      .def_property(
          "actuated_commands",
          [](const ExperimentalRun *run) {
            auto record = run->get_actuated_cmds();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded actuated commands of the agents as a numpy array of shape 
``(simulation steps, number of agents, 3)`` and dtype ``float``::

  [[[vx_0, vy_0, omega_0], 
    [vx_1, vy_1, omega_1], 
    ...], 
   ...]
  
The array is empty if actuated commands have not been recorded in the run.
)doc")
      .def_property(
          "safety_violations",
          [](const ExperimentalRun *run) {
            auto record = run->get_safety_violations();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded amounts of safety violation as a numpy array of shape 
``(simulation steps, number of agents)`` and dtype ``float``::

  [[violation_0, violation_1, ...],
   ...]

where a value of 0 represents no violations.

The array is empty if safety violations have not been recorded in the run.
)doc")
      .def_property(
          "collisions",
          [](const ExperimentalRun *run) {
            auto record = run->get_collisions();
            if (record)
              return as_array(*record);
            return make_empty_array<unsigned>();
          },
          nullptr, R"doc(
The recorded collisions between pairs of entities as
as a numpy array of shape ``(number of collisions, 3)``
and dtype ``np.uint32``::

  [[time_step, uid_0, uid_1], 
   ...]

The array is empty if collisions have not been recorded in the run.

)doc")
      .def(
          "get_collision_events",
          [](const ExperimentalRun *run, unsigned min_interval) {
            auto record = run->get_collision_events(min_interval);
            return record;
          },
          py::arg("min_interval") = 1, R"doc(
The recorded collision events between pairs of entities as
as a Dataset of shape ``(number of collisions, 3)``
and dtype ``np.uint32``::

  [[time_step, uid_0, uid_1], 
   ...]

Only collision among the same pair separated by at least 
``minimal_interval`` are returned.
The array is empty if collisions have not been recorded in the run.

)doc")
      .def(
          "get_steps_to_collision",
          [](const ExperimentalRun *run, unsigned min_interval) {
            auto record = run->get_steps_to_collision(min_interval);
            return record;
          },
          py::arg("min_interval") = 1, R"doc(
The number of steps to the next recorded collision for each agent as
as a Dataset of shape ``(number of steps, number of agents)``
and dtype ``np.uint32``::

  [[agent_0_steps, agent_1_steps, ...], 
   ...]

)doc")
      .def_property("sensing", &ExperimentalRun::get_sensing, nullptr,
                    DOC(navground, sim, ExperimentalRun, property, sensing))
      .def("get_sensing_for", &ExperimentalRun::get_sensing_for, py::arg("id"),
           DOC(navground, sim, ExperimentalRun, get_sensing_for))
      .def_property(
          "task_events",
          [](const ExperimentalRun *run) {
            std::map<unsigned, py::object> records;
            for (auto &[k, v] : run->get_task_events()) {
              records[k] = as_array(*v);
            }
            return records;
          },
          nullptr,
          R"doc(
The recorded events logged by the tasks all agents as a dictionart of numpy array of shape 
``(number events, size of event log)`` and dtype ``float``::

  {<agent_uid>: [[data_0, ...], ...]}

The array are empty if the agent's task has not been recorded in the run.

:return: The events logged by the tasks all agents
)doc")
      .def(
          "get_task_events",
          [](const ExperimentalRun *run, const Agent *agent) {
            // CHANGED(Jerome): check ``use_agent_uid_as_key``
            unsigned index = 0;
            if (run->get_record_config().use_agent_uid_as_key) {
              index = agent->uid;
            } else {
              auto i = run->index_of_agent(agent);
              if (i) {
                index = *i;
              }
            }
            auto record = run->get_task_events_for(index);
            if (!record) {
              return make_empty_array<ng_float_t>();
            }
            return as_array(*record);
          },
          py::arg("agent"),
          R"doc(
The recorded events logged by the task of an agent as a numpy array of shape 
``(number events, size of event log)`` and dtype ``float``::

  [[data_0, ...], 
   ...]

The array is empty if the agent's task has not been recorded in the run.

:param agent: The agent

:return: The events logged by the agent task
)doc")
      .def(
          "get_task_events",
          [](const ExperimentalRun *run, unsigned uid) -> py::object {
            auto record = run->get_task_events_for(uid);
            if (!record) {
              return make_empty_array<ng_float_t>();
            }
            return as_array(*record);
          },
          py::arg("uid"),
          R"doc(
The recorded events logged by the task of an agent as a numpy array of shape 
``(number events, size of event log)`` and dtype ``float``::

  [[data_0, ...], 
   ...]

The array is empty if the agent's task has not been recorded in the run.

:param uid: The uid of the agent

:return: The events logged by the agent task
)doc")
      .def_property(
          "deadlocks",
          [](const ExperimentalRun *run) {
            auto record = run->get_deadlocks();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The time since agents are deadlocked as a numpy array of shape 
``(number of agents, )`` and dtype ``float``::

  [time_0, time_1, ...]

If ``time_i`` is negative, the i-th agent is not stuck at the end of the recording.
Else, it has been stuck since ``time_i``.

The array is empty if deadlocks have not been recorded in the run.

)doc")
      .def_property(
          "efficacy",
          [](const ExperimentalRun *run) {
            auto record = run->get_efficacy();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded agents' efficacy as a numpy array of shape 
``(simulation steps, number of agents)`` and dtype ``float``::

  [[efficacy_0, efficacy_1, ...],
   ...]

The array is empty if efficacy has not been recorded in the run.
)doc")
      .def_property(
          "neighbors",
          [](const ExperimentalRun *run) {
            auto record = run->get_neighbors();
            if (record)
              return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded agents' neighbors as a numpy array of shape 
``(simulation steps, number of agents, number of neighbors, 5)`` and dtype ``float``::

  [[[radius, x, y, vx, vy]],
   ...]

The array is empty if efficacy has not been recorded in the run.
)doc")
      .def_property(
          "final_sim_time", &ExperimentalRun::get_final_sim_time, nullptr,
          DOC(navground, sim, ExperimentalRun, property_final_sim_time))
      .def_property("has_finished", &ExperimentalRun::has_finished, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_has_finished))
      .def_property(
          "recorded_steps", &ExperimentalRun::get_recorded_steps, nullptr,
          DOC(navground, sim, ExperimentalRun, property_recorded_steps))
      // .def_property(
      //     "number_of_agents", &ExperimentalRun::get_number_of_agents,
      //     nullptr, DOC(navground, sim, ExperimentalRun,
      //     property_number_of_agents))
      .def_property("seed", &ExperimentalRun::get_seed, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_seed))
      .def_property("world", &ExperimentalRun::get_world, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_world))
      // .def("update", &ExperimentalRun::update,
      //      DOC(navground, sim, ExperimentalRun, update))
      .def("run", &ExperimentalRun::run,
           DOC(navground, sim, ExperimentalRun, run))
      .def("start", &ExperimentalRun::start,
           DOC(navground, sim, ExperimentalRun, start))
      .def("update", &ExperimentalRun::update,
           DOC(navground, sim, ExperimentalRun, update))
      .def("stop", &ExperimentalRun::stop,
           DOC(navground, sim, ExperimentalRun, stop))
      .def_property("duration", &ExperimentalRun::get_duration, nullptr,
                    DOC(navground, sim, ExperimentalRun, property, duration))
      .def_property("record_config", &ExperimentalRun::get_record_config,
                    nullptr,
                    DOC(navground, sim, ExperimentalRun, record_config))
      // .def_readonly("run_config", &ExperimentalRun::run_config,
      //               DOC(navground, sim, ExperimentalRun, run_config))
      .def_property("time_step", &ExperimentalRun::get_time_step, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_time_step))
      .def_property("maximal_steps", &ExperimentalRun::get_maximal_steps,
                    nullptr,
                    DOC(navground, sim, ExperimentalRun, property_steps))
      .def_property("terminate_when_all_idle_or_stuck",
                    &ExperimentalRun::get_terminate_when_all_idle_or_stuck,
                    nullptr,
                    DOC(navground, sim, ExperimentalRun,
                        property_terminate_when_all_idle_or_stuck))
      .def_property("bounding_box", &ExperimentalRun::get_bounding_box, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_bounding_box))
      .def("get_collisions_at_step", &ExperimentalRun::get_collisions_at_step,
           py::arg("step"),
           DOC(navground, sim, ExperimentalRun, get_collisions_at_step))
      .def_static("target_from_data", &ExperimentalRun::target_from_data,
                  py::arg("data"),
                  DOC(navground, sim, ExperimentalRun, target_from_data))
      .def_static("data_from_target", &ExperimentalRun::data_from_target,
                  py::arg("target"),
                  DOC(navground, sim, ExperimentalRun, data_from_target))
      .def("go_to_step", &ExperimentalRun::go_to_step, py::arg("step"),
           py::arg("ignore_collisions") = false,
           py::arg("ignore_twists") = false, py::arg("ignore_cmds") = false,
           py::arg("ignore_targets") = false, py::arg("ignore_sensing") = false,
           DOC(navground, sim, ExperimentalRun, go_to_step))
      .def("reset", &ExperimentalRun::reset,
           DOC(navground, sim, ExperimentalRun, reset))
      .def(py::pickle(
          [](const ExperimentalRun &run) {
            // __getstate__
            return py::make_tuple(
                // run.get_world(),
                run.get_time_step(), run.get_maximal_steps(),
                run.get_terminate_when_all_idle_or_stuck(),
                run.get_record_config(), run.get_seed(),
                static_cast<int>(run.get_state()), run.get_recorded_steps(),
                run.get_begin(), run.get_end(), run.get_world_yaml(),
                run.get_records());
          },
          [](py::tuple t) { // __setstate__
            if (t.size() != 11)
              throw std::runtime_error("Invalid state!");
            // auto world = py::cast<std::shared_ptr<World>>(t[0]);
            auto time_step = py::cast<ng_float_t>(t[0]);
            auto max_steps = py::cast<unsigned>(t[1]);
            auto terminate_when_all_idle_or_stuck = py::cast<bool>(t[2]);
            auto record_config = py::cast<RecordConfig>(t[3]);
            unsigned seed = py::cast<unsigned>(t[4]);
            unsigned steps = py::cast<unsigned>(t[6]);
            ExperimentalRun::State state =
                static_cast<ExperimentalRun::State>(py::cast<int>(t[5]));
            ExperimentalRun::tp b = py::cast<ExperimentalRun::tp>(t[7]);
            ExperimentalRun::tp e = py::cast<ExperimentalRun::tp>(t[8]);
            const std::string yaml = py::cast<std::string>(t[9]);
            auto world = py::cast<std::shared_ptr<World>>(
                YAML::load_string_py<PyWorld>(yaml));
            auto records =
                py::cast<std::map<std::string, std::shared_ptr<Dataset>>>(
                    t[10]);
            auto run = ExperimentalRun(
                world, {time_step, max_steps, terminate_when_all_idle_or_stuck},
                record_config, seed, state, steps, b, e, yaml, records);
            run.go_to_step(-1, false, false, false);
            return run;
          }));

  py::class_<Scenario, PyScenario, HasRegister<Scenario>, HasProperties,
             std::shared_ptr<Scenario>>
      scenario(m, "Scenario", py::dynamic_attr(),
               DOC(navground, sim, Scenario));

  py::class_<PyExperiment, std::shared_ptr<PyExperiment>> experiment(
      m, "Experiment", DOC(navground, sim, Experiment));
  experiment
      .def(py::init<ng_float_t, int>(), py::arg("time_step") = 0.1,
           py::arg("steps") = 1000, DOC(navground, sim, Experiment, Experiment))
      .def_property("has_finished", &Experiment::has_finished, nullptr,
                    DOC(navground, sim, Experiment, property_has_finished))
      .def_property("is_running", &Experiment::is_running, nullptr,
                    DOC(navground, sim, Experiment, property_is_running))
      .def_property("time_step", &Experiment::get_time_step,
                    &Experiment::set_time_step,
                    DOC(navground, sim, Experiment, property_time_step))
      .def_property("steps", &Experiment::get_steps, &Experiment::set_steps,
                    DOC(navground, sim, Experiment, property_steps))
      .def_property("terminate_when_all_idle_or_stuck",
                    &Experiment::get_terminate_when_all_idle_or_stuck,
                    &Experiment::set_terminate_when_all_idle_or_stuck,
                    DOC(navground, sim, Experiment,
                        property_terminate_when_all_idle_or_stuck))
      .def_readwrite("record_config", &Experiment::record_config,
                     // DOC(navground, sim, Experiment, record_config)
                     "A reference to the record config on which data to record")
      // .def_readwrite("run_config", &Experiment::run_config,
      //                DOC(navground, sim, Experiment, run_config))
      .def_property("runs", &Experiment::get_runs, nullptr,
                    DOC(navground, sim, Experiment, property_runs))
      .def(
          "get_run",
          [](const PyExperiment &exp, unsigned index) {
            return exp.get_runs().at(index);
          },
          py::arg("index"))
      .def_property(
          "scenario", [](const PyExperiment *exp) { return exp->scenario; },
          &PyExperiment::set_scenario,
          DOC(navground, sim, Experiment, scenario))
      .def_readwrite("number_of_runs", &Experiment::number_of_runs,
                     DOC(navground, sim, Experiment, number_of_runs))
      .def_readwrite("run_index", &Experiment::run_index,
                     DOC(navground, sim, Experiment, run_index))
      .def_readwrite("save_directory", &Experiment::save_directory,
                     DOC(navground, sim, Experiment, save_directory))
      .def_readwrite("name", &Experiment::name,
                     DOC(navground, sim, Experiment, name))
      .def_readwrite("reset_uids", &Experiment::reset_uids,
                     DOC(navground, sim, Experiment, reset_uids))
      .def_readwrite(
          "record_scenario_properties", &Experiment::record_scenario_properties,
          DOC(navground, sim, Experiment, record_scenario_properties))
      .def_property("path", &Experiment::get_path, nullptr,
                    DOC(navground, sim, Experiment, property_path))
      // .def("add_callback", &Experiment::add_callback, py::arg("callback"),
      //      DOC(navground, sim, Experiment, add_callback))
      .def_property(
          "scenario_init_callback", &Experiment::get_scenario_init_callback,
          &Experiment::set_scenario_init_callback,
          DOC(navground, sim, Experiment, property_scenario_init_callback))
      .def_property("run_callbacks", &Experiment::get_run_callbacks,
                    &Experiment::set_run_callbacks,
                    DOC(navground, sim, Experiment, property_run_callbacks))
      .def("clear_run_callbacks", &Experiment::clear_run_callbacks,
           DOC(navground, sim, Experiment, clear_run_callbacks))
      .def("add_run_callback", &PyExperiment::add_run_callback_py,
           py::arg("callback"), py::arg("at_init") = false,
           DOC(navground, sim, Experiment, add_run_callback))
      // .def("add_run_callback", &Experiment::add_run_callback,
      //      py::arg("callback"), py::arg("at_init") = false,
      //      py::keep_alive<1, 2>(),
      //      DOC(navground, sim, Experiment, add_run_callback))
      .def("run_once", &Experiment::run_once, py::arg("seed"),
           py::return_value_policy::reference,
           DOC(navground, sim, Experiment, run_once))
      .def("run", &Experiment::run, py::arg("keep") = true,
           py::arg("number_of_threads") = 1,
           py::arg("start_index") = py::none(),
           py::arg("number_of_runs") = py::none(),
           py::arg("data_path") = py::none(),
           DOC(navground, sim, Experiment, run))
      // .def("init_run", &Experiment::init_run, py::arg("seed"),
      // DOC(navground, sim, Experiment, init_run))
      .def("start", &Experiment::start, py::arg("path") = py::none(),
           DOC(navground, sim, Experiment, start))
      .def("stop", &Experiment::stop, py::arg("save_runs") = false,
           DOC(navground, sim, Experiment, stop))
      .def("init_run", &Experiment::init_run, py::arg("seed"),
           py::arg("world") = py::none(),
           DOC(navground, sim, Experiment, init_run))
      .def("update_run", &Experiment::update_run, py::arg("run"),
           DOC(navground, sim, Experiment, update_run))
      .def("start_run", &Experiment::start_run, py::arg("run"),
           DOC(navground, sim, Experiment, start_run))
      .def("stop_run", &Experiment::stop_run, py::arg("run"),
           DOC(navground, sim, Experiment, stop_run))
      .def("remove_all_runs", &Experiment::remove_all_runs,
           DOC(navground, sim, Experiment, remove_all_runs))
      .def("add_run", &Experiment::add_run, py::arg("seed"), py::arg("run"),
           DOC(navground, sim, Experiment, add_run))
      .def("remove_run", &Experiment::remove_run, py::arg("seed"),
           DOC(navground, sim, Experiment, remove_run))
      .def_readwrite("_probes", &PyExperiment::_py_probe_factories)
      .def_readwrite("_record_probes",
                     &PyExperiment::_py_record_probe_factories)
      .def_readwrite("_group_record_probes",
                     &PyExperiment::_py_group_record_probe_factories)
      .def("add_probe", &PyExperiment::add_probe_py, py::arg("probe"),
           DOC(navground, sim, Experiment, add_probe))
      .def("add_record_probe", &PyExperiment::add_record_probe_py,
           py::arg("key"), py::arg("probe"), R"doc(
Register a probe to record data to during all runs.

:param key: the name associated to the record
:type key: str
:param probe: the probe factory like a RecordProbe class.
:type key: typing.Callable[[navground.sim.Dataset], sim.RecordProbe]
)doc")
      .def("add_group_record_probe", &PyExperiment::add_group_record_probe_py,
           py::arg("key"), py::arg("probe"),
           R"doc(
Register a probe to record a group of data to during all runs.

:param key: the name associated to the group
:type key: str
:param probe: the probe factory like a GroupRecordProbe class.
:type key: typing.Callable[[], sim.GroupRecordProbe]
)doc")
      .def("save", &Experiment::save, py::arg("directory") = py::none(),
           py::arg("path") = py::none(), DOC(navground, sim, Experiment, save))
      .def_property("duration", &Experiment::get_duration, nullptr,
                    DOC(navground, sim, Experiment, property_duration))
      .def_property("begin_time", &Experiment::get_begin_time, nullptr,
                    DOC(navground, sim, Experiment, property_begin_time))
      .def_static("schema", &YAML::schema_py<Experiment>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_py<PyExperiment>, py::arg("value"),
                  YAML::load_string_py_doc("experiment", "Experiment").c_str())
      .def("dump", &YAML::dump<PyExperiment>, YAML::dump_doc());

  experiment.def(py::pickle(
      [](const PyExperiment &exp) {
        // __getstate__
        return py::make_tuple(
            exp.record_config, exp.run_config, exp.number_of_runs,
            exp.save_directory, exp.name, exp.scenario, exp.run_index,
            exp.reset_uids, exp.record_scenario_properties,
            exp._py_probe_factories, exp._py_record_probe_factories,
            exp._py_group_record_probe_factories, exp._py_run_callbacks);
      },
      [](py::tuple t) { // __setstate__
        if (t.size() != 13) {
          throw std::runtime_error("Invalid state!");
        }
        PyExperiment exp;
        exp.record_config = py::cast<RecordConfig>(t[0]);
        exp.run_config = py::cast<RunConfig>(t[1]);
        exp.number_of_runs = py::cast<unsigned>(t[2]);
        exp.save_directory = py::cast<std::filesystem::path>(t[3]);
        exp.name = py::cast<std::string>(t[4]);
        exp.scenario = py::cast<std::shared_ptr<Scenario>>(t[5]);
        exp.run_index = py::cast<unsigned>(t[6]);
        exp.reset_uids = py::cast<bool>(t[7]);
        exp.record_scenario_properties = py::cast<bool>(t[8]);
        exp._py_probe_factories = py::cast<std::vector<py::object>>(t[9]);
        exp._py_record_probe_factories =
            py::cast<std::map<std::string, py::object>>(t[10]);
        exp._py_group_record_probe_factories =
            py::cast<std::map<std::string, py::object>>(t[11]);
        for (const auto &[k, vs] :
             py::cast<std::map<bool, std::vector<py::object>>>(t[12])) {
          for (const auto &v : vs) {
            exp.add_run_callback_py(v, k);
          }
        }
        return exp;
      }));

  py::class_<Scenario::Group, PyGroup, std::shared_ptr<Scenario::Group>>(
      scenario, "Group", DOC(navground, sim, Scenario_Group))
      .def(py::init<>(), "")
      // .def("dump", &YAML::dump<PyGroup>, YAML::dump_doc())
      // .def_static("load", &YAML::load_string_py<PyGroup>, py::arg("value"),
      //             YAML::load_string_py_doc("group", "Group").c_str())
      .def("add_to_world", &Scenario::Group::add_to_world,
           // [](PyGroup &g, World *world,
           //    std::optional<unsigned> seed = std::nullopt) {
           //   g.add_to_world(world, seed);
           // },
           py::arg("world"), py::arg("seed") = std::nullopt,
           DOC(navground, sim, Scenario_Group, add_to_world));

  py::class_<AgentSampler<PyWorld>, Scenario::Group,
             std::shared_ptr<AgentSampler<PyWorld>>>
      agent_sampler(m, "AgentSampler");

  agent_sampler
      .def("dump", &YAML::dump<AgentSampler<PyWorld>>, YAML::dump_doc())
      .def_readwrite("once", &AgentSampler<PyWorld>::once,
                     DOC(navground, sim, Sampler, once))
      .def("count", &AgentSampler<PyWorld>::count,
           DOC(navground, sim, Sampler, count))
      .def("done", &AgentSampler<PyWorld>::done,
           DOC(navground, sim, Sampler, done))
      .def("reset", &AgentSampler<PyWorld>::reset,
           py::arg("index") = std::nullopt, py::arg("keep") = false)
      .def("add_to_world", &AgentSampler<PyWorld>::add_to_world,
           py::arg("world"), py::arg("seed") = std::nullopt,
           DOC(navground, sim, Scenario_Group, add_to_world))
      .def(
          "sample",
          [](AgentSampler<PyWorld> &sampler, World &world) {
            RandomGenerator &rg = world.get_random_generator();
            return sampler.sample(rg);
          },
          py::arg("world"), R"doc(
Draws a sample using the world's random generator.

:param world:         The world.

:raises RuntimeError: When the generator is exhausted 
                      (i.e., when :py:meth:`done` returns true)

:return:              The new sample              
)doc")
      .def_static("load", &YAML::load_string_py<AgentSampler<PyWorld>>,
                  py::arg("value"),
                  YAML::load_string_py_doc("group", "AgentSampler").c_str());

  scenario.def(py::init<>())
      .def("init_world", &Scenario::init_world, py::arg("world"),
           py::arg("seed") = std::nullopt,
           DOC(navground, sim, Scenario, init_world))
      .def("apply_inits", &Scenario::apply_inits, py::arg("world"),
           DOC(navground, sim, Scenario, apply_inits))
      .def("set_attributes", &Scenario::set_attributes, py::arg("world"),
           DOC(navground, sim, Scenario, set_attributes))
      .def(
          "make_world",
          [](PyScenario &scenario, std::optional<int> seed = std::nullopt) {
            return scenario.make_world_py(seed);
          },
          py::arg("seed") = std::nullopt,
          DOC(navground, sim, Scenario, make_world))
// .def("sample", &Scenario::sample)
// .def("reset", &Scenario::reset)
#if 0
      .def_property("groups", py::cpp_function([](const Scenario *scenario) {
                      std::vector<Scenario::Group *> gs;
                      std::transform(scenario->groups.cbegin(),
                                     scenario->groups.cend(),
                                     std::back_inserter(gs),
                                     [](const auto &g) { return g.get(); });
                      return gs;
                    }),
                    nullptr, DOC(navground, sim, Scenario, obstacles))
      .def(
          "add_group",
          [](Scenario *scenario, py::object &group) {
            std::unique_ptr<Scenario::Group> g =
                group.cast<std::unique_ptr<Scenario::Group>>();
            scenario->groups.push_back(std::move(g));
          },
          "Add a group.")
#endif
      .def_readwrite("obstacles", &Scenario::obstacles,
                     DOC(navground, sim, Scenario, obstacles))
      .def_readwrite("walls", &Scenario::walls,
                     DOC(navground, sim, Scenario, walls))
      .def_property("groups", &Scenario::get_groups, nullptr,
                    DOC(navground, sim, Scenario, property_groups))
      .def("get_group", &Scenario::get_group, py::arg("index"),
           DOC(navground, sim, Scenario, get_group))
      .def("clear_groups", &clear_groups_py,
           DOC(navground, sim, Scenario, clear_groups))
      .def("remove_group", &remove_group_py, py::arg("group"),
           DOC(navground, sim, Scenario, remove_group))
      .def("remove_group_at_index", &remove_group_at_index_py, py::arg("index"),
           DOC(navground, sim, Scenario, remove_group_at_index))
      .def("add_group", &add_group_py, py::arg("group"),
           DOC(navground, sim, Scenario, add_group))
      .def_property("property_samplers", &Scenario::get_property_samplers,
                    nullptr,
                    DOC(navground, sim, Scenario, property_property_samplers))
      .def("clear_property_samplers", &Scenario::clear_property_samplers,
           DOC(navground, sim, Scenario, clear_property_samplers))
      .def("remove_property_sampler", &Scenario::remove_property_sampler,
           py::arg("name"),
           DOC(navground, sim, Scenario, remove_property_sampler))
      .def("add_property_sampler", &Scenario::add_property_sampler,
           py::arg("name"), py::arg("value"),
           DOC(navground, sim, Scenario, add_property_sampler))
      .def_readwrite("bounding_box", &Scenario::bounding_box,
                     DOC(navground, sim, Scenario, bounding_box))
      // py::return_value_policy::reference)
      // .def_property("initializers",
      // &Scenario::get_initializers, nullptr)
      .def(
          "add_init", //&Scenario::add_init,
          [](Scenario &scenario, const py::function &initializer) {
            const auto key =
                scenario.add_init(initializer.cast<Scenario::Init>());
            set_py_key(py::cast(scenario), key, initializer, "inits");
            return key;
          },
          py::arg("initializer"), DOC(navground, sim, Scenario, add_init))
      .def(
          "set_init",
          [](Scenario &scenario, const std::string &key,
             const py::function &initializer) {
            scenario.set_init(key, initializer.cast<Scenario::Init>());
            set_py_key(py::cast(scenario), key, initializer, "inits");
          },
          py::arg("key"), py::arg("initializer"),
          DOC(navground, sim, Scenario, set_init))

      .def(
          "remove_init", //&Scenario::remove_init,
          [](Scenario &scenario, const std::string &key) {
            scenario.remove_init(key);
            remove_py_key(py::cast(scenario), key, "inits");
          },
          py::arg("key"), DOC(navground, sim, Scenario, remove_init))
      .def(
          "clear_inits", //&Scenario::clear_inits,
          [](Scenario &scenario) {
            scenario.clear_inits();
            clear_py_map(py::cast(scenario), "inits");
          },
          DOC(navground, sim, Scenario, clear_inits))
      .def_property("inits", &Scenario::get_inits, nullptr,
                    DOC(navground, sim, Scenario, property_inits))
      .def(
          "set_yaml",
          [](Scenario &scenario, const std::string &value) {
            YAML::Node node = YAML::Load(value);
            YAML::update_scenario(scenario, node);
          },
          py::arg("value"), "Sets the yaml representation")
      .def_static("load", &YAML::load_string_py<PyScenario>, py::arg("value"),
                  YAML::load_string_py_doc("scenario", "Scenario").c_str());

  py::class_<SimpleScenario, Scenario, std::shared_ptr<SimpleScenario>> simple(
      m, "SimpleScenario", DOC(navground, sim, SimpleScenario));
  simple.def(py::init<>(), DOC(navground, sim, SimpleScenario, SimpleScenario));

  py::class_<AntipodalScenario, Scenario, std::shared_ptr<AntipodalScenario>>
      antipodal(m, "AntipodalScenario", DOC(navground, sim, AntipodalScenario));
  antipodal
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, bool>(),
           py::arg("radius") = AntipodalScenario::default_radius,
           py::arg("tolerance") = AntipodalScenario::default_tolerance,
           py::arg("position_noise") =
               AntipodalScenario::default_position_noise,
           py::arg("orientation_noise") =
               AntipodalScenario::default_orientation_noise,
           py::arg("shuffle") = false,
           // AntipodalScenario::default_shuffle,
           DOC(navground, sim, AntipodalScenario, AntipodalScenario))
      .def_property("radius", &AntipodalScenario::get_radius,
                    &AntipodalScenario::set_radius,
                    DOC(navground, sim, AntipodalScenario, property_radius))
      .def_property("tolerance", &AntipodalScenario::get_tolerance,
                    &AntipodalScenario::set_tolerance,
                    DOC(navground, sim, AntipodalScenario, property_tolerance))
      .def_property("shuffle", &AntipodalScenario::get_shuffle,
                    &AntipodalScenario::set_shuffle,
                    DOC(navground, sim, AntipodalScenario, property_shuffle))
      .def_property(
          "position_noise", &AntipodalScenario::get_position_noise,
          &AntipodalScenario::set_position_noise,
          DOC(navground, sim, AntipodalScenario, property_position_noise))
      .def_property(
          "orientation_noise", &AntipodalScenario::get_orientation_noise,
          &AntipodalScenario::set_orientation_noise,
          DOC(navground, sim, AntipodalScenario, property_orientation_noise));

  py::class_<CrossScenario, Scenario, std::shared_ptr<CrossScenario>> cross(
      m, "CrossScenario", DOC(navground, sim, CrossScenario, CrossScenario));
  cross
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, bool, ng_float_t>(),
           py::arg("side") = CrossScenario::default_side,
           py::arg("tolerance") = CrossScenario::default_tolerance,
           py::arg("agent_margin") = CrossScenario::default_agent_margin,
           py::arg("add_safety_to_agent_margin") =
               CrossScenario::default_add_safety_to_agent_margin,
           py::arg("target_margin") = CrossScenario::default_target_margin,
           DOC(navground, sim, CrossScenario))
      .def_property("side", &CrossScenario::get_side, &CrossScenario::set_side,
                    DOC(navground, sim, CrossScenario, property_side))
      .def_property("tolerance", &CrossScenario::get_tolerance,
                    &CrossScenario::set_tolerance,
                    DOC(navground, sim, CrossScenario, property_tolerance))
      .def_property("agent_margin", &CrossScenario::get_agent_margin,
                    &CrossScenario::set_agent_margin,
                    DOC(navground, sim, CrossScenario, property_agent_margin))
      .def_property("add_safety_to_agent_margin",
                    &CrossScenario::get_add_safety_to_agent_margin,
                    &CrossScenario::set_add_safety_to_agent_margin,
                    DOC(navground, sim, CrossScenario,
                        property_add_safety_to_agent_margin))
      .def_property("target_margin", &CrossScenario::get_target_margin,
                    &CrossScenario::set_target_margin,
                    DOC(navground, sim, CrossScenario, property_target_margin));

  py::class_<CorridorScenario, Scenario, std::shared_ptr<CorridorScenario>>
      corridor(m, "CorridorScenario", DOC(navground, sim, CorridorScenario));
  corridor
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, bool, bool>(),
           py::arg("width") = CorridorScenario::default_width,
           py::arg("length") = CorridorScenario::default_length,
           py::arg("agent_margin") = CorridorScenario::default_agent_margin,
           py::arg("add_safety_to_agent_margin") =
               CorridorScenario::default_add_safety_to_agent_margin,
           py::arg("bidirectional") = CorridorScenario::default_bidirectional,
           DOC(navground, sim, CorridorScenario, CorridorScenario))
      .def_property("width", &CorridorScenario::get_width,
                    &CorridorScenario::set_width,
                    DOC(navground, sim, CorridorScenario, property_width))
      .def_property("length", &CorridorScenario::get_length,
                    &CorridorScenario::set_length,
                    DOC(navground, sim, CorridorScenario, property_length))
      .def_property(
          "agent_margin", &CorridorScenario::get_agent_margin,
          &CorridorScenario::set_agent_margin,
          DOC(navground, sim, CorridorScenario, property_agent_margin))
      .def_property("add_safety_to_agent_margin",
                    &CorridorScenario::get_add_safety_to_agent_margin,
                    &CorridorScenario::set_add_safety_to_agent_margin,
                    DOC(navground, sim, CorridorScenario,
                        property_add_safety_to_agent_margin))
      .def_property(
          "bidirectional", &CorridorScenario::get_bidirectional,
          &CorridorScenario::set_bidirectional,
          DOC(navground, sim, CorridorScenario, property_bidirectional));

  py::class_<CrossTorusScenario, Scenario, std::shared_ptr<CrossTorusScenario>>
      cross_torus(m, "CrossTorusScenario",
                  DOC(navground, sim, CrossTorusScenario));
  cross_torus
      .def(py::init<ng_float_t, ng_float_t, bool>(),
           py::arg("side") = CrossTorusScenario::default_side,
           py::arg("agent_margin") = CrossTorusScenario::default_agent_margin,
           py::arg("add_safety_to_agent_margin") =
               CrossTorusScenario::default_add_safety_to_agent_margin,
           DOC(navground, sim, CrossTorusScenario, CrossTorusScenario))
      .def_property("side", &CrossTorusScenario::get_side,
                    &CrossTorusScenario::set_side,
                    DOC(navground, sim, CrossTorusScenario, property_side))
      .def_property(
          "agent_margin", &CrossTorusScenario::get_agent_margin,
          &CrossTorusScenario::set_agent_margin,
          DOC(navground, sim, CrossTorusScenario, property_agent_margin))
      .def_property("add_safety_to_agent_margin",
                    &CrossTorusScenario::get_add_safety_to_agent_margin,
                    &CrossTorusScenario::set_add_safety_to_agent_margin,
                    DOC(navground, sim, CrossTorusScenario,
                        property_add_safety_to_agent_margin));

  py::class_<PropertySampler, std::shared_ptr<PropertySampler>> sampler(
      m, "Sampler", DOC(navground, sim, PropertySampler));

  sampler.def("__repr__", &sampler_to_string)
      .def(
          "sample",
          [](PropertySampler &sampler, World &world) {
            return sampler.sample(world.get_random_generator());
          },
          py::arg("world"), R"doc(
Draws a sample using the world's random generator.

:param world:         The world.

:raises RuntimeError: When the generator is exhausted 
                      (i.e., when :py:meth:`done` returns true)

:return:              The new sample
)doc")
      .def_readonly("type_name", &PropertySampler::type_name,
                    "The samples type name")
      .def_readwrite("once", &PropertySampler::once,
                     DOC(navground, sim, Sampler, once))
      .def("reset", &PropertySampler::reset, py::arg("index") = std::nullopt,
           py::arg("keep") = false, DOC(navground, sim, Sampler, reset))
      .def("count", &PropertySampler::count,
           DOC(navground, sim, Sampler, count))
      .def("done", &PropertySampler::done, DOC(navground, sim, Sampler, done))
      .def_static("load", &YAML::load_property_sampler, py::arg("value"),
                  py::arg("type_name"), R"doc(
Load a Sampler from a YAML string.

:param value:     The YAML string.
:param type_name: The samples type name.
:return:          The loaded Sampler or ``None`` if loading fails.
:rtype:           Sampler| None
)doc")
      .def(
          "dump",
          [](const PropertySampler *sampler) { return YAML::dump(sampler); },
          YAML::dump_doc());

  m.def("use_compact_samplers", &YAML::set_use_compact_samplers,
        py::arg("value"),
        "Whether to represent sampler compactly in YAML when possible");

  // add [partial] pickle support
  // pickle_via_yaml<PropertySampler>(sampler);
  pickle_via_yaml<PyStateEstimation>(se);
  pickle_via_yaml<PyStateEstimation>(bse);
  pickle_via_yaml<PyStateEstimation>(lse);
  pickle_via_yaml<PyStateEstimation>(ose);
  pickle_via_yaml<PyStateEstimation>(dse);
  pickle_via_yaml<PyStateEstimation>(cse);
  pickle_via_yaml<PyStateEstimation>(sse);
  pickle_via_yaml<PyStateEstimation>(marker_se);
  pickle_via_yaml<PyStateEstimation>(gmse);
  pickle_via_yaml<PyStateEstimation>(boundary_sensor);
  pickle_via_yaml<PyTask>(task);
  pickle_via_yaml<PyTask>(waypoints);
  pickle_via_yaml<PyTask>(direction);
  pickle_via_yaml<PyTask>(gotopose);
  pickle_via_yaml<PyScenario>(scenario, init_scenario);
  pickle_via_yaml<PyScenario>(simple, init_scenario);
  pickle_via_yaml<PyScenario>(antipodal, init_scenario);
  pickle_via_yaml<PyScenario>(cross, init_scenario);
  pickle_via_yaml<PyScenario>(cross_torus, init_scenario);
  pickle_via_yaml<PyScenario>(corridor, init_scenario);
  pickle_via_yaml<PyWorld>(world);
  pickle_via_yaml<PyAgent>(agent);
  pickle_via_yaml_native<AgentSampler<PyWorld>>(agent_sampler);
  // pickle_via_yaml<PyExperiment>(experiment);

  m.def(
      "bundle_schema",
      []() { return YAML::to_py(navground::sim::bundle_schema()); },
      R"doc(
Returns the bundle json-schema

:return: json-schema
:rtype: :py:type:`dict[str, typing.Any]`
)doc");

  m.def(
      "uses_doubles",
      []() { return std::is_same<ng_float_t, float>::value == false; },
      "Returns whether navground has been compiled to use floats or doubles");

  m.def("get_build_info", &navground::sim_py::build_info,
        DOC(navground, core, get_build_info));
  m.def("get_build_dependencies", &build_dependencies_sim_py,
        DOC(navground, core, get_build_dependencies));
  // m.def("load_plugins", &load_plugins, py::arg("plugins") = "",
  //       py::arg("env") = "", py::arg("directory") = py::none());
}
