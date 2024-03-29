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
#include "navground/core/behavior.h"
#include "navground/core/kinematics.h"
#include "navground/core/plugins.h"
#include "navground/core/types.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/dataset.h"
#include "navground/sim/experiment.h"
#include "navground/sim/experimental_run.h"
#include "navground/sim/probe.h"
#include "navground/sim/scenario.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/corridor.h"
#include "navground/sim/scenarios/cross.h"
#include "navground/sim/scenarios/cross_torus.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/state_estimations/geometric_bounded.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/state_estimations/sensor_discs.h"
#include "navground/sim/state_estimations/sensor_lidar.h"
#include "navground/sim/task.h"
#include "navground/sim/tasks/waypoints.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/world.h"
#include "navground_py/register.h"
#include "navground_py/yaml.h"

using namespace navground::core;
using namespace navground::sim;

namespace py = pybind11;

// PYBIND11_MAKE_OPAQUE(std::map<unsigned, ExperimentalRun>);

void set_dataset_type_py(Dataset &dataset, const py::object &obj);
void set_dataset_data_py(Dataset &dataset, const py::object &obj,
                         bool append = false);

template <typename T>
struct get<T, py::object> {
  static typename T::Native *ptr(const py::object &c) {
    return c.cast<typename T::Native *>();
  }
};

struct PyBehavior : public Behavior {
  using C = py::object;
  using Native = Behavior;

  static py::object make_type(const std::string &type) {
    py::module_ nav = py::module_::import("navground.core");
    return nav.attr("Behavior").attr("make_type")(type);
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
};

struct PyTask : Task, virtual PyHasRegister<Task> {
  /* Inherit the constructors */
  using Task::Task;
  using PyHasRegister<Task>::C;
  using Native = Task;

  void update(Agent *agent, World *world, ng_float_t time) override {
    PYBIND11_OVERRIDE(void, Task, update, agent, world, time);
  }

  void prepare(Agent *agent, World *world) const override {
    PYBIND11_OVERRIDE(void, Task, prepare, agent, world);
  }

  bool done() const override { PYBIND11_OVERRIDE(bool, Task, done); }

  const Properties &get_properties() const override {
    const std::string t = get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

struct PyStateEstimation : StateEstimation,
                           virtual PyHasRegister<StateEstimation> {
  /* Inherit the constructors */
  using StateEstimation::StateEstimation;
  using PyHasRegister<StateEstimation>::C;
  using Native = StateEstimation;

  void update(Agent *agent, World *world,
              EnvironmentState *state) const override {
    PYBIND11_OVERRIDE(void, StateEstimation, update, agent, world, state);
  }

  void prepare(Agent *agent, World *world) const override {
    PYBIND11_OVERRIDE(void, StateEstimation, prepare, agent, world);
  }

  const Properties &get_properties() const override {
    const std::string t = get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

struct PySensor : Sensor, virtual PyStateEstimation {
  Sensor::Description get_description() const override {
    PYBIND11_OVERRIDE_PURE(Sensor::Description, Sensor, get_description);
  }
};

class PyAgent : public Agent {
 public:
  using B = PyBehavior;
  using K = PyKinematics;
  using T = PyTask;
  using S = PyStateEstimation;
  using Native = PyAgent;

  virtual ~PyAgent() = default;

  using C = py::object;

  PyAgent(ng_float_t radius = 0, const py::object &behavior = py::none(),
          const py::object &kinematics = py::none(),
          const py::object &task = py::none(),
          const py::object &estimation = py::none(),
          ng_float_t control_period = 0, unsigned id = 0)
      : Agent(radius, nullptr, nullptr, nullptr, nullptr, control_period, id) {
    set_kinematics(kinematics);
    set_behavior(behavior);
    set_state_estimation(estimation);
    set_task(task);
  }

  static py::object make(ng_float_t radius = 0,
                         const py::object &behavior = py::none(),
                         const py::object &kinematics = py::none(),
                         const py::object &task = py::none(),
                         const py::object &estimation = py::none(),
                         ng_float_t control_period = 0, unsigned id = 0) {
    auto a = std::make_shared<PyAgent>(radius, behavior, kinematics, task,
                                       estimation, control_period, id);
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
    py_state_estimation = obj;
    Agent::set_state_estimation(obj.cast<std::shared_ptr<StateEstimation>>());
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
  py::object py_state_estimation;
  py::object py_task;
};

struct PyWorld : public World {
  /* Inherit the constructors */
  using World::World;
  using A = PyAgent;

  ~PyWorld() = default;

  std::vector<py::object> py_agents;

  void add_agent(const py::object &value) {
    py_agents.push_back(value);
    std::shared_ptr<Agent> agent = value.cast<std::shared_ptr<Agent>>();
    World::add_agent(agent);
  }
};

struct PyGroup : public virtual Scenario::Group {
  /* Inherit the constructors */
  PyGroup(){};

  void add_to_world(World *world) override {
    PYBIND11_OVERRIDE_PURE(void, Scenario::Group, add_to_world, world);
  }

  void reset() override {
    PYBIND11_OVERRIDE_PURE(void, Scenario::Group, reset);
  }
};

struct PyScenario : public Scenario, virtual PyHasRegister<Scenario> {
  /* Inherit the constructors */
  using Scenario::Scenario;
  using Native = Scenario;

  void init_world(World *world,
                  std::optional<int> seed = std::nullopt) override {
    PYBIND11_OVERRIDE(void, Scenario, init_world, world, seed);
  }

  const Properties &get_properties() const override {
    const std::string t = PyHasRegister<Scenario>::get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
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

  ShapeMap get_shapes(const World &world) const override {
    PYBIND11_OVERRIDE(ShapeMap, GroupRecordProbe, get_shapes, world);
  }
};

struct PyExperiment : public Experiment {
  /* Inherit the constructors */
  using Experiment::Experiment;

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
    for (size_t i = start_index.value_or(run_index); i < max_index; i++) {
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
        if (!sim_run) continue;
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

  void instantiate_record_probe(const std::string &name, const py::object cls,
                                ExperimentalRun &run) {
    const auto dtype = cls.attr("dtype");
    auto ds = run.add_record(name);
    set_dataset_type_py(*ds, dtype);
    auto obj = cls.attr("__call__")(ds);
    auto probe = obj.cast<std::shared_ptr<Probe>>();
    run.add_probe(probe);
    _py_probes[run.get_seed()].push_back(obj);
  }

  void instantiate_group_record_probe(const std::string &name,
                                      const py::object cls,
                                      ExperimentalRun &run) {
    const auto dtype = cls.attr("dtype");
    const auto factory = [name, dtype, &run](const std::string &sub_key) {
      auto ds = run.add_record(sub_key, name);
      set_dataset_type_py(*ds, dtype);
      return ds;
    };
    auto obj = cls.attr("__call__")();
    auto probe = obj.cast<std::shared_ptr<GroupRecordProbe>>();
    probe->set_factory(factory);
    run.add_probe(probe);
    _py_probes[run.get_seed()].push_back(obj);
  }
};

namespace YAML {

template <>
struct convert<PyAgent> {
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
      if (!rhs.get_state_estimation() && node["state_estimation"]) {
        auto value = load_node_py<PyStateEstimation>(node["state_estimation"]);
        rhs.set_state_estimation(value);
      }
      return true;
    }
    return false;
  }
};

template <>
struct convert<std::shared_ptr<PyAgent>> {
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

PyWorld load_world(const Node &node) {
  PyWorld world;
  convert_world<PyAgent>::decode(node, world);
  return world;
}

py::object load_scenario(const Node &node) {
  auto obj = make_type_from_yaml_py<PyScenario>(node);
  if (obj.is_none()) {
    auto ws = std::make_shared<Scenario>();
    obj = py::cast(ws);
  }
  convert_scenario<PyWorld>::decode(node, obj.cast<Scenario &>());
  return obj;
};

void update_scenario(Scenario &scenario, const Node &node) {
  convert_scenario<PyWorld>::decode(node, scenario);
};

std::string dump_scenario(Scenario *sampler) {
  auto node = convert_scenario<PyWorld>::encode(*sampler);
  Emitter out;
  out << node;
  return std::string(out.c_str());
};

// TODO(move to PyExperiment that creates a PyWorld instead of a World)
// Experiment load_experiment(const Node &node) {
//   Experiment experiment;
//   convert_experiment<PyAgent, PyBehavior, PyKinematics, PyTask,
//                      PyStateEstimation, PyWorld>::decode(node, experiment);
//   return experiment;
// };

// std::string dump_experiment(const Experiment *experiment) {
//   if (!experiment) return "";
//   // const auto node = convert_experiment<PyAgent, PyBehavior, PyKinematics,
//   // PyTask,
//   //                    PyStateEstimation,
//   //                    PyWorld>::encode(*experiment);
//   Node node;
//   Emitter out;
//   out << node;
//   return std::string(out.c_str());
// };

template <>
struct convert<PyExperiment> {
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
        rhs.set_scenario(load_scenario(node["scenario"]));
      }
      return true;
    }
    return false;
  }
};

}  // namespace YAML

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

static std::map<std::string, py::array> as_dict_array(
    const std::map<std::string, std::shared_ptr<Dataset>> &records) {
  std::map<std::string, py::array> m;
  for (const auto &[key, ds] : records) {
    m.emplace(key, as_array(*ds));
  }
  return m;
}

template <typename T>
static py::array make_empty_array() {
  return py::array(0, static_cast<const T *>(nullptr));
}

void set_dataset_type_py(Dataset &dataset, const py::object &obj) {
  py::module_ np = py::module_::import("numpy");
  py::dtype dtype = np.attr("dtype")(obj);
  if (dtype.is(py::dtype::of<int8_t>())) {
    dataset.set_dtype<int8_t>();
  } else if (dtype.is(py::dtype::of<int16_t>())) {
    dataset.set_dtype<int16_t>();
  } else if (dtype.is(py::dtype::of<int32_t>())) {
    dataset.set_dtype<int32_t>();
  } else if (dtype.is(py::dtype::of<int64_t>())) {
    dataset.set_dtype<int64_t>();
  } else if (dtype.is(py::dtype::of<uint8_t>()) ||
             dtype.is(py::dtype::of<bool>())) {
    dataset.set_dtype<uint8_t>();
  } else if (dtype.is(py::dtype::of<uint16_t>())) {
    dataset.set_dtype<uint16_t>();
  } else if (dtype.is(py::dtype::of<uint32_t>())) {
    dataset.set_dtype<uint32_t>();
  } else if (dtype.is(py::dtype::of<uint64_t>())) {
    dataset.set_dtype<uint64_t>();
  } else if (dtype.is(py::dtype::of<float>())) {
    dataset.set_dtype<float>();
  } else if (dtype.is(py::dtype::of<double>())) {
    dataset.set_dtype<double>();
  } else {
    py::print("Type unknown", dtype);
  }
}

Dataset::Data data_of_type(py::dtype dtype, void *ptr, const size_t size) {
  if (dtype.is(py::dtype::of<int8_t>())) {
    auto begin = reinterpret_cast<int8_t *>(ptr);
    return std::vector<int8_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<int16_t>())) {
    auto begin = reinterpret_cast<int16_t *>(ptr);
    return std::vector<int16_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<int32_t>())) {
    auto begin = reinterpret_cast<int32_t *>(ptr);
    return std::vector<int32_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<int64_t>())) {
    auto begin = reinterpret_cast<int64_t *>(ptr);
    return std::vector<int64_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<uint8_t>()) ||
             dtype.is(py::dtype::of<bool>())) {
    auto begin = reinterpret_cast<uint8_t *>(ptr);
    return std::vector<uint8_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<uint16_t>())) {
    auto begin = reinterpret_cast<uint16_t *>(ptr);
    return std::vector<uint16_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<uint32_t>())) {
    auto begin = reinterpret_cast<uint32_t *>(ptr);
    return std::vector<uint32_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<uint64_t>())) {
    auto begin = reinterpret_cast<uint64_t *>(ptr);
    return std::vector<uint64_t>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<float>())) {
    auto begin = reinterpret_cast<float *>(ptr);
    return std::vector<float>(begin, begin + size);
  } else if (dtype.is(py::dtype::of<double>())) {
    auto begin = reinterpret_cast<double *>(ptr);
    return std::vector<double>(begin, begin + size);
  } else {
    auto begin = reinterpret_cast<int8_t *>(ptr);
    return std::vector<int8_t>(begin, begin + size);
  }
}

void set_dataset_data_py(Dataset &dataset, const py::array &value,
                         bool append = false) {
  const auto sshape = value.request().shape;
  const Dataset::Shape shape(sshape.begin(), sshape.end());
  const Dataset::Shape item_shape(sshape.begin() + 1, sshape.end());
  void *ptr = value.request().ptr;
  const auto count = std::accumulate(std::begin(shape), std::end(shape), 1,
                                     std::multiplies<>{});
  const auto data = data_of_type(value.dtype(), ptr, count);
  if (append) {
    dataset.append(data);
  } else {
    dataset.set_data(data);
    dataset.set_item_shape(item_shape);
  }
}

py::dtype get_dataset_type_py(const Dataset &dataset) {
  return std::visit(
      [](auto &&arg) {
        using T = std::remove_reference_t<decltype(arg[0])>;
        return py::dtype::of<T>();
      },
      dataset.get_data());
}

PYBIND11_MODULE(_navground_sim, m) {
  declare_register<StateEstimation>(m, "StateEstimation");
  declare_register<Task>(m, "Task");
  declare_register<Scenario>(m, "Scenario");
  //  declare_register<PScenario>(m, "Scenario");

  py::class_<Entity, std::shared_ptr<Entity>>(m, "Entity",
                                              DOC(navground, sim, Entity))
      .def_readonly("_uid", &Entity::uid, DOC(navground, sim, Entity, uid));

  py::class_<Wall, Entity, std::shared_ptr<Wall>>(m, "Wall",
                                                  DOC(navground, sim, Wall))
      .def(py::init<Vector2, Vector2>(), py::arg("p1"), py::arg("p2"),
           DOC(navground, sim, Wall, Wall))
      .def(py::init<LineSegment>(), py::arg("line"),
           DOC(navground, sim, Wall, Wall, 3))
      .def_readwrite("line", &Wall::line, DOC(navground, sim, Wall, line));

  py::class_<Obstacle, Entity, std::shared_ptr<Obstacle>>(
      m, "Obstacle", DOC(navground, sim, Obstacle))
      .def(py::init<Vector2, ng_float_t>(), py::arg("position"),
           py::arg("radius"), DOC(navground, sim, Obstacle, Obstacle))
      .def(py::init<Disc>(), py::arg("disc"),
           DOC(navground, sim, Obstacle, Obstacle, 3))
      .def_readwrite("disc", &Obstacle::disc,
                     DOC(navground, sim, Obstacle, disc));

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
           )doc");

  py::class_<Agent, Entity, std::shared_ptr<Agent>>(m, "NativeAgent",
                                                    DOC(navground, sim, Agent))
      .def_readwrite("id", &Agent::id, DOC(navground, sim, Agent, id))
      .def_readwrite("type", &Agent::type, DOC(navground, sim, Agent, type))
      .def_readwrite("color", &Agent::color, DOC(navground, sim, Agent, color))
      .def_readwrite("radius", &Agent::radius,
                     DOC(navground, sim, Agent, radius))
      .def_readwrite("control_period", &Agent::control_period,
                     DOC(navground, sim, Agent, control_period))
      .def_readwrite("pose", &Agent::pose, DOC(navground, sim, Agent, pose))
      .def_readwrite("twist", &Agent::twist, DOC(navground, sim, Agent, twist))
      .def_readwrite("last_cmd", &Agent::last_cmd,
                     DOC(navground, sim, Agent, last_cmd))
      .def_readonly("tags", &Agent::tags, DOC(navground, sim, Agent, tags))
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
      .def_property("controller", &Agent::get_controller, nullptr,
                    py::return_value_policy::reference,
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
           DOC(navground, sim, Agent, actuate, 2));

  py::class_<PyAgent, Agent, Entity, std::shared_ptr<PyAgent>>(
      m, "Agent", py::dynamic_attr(), DOC(navground, sim, Agent))
      .def(py::init<ng_float_t, const py::object &, const py::object &,
                    const py::object &, const py::object &, ng_float_t,
                    unsigned>(),
           py::arg("radius") = 0, py::arg("behavior") = py::none(),
           py::arg("kinematics") = py::none(), py::arg("task") = py::none(),
           py::arg("state_estimation") = py::none(),
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
      .def_property("task", &PyAgent::get_task, &PyAgent::set_task)
      .def_property("state_estimation", &PyAgent::get_state_estimation,
                    &PyAgent::set_state_estimation)
      .def_property("behavior", &PyAgent::get_behavior, &PyAgent::set_behavior)
      .def_property("kinematics", &PyAgent::get_kinematics,
                    &PyAgent::set_kinematics)
      .def_property("controller", &PyAgent::get_controller, nullptr,
                    py::return_value_policy::reference);

  py::class_<World, std::shared_ptr<World>>(m, "NativeWorld",
                                            DOC(navground, sim, World, 2))
      .def(py::init<>(), DOC(navground, sim, World, World))
      .def("add_callback", &World::add_callback, py::arg("callback"),
           py::keep_alive<1, 2>(), DOC(navground, sim, World, add_callback))
      .def("reset_callbacks", &World::reset_callbacks,
           DOC(navground, sim, World, reset_callbacks))
      .def("update", &World::update, py::arg("time_step"),
           DOC(navground, sim, World, update))
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
      .def_property("agents", &World::get_agents, nullptr,
                    DOC(navground, sim, World, property_agents))
      .def_property("walls", &World::get_walls, nullptr,
                    DOC(navground, sim, World, property_walls))
      .def_property("obstacles", &World::get_obstacles, nullptr,
                    DOC(navground, sim, World, property_obstacles))
      .def_property("discs", &World::get_discs, nullptr,
                    DOC(navground, sim, World, property_discs))
      .def_property("line_obstacles", &World::get_line_obstacles, nullptr,
                    DOC(navground, sim, World, property_line_obstacles))
      .def("get_agents_in_region", &World::get_agents_in_region,
           py::arg("bounding_box"),
           DOC(navground, sim, World, get_agents_in_region))
      .def("get_static_obstacles_in_region",
           &World::get_static_obstacles_in_region, py::arg("bounding_box"),
           DOC(navground, sim, World, get_static_obstacles_in_region))
      .def("get_line_obstacles_in_region", &World::get_line_obstacles_in_region,
           py::arg("bounding_box"),
           DOC(navground, sim, World, get_line_obstacles_in_region))
      .def("get_neighbors", &World::get_neighbors, py::arg("agent"),
           py::arg("distance"), DOC(navground, sim, World, get_neighbors))
      .def_property("collisions", &World::get_collisions,
                    &World::set_collisions,
                    DOC(navground, sim, World, property_collisions))
      .def("compute_safety_violation", &World::compute_safety_violation,
           py::arg("agent"),
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
      .def("agents_are_idle_or_stuck", &World::agents_are_idle_or_stuck,
           DOC(navground, sim, World, agents_are_idle_or_stuck))
      .def("in_collision", &World::in_collision, py::arg("e1"), py::arg("e2"),
           DOC(navground, sim, World, in_collision))
      .def("copy_random_generator", &World::copy_random_generator,
           py::arg("world"), DOC(navground, sim, World, copy_random_generator));

  py::class_<PyWorld, World, std::shared_ptr<PyWorld>>(
      m, "World", py::dynamic_attr(), DOC(navground, sim, World))
      .def(py::init<>(), DOC(navground, sim, World, World))
      .def("add_agent", &PyWorld::add_agent, py::arg("agent"),
           DOC(navground, sim, World, add_agent));

  py::class_<StateEstimation, PyStateEstimation, HasRegister<StateEstimation>,
             HasProperties, std::shared_ptr<StateEstimation>>(
      m, "StateEstimation", DOC(navground, sim, StateEstimation))
      .def(py::init<>(), DOC(navground, sim, StateEstimation, StateEstimation))
      .def("update",
           py::overload_cast<Agent *, World *, EnvironmentState *>(
               &StateEstimation::update, py::const_),
           py::arg("agent"), py::arg("world"), py::arg("state"),
           DOC(navground, sim, StateEstimation, update))
      .def_property(
          "type", [](StateEstimation *obj) { return obj->get_type(); }, nullptr,
          "The name associated to the type of an object")
      .def(py::pickle(
          [](StateEstimation *se) {
            // __getstate__
            return py::make_tuple(YAML::dump<StateEstimation>(se));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            std::string node = t[0].cast<std::string>();
            auto obj = YAML::load_string_py<PyStateEstimation>(node);
            return obj.cast<std::shared_ptr<StateEstimation>>();
          }));

  py::class_<BoundedStateEstimation, StateEstimation,
             std::shared_ptr<BoundedStateEstimation>>(
      m, "BoundedStateEstimation", DOC(navground, sim, BoundedStateEstimation))
      .def(py::init<ng_float_t, bool>(),
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
           DOC(navground, sim, BoundedStateEstimation, neighbors_of_agent))
      .def(py::pickle(
          [](BoundedStateEstimation *se) {
            // __getstate__
            return py::make_tuple(YAML::dump<StateEstimation>(se));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            std::string node = t[0].cast<std::string>();
            auto obj = YAML::load_string_py<PyStateEstimation>(node);
            return obj.cast<std::shared_ptr<BoundedStateEstimation>>();
          }));

  py::class_<Sensor, PySensor, StateEstimation, std::shared_ptr<Sensor>>(
      m, "Sensor", DOC(navground, sim, Sensor))
      .def(py::init<>(), DOC(navground, sim, Sensor, Sensor))
      .def_property("description", &Sensor::get_description, nullptr,
                    DOC(navground, sim, Sensor, property_description))
      .def("prepare",
           py::overload_cast<SensingState &>(&Sensor::prepare, py::const_),
           DOC(navground, sim, Sensor, prepare));

  py::class_<LidarStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<LidarStateEstimation>>(
      m, "LidarStateEstimation", DOC(navground, sim, LidarStateEstimation))
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, unsigned>(),
           py::arg("range") = 0.0, py::arg("start_angle") = -M_PI,
           py::arg("field_of_view") = 2 * M_PI, py::arg("resolution") = 100,
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
      .def(py::pickle(
          [](LidarStateEstimation *se) {
            // __getstate__
            return py::make_tuple(YAML::dump<StateEstimation>(se));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            std::string node = t[0].cast<std::string>();
            auto obj = YAML::load_string_py<PyStateEstimation>(node);
            return obj.cast<std::shared_ptr<LidarStateEstimation>>();
          }));

  py::class_<DiscsStateEstimation, Sensor, StateEstimation,
             std::shared_ptr<DiscsStateEstimation>>(
      m, "DiscsStateEstimation", DOC(navground, sim, DiscsStateEstimation))
      .def(py::init<ng_float_t, unsigned, ng_float_t, ng_float_t, bool>(),
           py::arg("range") = 1.0, py::arg("number") = 1,
           py::arg("max_radius") = 1, py::arg("max_speed") = 1,
           py::arg("include_valid") = true,
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
      .def(py::pickle(
          [](DiscsStateEstimation *se) {
            // __getstate__
            return py::make_tuple(YAML::dump<StateEstimation>(se));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            std::string node = t[0].cast<std::string>();
            auto obj = YAML::load_string_py<PyStateEstimation>(node);
            return obj.cast<std::shared_ptr<DiscsStateEstimation>>();
          }));

  py::class_<Task, PyTask, HasRegister<Task>, HasProperties,
             std::shared_ptr<Task>>(m, "Task", DOC(navground, sim, Task))
      .def(py::init<>())
      // .def("update", &Task::update)
      .def_property(
          "type", [](Task *obj) { return obj->get_type(); }, nullptr,
          "The name associated to the type of an object")
      .def("done", &Task::done, DOC(navground, sim, Task, done))
      .def_property("log_size", &Task::get_log_size, nullptr,
                    DOC(navground, sim, Task, property, log_size))
      .def("add_callback", &Task::add_callback, py::arg("callback"),
           DOC(navground, sim, Task, add_callback));

  py::class_<WaypointsTask, Task, std::shared_ptr<WaypointsTask>>(
      m, "WaypointsTask", DOC(navground, sim, WaypointsTask))
      .def(py::init<Waypoints, bool, ng_float_t>(),
           py::arg("waypoints") = Waypoints{},
           py::arg("loop") = WaypointsTask::default_loop,
           py::arg("tolerance") = WaypointsTask::default_tolerance,
           DOC(navground, sim, WaypointsTask, WaypointsTask))
      .def_property("log_size", &WaypointsTask::get_log_size, nullptr,
                    DOC(navground, sim, WaypointsTask, property, log_size))
      .def_property("waypoints", &WaypointsTask::get_waypoints,
                    &WaypointsTask::set_waypoints,
                    DOC(navground, sim, WaypointsTask, property_waypoints))
      .def_property("tolerance", &WaypointsTask::get_tolerance,
                    &WaypointsTask::set_tolerance,
                    DOC(navground, sim, WaypointsTask, property_tolerance))
      .def_property("loop", &WaypointsTask::get_loop, &WaypointsTask::set_loop,
                    DOC(navground, sim, WaypointsTask, property_loop));

  py::class_<RecordConfig>(m, "RecordConfig", DOC(navground, sim, RecordConfig))
      .def(py::init([](bool time, bool pose, bool twist, bool cmd, bool target,
                       bool safety_violation, bool collisions, bool task_events,
                       bool deadlocks, bool efficacy) {
             return new RecordConfig{time,       pose,        twist,
                                     cmd,        target,      safety_violation,
                                     collisions, task_events, deadlocks,
                                     efficacy};
           }),
           py::arg("time") = false, py::arg("pose") = false,
           py::arg("twist") = false, py::arg("cmd") = false,
           py::arg("target") = false, py::arg("safety_violation") = false,
           py::arg("collisions") = false, py::arg("task_events") = false,
           py::arg("deadlocks") = false, py::arg("efficacy") = false)
      // DOC(navground, sim, RecordConfig, RecordConfig)
      .def_readwrite("time", &RecordConfig::time,
                     DOC(navground, sim, RecordConfig, time))
      .def_readwrite("pose", &RecordConfig::pose,
                     DOC(navground, sim, RecordConfig, pose))
      .def_readwrite("twist", &RecordConfig::twist,
                     DOC(navground, sim, RecordConfig, twist))
      .def_readwrite("cmd", &RecordConfig::cmd,
                     DOC(navground, sim, RecordConfig, cmd))
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
      .def_static("all", &RecordConfig::all, py::arg("value"),
                  DOC(navground, sim, RecordConfig, all))
      .def("set_all", &RecordConfig::set_all, py::arg("value"),
           DOC(navground, sim, RecordConfig, set_all))
      .def("__repr__", [](const RecordConfig &value) -> py::str {
        py::str r("RecordConfig(time=");
        r += py::str(py::cast(value.time));
        r += py::str(", pose=") + py::str(py::cast(value.pose));
        r += py::str(", twist=") + py::str(py::cast(value.twist));
        r += py::str(", cmd=") + py::str(py::cast(value.cmd));
        r += py::str(", target=") + py::str(py::cast(value.target));
        r += py::str(", safety_violation=") +
             py::str(py::cast(value.safety_violation));
        r += py::str(", collisions=") + py::str(py::cast(value.collisions));
        r += py::str(", task_events=") + py::str(py::cast(value.task_events));
        r += py::str(", deadlocks=") + py::str(py::cast(value.deadlocks));
        r += py::str(", efficacy=") + py::str(py::cast(value.efficacy)) +
             py::str(")");
        return r;
      });

  py::class_<Dataset, std::shared_ptr<Dataset>>(
      m, "Dataset", py::buffer_protocol(), DOC(navground, sim, Dataset))
      .def(py::init([](py::array b) {
             Dataset ds;
             set_dataset_data_py(ds, b);
             return ds;
           }),
           py::arg("data"), R"doc(
Instantiate a dataset.

:param data: Copies shape, data and dtype from this numpy array
:type data: :py:class:`numpy.ndarray`

)doc")
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
          [](Dataset &ds, py::array b, bool reset) {
            set_dataset_data_py(ds, b, !reset);
          },
          py::arg("values"), py::arg("reset") = false, R"doc(
Append items from a numpy array.

:param values: Append data and dtype from this numpy array
:type values: :py:class:`numpy.ndarray`
:param reset: Wheter to replace the data instead of appending. 
:type reset: bool

)doc")
      .def(
          "append",
          [](Dataset &ds, const Dataset::Data &values) { ds.append(values); },
          py::arg("values"), DOC(navground, sim, Dataset, append))
      .def(
          "push",
          [](Dataset &ds, const Dataset::Scalar &value) { ds.push(value); },
          py::arg("value"), DOC(navground, sim, Dataset, push))
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

Can be set to any object that is convertible to a :py:class:`numpy.dtype`.

)doc");

  py::class_<Probe, PyProbe, std::shared_ptr<Probe>>(m, "Probe",
                                                     DOC(navground, sim, Probe))
      .def(py::init<>(), DOC(navground, sim, Probe, Probe));

  py::class_<RecordProbe, Probe, PyRecordProbe, std::shared_ptr<RecordProbe>>(
      m, "RecordProbe", DOC(navground, sim, RecordProbe))
      .def(py::init<std::shared_ptr<Dataset>>(), py::arg("record") = nullptr,
           DOC(navground, sim, RecordProbe, RecordProbe))
      .def_readonly("data", &RecordProbe::data,
                    DOC(navground, sim, RecordProbe, data));

  py::class_<GroupRecordProbe, Probe, PyGroupRecordProbe,
             std::shared_ptr<GroupRecordProbe>>(
      m, "GroupRecordProbe", DOC(navground, sim, GroupRecordProbe))
      .def(py::init<GroupRecordProbe::Factory>(),
           py::arg("factory") = GroupRecordProbe::default_factory,
           DOC(navground, sim, GroupRecordProbe, GroupRecordProbe))
      .def("get_data", &GroupRecordProbe::get_data, py::arg("key"),
           DOC(navground, sim, GroupRecordProbe, get_data));

  py::class_<ExperimentalRun, std::shared_ptr<ExperimentalRun>>(
      m, "ExperimentalRun", DOC(navground, sim, ExperimentalRun))
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
          [](ExperimentalRun &run, const std::string &name, py::array data) {
            auto ds = run.add_record(name);
            set_dataset_data_py(*ds, data);
            return ds;
          },
          py::arg("name"), py::arg("data"), "TODO")
      .def("add_probe", &ExperimentalRun::add_probe, py::keep_alive<1, 2>(),
           py::arg("probe"), DOC(navground, sim, ExperimentalRun, add_probe))
      .def(
          "add_record_probe",
          [](ExperimentalRun &run, const std::string &name, py::object cls) {
            auto ds = run.add_record(name);
            set_dataset_type_py(*ds, cls.attr("dtype"));
            auto obj = cls.attr("__call__")(ds);
            auto probe = obj.cast<std::shared_ptr<Probe>>();
            run.add_probe(probe);
            return obj;
          },
          py::arg("key"), py::arg("probe_cls"), R"doc(
Adds a record probe.

:param key: the key of the record to be created
:type key: str

:param probe_cls: the probe class
:type key: Type[sim.RecordProbe]

)doc")
      .def(
          "add_group_record_probe",
          [](ExperimentalRun &run, const std::string &name, py::object cls) {
            const auto dtype = cls.attr("dtype");
            const auto factory = [name, dtype,
                                  &run](const std::string &sub_key) {
              auto ds = run.add_record(sub_key, name);
              set_dataset_type_py(*ds, dtype);
              return ds;
            };
            auto obj = cls.attr("__call__")();
            auto probe = obj.cast<std::shared_ptr<GroupRecordProbe>>();
            probe->set_factory(factory);
            run.add_probe(probe);
            return obj;
          },
          py::arg("key"), py::arg("probe_cls"), R"doc(
Adds a group record probe.

:param key: the key of the group to be created
:type key: str

:param probe_cls: the probe class
:type key: Type[sim.GroupRecordProbe]

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
      .def_property(
          "times",
          [](const ExperimentalRun *run) {
            auto record = run->get_times();
            if (record) return as_array(*record);
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
            if (record) return as_array(*record);
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
            if (record) return as_array(*record);
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
            if (record) return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded targets of the agents as a numpy array of shape 
``(simulation steps, number of agents, 3)`` and dtype ``float``::

  [[[x_0, y_0, theta_0], 
    [x_1, y_1, theta_1], 
    ...], 
   ...]
  
The array is empty if targets have not been recorded in the run.
)doc")
      .def_property(
          "commands",
          [](const ExperimentalRun *run) {
            auto record = run->get_cmds();
            if (record) return as_array(*record);
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
          "safety_violations",
          [](const ExperimentalRun *run) {
            auto record = run->get_safety_violations();
            if (record) return as_array(*record);
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
            if (record) return as_array(*record);
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
            auto record = run->get_task_events_for(agent->uid);
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
            if (record) return as_array(*record);
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
            if (record) return as_array(*record);
            return make_empty_array<ng_float_t>();
          },
          nullptr, R"doc(
The recorded agents' efficacy as a numpy array of shape 
``(simulation steps, number of agents)`` and dtype ``float``::

  [[efficacy_0, efficacy_1, ...],
   ...]

The array is empty if efficacy has not been recorded in the run.
)doc")
      .def_property("has_finished", &ExperimentalRun::has_finished, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_has_finished))
      .def_property(
          "recorded_steps", &ExperimentalRun::get_recorded_steps, nullptr,
          DOC(navground, sim, ExperimentalRun, property_recorded_steps))
      .def_property(
          "number_of_agents", &ExperimentalRun::get_number_of_agents, nullptr,
          DOC(navground, sim, ExperimentalRun, property_number_of_agents))
      .def_property("seed", &ExperimentalRun::get_seed, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_seed))
      .def_property("world", &ExperimentalRun::get_world, nullptr,
                    DOC(navground, sim, ExperimentalRun, property_world))
      // .def("update", &ExperimentalRun::update,
      //      DOC(navground, sim, ExperimentalRun, update))
      .def("run", &ExperimentalRun::run,
           DOC(navground, sim, ExperimentalRun, run))
      .def_property("duration", &ExperimentalRun::get_duration_ns, nullptr,
                    DOC(navground, sim, ExperimentalRun, property, duration_ns))
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
                        property_terminate_when_all_idle_or_stuck));

  py::class_<PyExperiment>(m, "Experiment", DOC(navground, sim, Experiment))
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
                     DOC(navground, sim, Experiment, record_config))
      // .def_readwrite("run_config", &Experiment::run_config,
      //                DOC(navground, sim, Experiment, run_config))
      .def_property("runs", &Experiment::get_runs, nullptr,
                    DOC(navground, sim, Experiment, property_runs))
      .def("get_run", [](const Experiment &exp,
                         unsigned index) { return exp.get_runs().at(index); })
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
      .def_property("path", &Experiment::get_path, nullptr,
                    DOC(navground, sim, Experiment, property_path))
      // .def("add_callback", &Experiment::add_callback, py::arg("callback"),
      //      DOC(navground, sim, Experiment, add_callback))
      .def("clear_run_callbacks", &Experiment::clear_run_callbacks,
           DOC(navground, sim, Experiment, clear_run_callbacks))
      .def("add_run_callback", &Experiment::add_run_callback,
           py::arg("callback"), py::arg("at_init") = false,
           py::keep_alive<1, 2>(),
           DOC(navground, sim, Experiment, add_run_callback))
      .def("run_once", &Experiment::run_once, py::arg("seed"),
           py::return_value_policy::reference,
           DOC(navground, sim, Experiment, run_once))
      .def("run", &Experiment::run, py::arg("keep") = true,
           py::arg("number_of_threads") = 1,
           py::arg("start_index") = py::none(),
           py::arg("number_of_runs") = py::none(),
           py::arg("data_path") = py::none(),
           DOC(navground, sim, Experiment, run))
      // .def("init_run", &Experiment::init_run, py::arg("seed"), DOC(navground,
      // sim, Experiment, init_run))
      .def("start", &Experiment::start, py::arg("path") = py::none(),
           DOC(navground, sim, Experiment, start))
      .def("stop", &Experiment::stop, DOC(navground, sim, Experiment, stop))
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
      .def("remove_run", &Experiment::remove_run, py::arg("seed"),
           DOC(navground, sim, Experiment, remove_run))
      .def("add_probe", &PyExperiment::add_probe_py, py::arg("factory"),
           DOC(navground, sim, Experiment, add_probe))
      .def("add_record_probe", &PyExperiment::add_record_probe_py,
           py::arg("key"), py::arg("probe_cls"), R"doc(
Register a probe to record data to during all runs.

:param key: the name associated to the record
:type key: str
:param probe_cls: the class of the probe.
:type probe_cls: Type[sim.RecordProbe]
)doc")
      .def("add_group_record_probe", &PyExperiment::add_group_record_probe_py,
           py::arg("key"), py::arg("probe_cls"),
           R"doc(
Register a probe to record a group of data to during all runs.

:param key: the name associated to the group
:type key: str
:param probe_cls: the class of the probe.
:type probe_cls: Type[sim.GroupRecordProbe]
)doc")
      .def("save", &Experiment::save, py::arg("directory") = py::none(),
           py::arg("path") = py::none(), DOC(navground, sim, Experiment, save))
      .def_property("duration", &Experiment::get_duration_ns, nullptr,
                    DOC(navground, sim, Experiment, property_duration_ns))
      .def_property("begin_time", &Experiment::get_begin_time, nullptr,
                    DOC(navground, sim, Experiment, property_begin_time));

  auto scenario = py::class_<Scenario, PyScenario, HasRegister<Scenario>,
                             HasProperties, std::shared_ptr<Scenario>>(
      m, "Scenario", DOC(navground, sim, Scenario));

  py::class_<Scenario::Group, PyGroup>(scenario, "Group",
                                       DOC(navground, sim, Scenario_Group))
      .def(py::init<>(), "");

  scenario.def(py::init<>())
      .def("init_world", &Scenario::init_world, py::arg("world"),
           py::arg("seed") = py::none(),
           DOC(navground, sim, Scenario, init_world))
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
      // .def_readwrite("groups", &Scenario::groups,
      // py::return_value_policy::reference) .def_property("initializers",
      // &Scenario::get_initializers, nullptr)
      .def("add_init", &Scenario::add_init, py::arg("initializer"),
           DOC(navground, sim, Scenario, add_init))
      .def("set_yaml",
           [](Scenario &scenario, const std::string &value) {
             YAML::Node node = YAML::Load(value);
             YAML::update_scenario(scenario, node);
           })
      .def(py::pickle(
          [](Scenario *scenario) {
            // __getstate__
            return py::make_tuple(YAML::dump_scenario(scenario));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            YAML::Node node = YAML::Load(t[0].cast<std::string>());
            auto obj = YAML::load_scenario(node);
            return obj.cast<std::shared_ptr<Scenario>>();
          }));

  py::class_<SimpleScenario, Scenario, std::shared_ptr<SimpleScenario>>(
      m, "SimpleScenario", DOC(navground, sim, SimpleScenario))
      .def(py::init<>(), DOC(navground, sim, SimpleScenario, SimpleScenario))
      .def(py::pickle(
          [](SimpleScenario *scenario) {
            // __getstate__
            return py::make_tuple(YAML::dump_scenario(scenario));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            YAML::Node node = YAML::Load(t[0].cast<std::string>());
            auto obj = YAML::load_scenario(node);
            return obj.cast<std::shared_ptr<SimpleScenario>>();
          }));

  py::class_<AntipodalScenario, Scenario, std::shared_ptr<AntipodalScenario>>(
      m, "AntipodalScenario", DOC(navground, sim, AntipodalScenario))
      .def(
          py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, bool>(),
          py::arg("radius") = AntipodalScenario::default_radius,
          py::arg("tolerance") = AntipodalScenario::default_tolerance,
          py::arg("position_noise") = AntipodalScenario::default_position_noise,
          py::arg("orientation_noise") =
              AntipodalScenario::default_orientation_noise,
          py::arg("shuffle") = AntipodalScenario::default_shuffle,
          DOC(navground, sim, AntipodalScenario, AntipodalScenario))
      .def_property("radius", &AntipodalScenario::get_radius,
                    &AntipodalScenario::set_radius,
                    DOC(navground, sim, AntipodalScenario, property_radius))
      .def_property("tolerance", &AntipodalScenario::get_tolerance,
                    &AntipodalScenario::set_tolerance,
                    DOC(navground, sim, AntipodalScenario, property_tolerance))
      .def_property(
          "position_noise", &AntipodalScenario::get_position_noise,
          &AntipodalScenario::set_position_noise,
          DOC(navground, sim, AntipodalScenario, property_position_noise))
      .def_property(
          "orientation_noise", &AntipodalScenario::get_orientation_noise,
          &AntipodalScenario::set_orientation_noise,
          DOC(navground, sim, AntipodalScenario, property_orientation_noise))
      .def(py::pickle(
          [](AntipodalScenario *scenario) {
            // __getstate__
            return py::make_tuple(YAML::dump_scenario(scenario));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            YAML::Node node = YAML::Load(t[0].cast<std::string>());
            auto obj = YAML::load_scenario(node);
            return obj.cast<std::shared_ptr<AntipodalScenario>>();
          }));

  py::class_<CrossScenario, Scenario, std::shared_ptr<CrossScenario>>(
      m, "CrossScenario", DOC(navground, sim, CrossScenario, CrossScenario))
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
                    DOC(navground, sim, CrossScenario, property_target_margin))
      .def(py::pickle(
          [](CrossScenario *scenario) {
            // __getstate__
            return py::make_tuple(YAML::dump_scenario(scenario));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            YAML::Node node = YAML::Load(t[0].cast<std::string>());
            auto obj = YAML::load_scenario(node);
            return obj.cast<std::shared_ptr<CrossScenario>>();
          }));

  py::class_<CorridorScenario, Scenario, std::shared_ptr<CorridorScenario>>(
      m, "CorridorScenario", DOC(navground, sim, CorridorScenario))
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, bool>(),
           py::arg("width") = CorridorScenario::default_width,
           py::arg("length") = CorridorScenario::default_length,
           py::arg("agent_margin") = CorridorScenario::default_agent_margin,
           py::arg("add_safety_to_agent_margin") =
               CorridorScenario::default_add_safety_to_agent_margin,
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
      .def(py::pickle(
          [](CorridorScenario *scenario) {
            // __getstate__
            return py::make_tuple(YAML::dump_scenario(scenario));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            YAML::Node node = YAML::Load(t[0].cast<std::string>());
            auto obj = YAML::load_scenario(node);
            return obj.cast<std::shared_ptr<CorridorScenario>>();
          }));

  py::class_<CrossTorusScenario, Scenario, std::shared_ptr<CrossTorusScenario>>(
      m, "CrossTorusScenario", DOC(navground, sim, CrossTorusScenario))
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
                        property_add_safety_to_agent_margin))
      .def(py::pickle(
          [](CrossTorusScenario *scenario) {
            // __getstate__
            return py::make_tuple(YAML::dump_scenario(scenario));
          },
          [](py::tuple t) {
            // __setstate__
            if (t.size() != 1) throw std::runtime_error("Invalid state!");
            YAML::Node node = YAML::Load(t[0].cast<std::string>());
            auto obj = YAML::load_scenario(node);
            return obj.cast<std::shared_ptr<CrossTorusScenario>>();
          }));

  m.def("load_task", &YAML::load_string_py<PyTask>, py::arg("value"),
        R"doc(
Load a task from a YAML string.

:return:
  The loaded task or ``None`` if loading fails.)doc");

  m.def("load_state_estimation", &YAML::load_string_py<PyStateEstimation>,
        py::arg("value"),
        R"doc(
Load a state estimation from a YAML string.

:return:
  The loaded state estimation or ``None`` if loading fails.)doc");
  m.def(
      "load_agent",
      [](const std::string &yaml) {
        YAML::Node node = YAML::Load(yaml);
        return node.as<PyAgent>();
      },
      py::arg("value"),
      R"doc(
Load an agent from a YAML string.

:return:
  The loaded agent or ``None`` if loading fails.)doc");
  m.def(
      "load_world",
      [](const std::string &yaml) -> PyWorld {
        YAML::Node node = YAML::Load(yaml);
        return YAML::load_world(node);
      },
      py::arg("value"),
      R"doc(
Load a world from a YAML string.

:return:
  The loaded world or ``None`` if loading fails.)doc");
  m.def(
      "load_scenario",
      [](const std::string &yaml) {
        YAML::Node node = YAML::Load(yaml);
        return YAML::load_scenario(node);
      },
      py::arg("value"),
      R"doc(
Load a scenario from a YAML string.

:return:
  The loaded scenario or ``None`` if loading fails.)doc");
  m.def(
      "load_experiment",
      [](const std::string &yaml) {
        YAML::Node node = YAML::Load(yaml);
        return node.as<PyExperiment>();
      },
      py::arg("value"),
      R"doc(
Load an experiment from a YAML string.

:return:
  The loaded experiment or ``None`` if loading fails.)doc");

  m.def("dump", &YAML::dump<Task>, py::arg("task"),
        "Dump a task to a YAML-string");
  m.def("dump", &YAML::dump<StateEstimation>, py::arg("state_estimation"),
        "Dump a state_estimation to a YAML-string");
  m.def("dump", &YAML::dump<World>, py::arg("world"),
        "Dump a world to a YAML-string");
  m.def("dump", &YAML::dump_scenario, py::arg("scenario"),
        "Dump a scenario to a YAML-string");
  // m.def("dump", &YAML::dump<Scenario>);
  m.def("dump", &YAML::dump<Agent>, py::arg("agent"),
        "Dump an agent to a YAML-string");
  // m.def("dump", &YAML::dump<PyAgent>, py::arg("agent"),
  //       "Dump an agent to a YAML-string");
  // m.def("dump", &YAML::dump<Experiment>, py::arg("experiment"),
  //       "Dump an experiment to a YAML-string");
  m.def("dump", &YAML::dump<PyExperiment>, py::arg("experiment"),
        "Dump an experiment to a YAML-string");
  m.def("dump", &YAML::dump<Behavior>, py::arg("behavior"),
        "Dump a behavior to a YAML-string");
  m.def("dump", &YAML::dump<Kinematics>, py::arg("kinematics"),
        "Dump a kinematics to a YAML-string");

  // m.def("load_plugins", &load_plugins, py::arg("plugins") = "",
  //       py::arg("env") = "", py::arg("directory") = py::none());
}
