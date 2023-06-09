#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include <vector>

#include "docstrings.h"
#include "navground/core/behavior.h"
#include "navground/core/kinematics.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenario.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/corridor.h"
#include "navground/sim/scenarios/cross.h"
#include "navground/sim/scenarios/cross_torus.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/state_estimations/geometric_bounded.h"
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

template <typename T>
struct get<T, py::object> {
  static T *ptr(const py::object &c) { return c.cast<T *>(); }
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

  void update(Agent *agent, World *world, float time) override {
    PYBIND11_OVERRIDE(void, Task, update, agent, world, time);
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

  void update(Agent *agent, World *world) const override {
    PYBIND11_OVERRIDE(void, StateEstimation, update, agent, world);
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

class PyAgent : public Agent {
 public:
  using B = PyBehavior;
  using K = PyKinematics;
  using T = PyTask;
  using S = PyStateEstimation;

  virtual ~PyAgent() = default;

  using C = py::object;

  PyAgent(float radius = 0.0f, const py::object &behavior = py::none(),
          const py::object &kinematics = py::none(),
          const py::object &task = py::none(),
          const py::object &estimation = py::none(),
          float control_period = 0.0f, unsigned id = 0)
      : Agent(radius, nullptr, nullptr, nullptr, nullptr, control_period, id) {
    set_kinematics(kinematics);
    set_behavior(behavior);
    set_state_estimation(estimation);
    set_task(task);
  }

  static py::object make(float radius = 0.0f,
                         const py::object &behavior = py::none(),
                         const py::object &kinematics = py::none(),
                         const py::object &task = py::none(),
                         const py::object &estimation = py::none(),
                         float control_period = 0.0f, unsigned id = 0) {
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

  void init_world(World *world) override {
    PYBIND11_OVERRIDE(void, Scenario, init_world, world);
  }

  const Properties &get_properties() const override {
    const std::string t = PyHasRegister<Scenario>::get_type();
    if (type_properties().count(t)) {
      return type_properties().at(t);
    }
    return Native::get_properties();
  };
};

struct PyExperiment : public Experiment {
  /* Inherit the constructors */
  using Experiment::Experiment;

  ~PyExperiment() = default;

  // virtual ~PyExperiment() = default;

  py::object py_scenario;

  std::shared_ptr<World> make_world() override {
    return std::make_shared<PyWorld>();
  }

  std::string dump() override { return YAML::dump<PyExperiment>(this); }

  void set_scenario(const py::object &value) {
    py_scenario = value;
    scenario = value.cast<std::shared_ptr<Scenario>>();
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

static py::memoryview trace_view(const Trace *trace, const float *data) {
  const std::array<ssize_t, 3> shape{trace->steps, trace->number, 3};
  const std::array<ssize_t, 3> strides{
      static_cast<ssize_t>(sizeof(float) * 3 * trace->number),
      3 * sizeof(float), sizeof(float)};
  return py::memoryview::from_buffer(data, shape, strides);
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
      .def(py::init<Vector2, float>(), py::arg("position"), py::arg("radius"),
           DOC(navground, sim, Obstacle, Obstacle))
      .def(py::init<Disc>(), py::arg("disc"),
           DOC(navground, sim, Obstacle, Obstacle, 3))
      .def_readwrite("disc", &Obstacle::disc,
                     DOC(navground, sim, Obstacle, disc));

  py::class_<BoundingBox>(m, "BoundingBox", "A rectangular region")
      .def(py::init<float, float, float, float>(), py::arg("min_x"),
           py::arg("max_x"), py::arg("min_y"), py::arg("max_x"),
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
          [](Agent *agent, const float value) {
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
          [](Agent *agent, const float value) {
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
                    DOC(navground, sim, Agent, idle));

  py::class_<PyAgent, Agent, Entity, std::shared_ptr<PyAgent>>(
      m, "Agent", py::dynamic_attr(), DOC(navground, sim, Agent))
      .def(py::init<float, const py::object &, const py::object &,
                    const py::object &, const py::object &, float, unsigned>(),
           py::arg("radius") = 0.0f, py::arg("behavior") = py::none(),
           py::arg("kinematics") = py::none(), py::arg("task") = py::none(),
           py::arg("state_estimation") = py::none(),
           py::arg("control_period") = 0.0f, py::arg("id") = 0,
           DOC(navground, sim, Agent, Agent))
#if 0
      .def(py::init<float, std::shared_ptr<Behavior>,
                    std::shared_ptr<Kinematics>, std::shared_ptr<Task>,
                    std::shared_ptr<StateEstimation>, float, unsigned>(),
           py::arg("radius") = 0.0f, py::arg("behavior") = nullptr,
           py::arg("kinematics") = nullptr, py::arg("task") = nullptr,
           py::arg("state_estimation") = nullptr,
           py::arg("control_period") = 0.0f, py::arg("id") = 0)
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
           DOC(navground, sim, World, add_callback))
      .def("reset_callbacks", &World::reset_callbacks,
           DOC(navground, sim, World, reset_callbacks))
      .def("update", &World::update, py::arg("time_step"),
           DOC(navground, sim, World, update))
      .def("run", &World::run, py::arg("steps"), py::arg("time_step"),
           DOC(navground, sim, World, run))
      .def("run_until", &World::run_until, py::arg("condition"),
           py::arg("time_step"), DOC(navground, sim, World, run_until))
      .def_property("time", &World::get_time, nullptr,
                    DOC(navground, sim, World, property_time))
      .def_property("step", &World::get_step, nullptr,
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
      .def_property("collisions", &World::get_collisions, nullptr,
                    DOC(navground, sim, World, property_collisions))
      .def("compute_safety_violation", &World::compute_safety_violation,
           py::arg("agent"),
           DOC(navground, sim, World, compute_safety_violation))
      .def("agents_are_idle", &World::agents_are_idle,
           DOC(navground, sim, World, agents_are_idle))
      .def("space_agents_apart", &World::space_agents_apart,
           py::arg("minimal_distance") = 0.0f,
           py::arg("with_safety_margin") = false,
           py::arg("max_iterations") = 10,
           DOC(navground, sim, World, space_agents_apart))
      .def_static("set_seed", &World::set_seed, py::arg("seed"),
                  DOC(navground, sim, World, set_seed))
      .def("get_entity", &World::get_entity, py::arg("uid"),
           py::return_value_policy::reference,
           DOC(navground, sim, World, get_entity))
      .def("_prepare", &World::prepare)
      .def("in_collision", &World::in_collision, py::arg("e1"), py::arg("e2"),
           DOC(navground, sim, World, in_collision));

  py::class_<PyWorld, World, std::shared_ptr<PyWorld>>(
      m, "World", DOC(navground, sim, World))
      .def(py::init<>(), DOC(navground, sim, World, World))
      .def("add_agent", &PyWorld::add_agent,
           DOC(navground, sim, World, add_agent));

  py::class_<StateEstimation, PyStateEstimation, HasRegister<StateEstimation>,
             HasProperties, std::shared_ptr<StateEstimation>>(
      m, "StateEstimation", DOC(navground, sim, StateEstimation))
      .def(py::init<>(), DOC(navground, sim, StateEstimation, StateEstimation))
      // .def_readwrite("world", &StateEstimation::world,
      //                py::return_value_policy::reference,
      //                DOC(navground, sim, StateEstimation, world))
      // .def("_update", &StateEstimation::update)
      // .def("_prepare", &StateEstimation::prepare);
      .def_property(
          "type", [](StateEstimation *obj) { return obj->get_type(); }, nullptr,
          "The name associated to the type of an object");

  py::class_<BoundedStateEstimation, StateEstimation,
             std::shared_ptr<BoundedStateEstimation>>(
      m, "BoundedStateEstimation", DOC(navground, sim, BoundedStateEstimation))
      .def(py::init<float>(),
           // py::arg("field_of_view") = 0.0,
           py::arg("range_of_view") = 0.0,
           DOC(navground, sim, BoundedStateEstimation, BoundedStateEstimation))
      // .def_property("field_of_view",
      // &BoundedStateEstimation::get_field_of_view,
      //               &BoundedStateEstimation::set_field_of_view)
      .def_property(
          "range_of_view", &BoundedStateEstimation::get_range_of_view,
          &BoundedStateEstimation::set_range_of_view,
          DOC(navground, sim, BoundedStateEstimation, property_range_of_view))
      .def("_neighbors_of_agent", &BoundedStateEstimation::neighbors_of_agent,
           py::arg("agent"), py::arg("world"),
           DOC(navground, sim, BoundedStateEstimation, neighbors_of_agent));

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
      .def(py::init<Waypoints, bool, float>(),
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

  py::class_<Trace>(m, "Trace", DOC(navground, sim, Trace))
      .def("index_of_agent", &Trace::index_of_agent, py::arg("agent"),
           DOC(navground, sim, Trace, index_of_agent))
      .def_property(
          "poses",
          [](const Trace *trace) {
            if (trace->record_pose) {
              return trace_view(trace, trace->pose_data.data());
            }
            return empty_float_view();
          },
          nullptr, R"doc(
The recorded poses of the agents
as a memory view to the floating point buffer::

  [[[agent_0_x, agent_0_y, agent_0_theta], 
    [agent_1_x, agent_1_y, agent_1_theta], 
    ...], 
   ...]
  

of shape ``(simulation steps, number of agents, 3)``.

The view is empty if poses have 
not been recorded in the trace.
)doc")
      .def_property(
          "twists",
          [](const Trace *trace) {
            if (trace->record_twist) {
              return trace_view(trace, trace->twist_data.data());
            }
            return empty_float_view();
          },
          nullptr, R"doc(
The recorded twist of the agents
as a memory view to the floating point buffer::

  [[[agent_0_x, agent_0_y, agent_0_theta], 
    [agent_1_x, agent_1_y, agent_1_theta], 
    ...], 
   ...]
  

of shape ``(simulation steps, number of agents, 3)``.

The view is empty if twist have 
not been recorded in the trace.
)doc")
      .def_property(
          "targets",
          [](const Trace *trace) {
            if (trace->record_target) {
              return trace_view(trace, trace->target_data.data());
            }
            return py::memoryview::from_buffer<float>(trace->target_data.data(),
                                                      {0}, {0});
          },
          nullptr, R"doc(
The recorded targets of the agents
as a memory view to the floating point buffer::

  [[[agent_0_x, agent_0_y, agent_0_theta], 
    [agent_1_x, agent_1_y, agent_1_theta], 
    ...], 
   ...]
  
of shape ``(simulation steps, number of agents, 3)``.

The view is empty if targets have 
not been recorded in the trace.
)doc")
      .def_property(
          "commands",
          [](const Trace *trace) {
            if (trace->record_cmd) {
              return trace_view(trace, trace->cmd_data.data());
            }
            return empty_float_view();
          },
          nullptr, R"doc(
The recorded commands of the agents
as a memory view to the floating point buffer::
    
  [[[agent_0_x, agent_0_y, agent_0_theta], 
   [agent_1_x, agent_1_y, agent_1_theta], 
    ...], 
   ...]

of shape ``(simulation steps, number of agents, 3)``.

The view is empty if commands have 
not been recorded in the trace.
)doc")
      .def_property(
          "safety_violations",
          [](const Trace *trace) {
            if (trace->record_safety_violation) {
              const std::array<ssize_t, 2> shape{trace->steps, trace->number};
              const std::array<ssize_t, 2> strides{
                  static_cast<ssize_t>(sizeof(float) * trace->number),
                  sizeof(float)};
              return py::memoryview::from_buffer(
                  trace->safety_violation_data.data(), shape, strides);
            }
            return empty_float_view();
          },
          nullptr, R"doc(
The recorded amounts of safety violation")
as a memory view to the floating point buffer::

  [[agent_0_violation, agent_1_violation, ...],
   ...]

of shape ``(simulation steps, number of agents)`` and
where a value of 0 represents no violations.

The view is empty if safety violations have 
not been recorded in the trace.
)doc")
      .def_property(
          "collisions",
          [](const Trace *trace) {
            if (trace->record_collisions) {
              const ssize_t n = trace->collisions_data.size() / 3;
              const std::array<ssize_t, 2> shape{n, 3};
              const std::array<ssize_t, 2> strides{sizeof(float) * 3,
                                                   sizeof(float)};
              return py::memoryview::from_buffer(trace->collisions_data.data(),
                                                 shape, strides);
            }
            return empty_unsigned_view();
          },
          nullptr, R"doc(
The recorded collisions between pairs of entities as
a memory view to the uint32 buffer::

  [[time_step, entity 1 uid, entity 1 uid], 
   ...]

of shape ``(number of collisions, 3)``.

The view is empty if collisions have 
not been recorded in the trace.
)doc")
      .def(
          "get_task_events",
          [](const Trace *trace, const Agent *agent) {
            const auto index = trace->index_of_agent(agent);
            if (index && trace->record_task_events) {
              const auto i = *index;
              const ssize_t n = trace->task_events[i];
              const auto &data = trace->task_events_data[i];
              const ssize_t m = n ? data.size() / n : 0;
              const std::array<ssize_t, 2> shape{n, m};
              const std::array<ssize_t, 2> strides{
                  static_cast<ssize_t>(sizeof(float) * m), sizeof(float)};
              return py::memoryview::from_buffer(data.data(), shape, strides);
            }
            return empty_float_view();
          },
          R"doc(
The recorded events logged by the task of an agent 
as memory view to the floating point buffer::

  [[data_0, ...], 
   ...]

of shape ``(number events, size of event log)``.

The view is empty if the agent's task has not been recorded in the trace.

:param agent: The agent

:return: The events logged by the agent task
)doc")
      .def_readwrite("record_pose", &Trace::record_pose,
                     DOC(navground, sim, Trace, record_pose))
      .def_readwrite("record_twist", &Trace::record_twist,
                     DOC(navground, sim, Trace, record_twist))
      .def_readwrite("record_cmd", &Trace::record_cmd,
                     DOC(navground, sim, Trace, record_cmd))
      .def_readwrite("record_target", &Trace::record_target,
                     DOC(navground, sim, Trace, record_target))
      .def_readwrite("record_safety_violation", &Trace::record_safety_violation,
                     DOC(navground, sim, Trace, record_safety_violation))
      .def_readwrite("record_collisions", &Trace::record_collisions,
                     DOC(navground, sim, Trace, record_collisions))
      .def_readwrite("record_task_events", &Trace::record_task_events,
                     DOC(navground, sim, Trace, record_task_events))
      .def_readonly("number", &Trace::number,
                    DOC(navground, sim, Trace, number))
      .def_readonly("steps", &Trace::steps, DOC(navground, sim, Trace, steps));

  py::class_<PyExperiment>(m, "Experiment", DOC(navground, sim, Experiment))
      .def(py::init<float, int>(), py::arg("time_step") = 0.1,
           py::arg("steps") = 1000, DOC(navground, sim, Experiment, Experiment))
      .def_readwrite("time_step", &Experiment::time_step,
                     DOC(navground, sim, Experiment, time_step))
      .def_readwrite("steps", &Experiment::steps,
                     DOC(navground, sim, Experiment, steps))
      .def_readonly("trace", &Experiment::trace,
                    DOC(navground, sim, Experiment, trace))
      .def_property(
          "scenario", [](const PyExperiment *exp) { return exp->scenario; },
          &PyExperiment::set_scenario,
          DOC(navground, sim, Experiment, scenario))
      .def_property("world", &Experiment::get_world, nullptr,
                    DOC(navground, sim, Experiment, world))
      .def_readwrite("runs", &Experiment::runs,
                     DOC(navground, sim, Experiment, runs))
      .def_readwrite("save_directory", &Experiment::save_directory,
                     DOC(navground, sim, Experiment, save_directory))
      .def_readwrite("name", &Experiment::name,
                     DOC(navground, sim, Experiment, name))
      .def_property("path", &Experiment::get_path, nullptr,
                    DOC(navground, sim, Experiment, property_path))
      .def("add_callback", &Experiment::add_callback, py::arg("callback"),
           DOC(navground, sim, Experiment, add_callback))
      .def("run_once", &Experiment::run_once, py::arg("seed"),
           DOC(navground, sim, Experiment, run_once))
      .def("run", &Experiment::run, DOC(navground, sim, Experiment, run))
      .def_property("run_duration", &Experiment::get_run_duration_ns, nullptr,
                    DOC(navground, sim, Experiment, property, run_duration_ns))
      .def_property("duration", &Experiment::get_duration_ns, nullptr,
                    DOC(navground, sim, Experiment, property, duration_ns))
      .def_property("begin_time", &Experiment::get_begin_time, nullptr,
                    DOC(navground, sim, Experiment, property, begin_time));

  auto scenario = py::class_<Scenario, PyScenario, HasRegister<Scenario>,
                             HasProperties, std::shared_ptr<Scenario>>(
      m, "Scenario", DOC(navground, sim, Scenario));

  py::class_<Scenario::Group, PyGroup>(scenario, "Group",
                                       DOC(navground, sim, Scenario_Group))
      .def(py::init<>(), "");

  scenario.def(py::init<>())
      .def("init_world", &Scenario::init_world,
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
      .def("add_init", &Scenario::add_init,
           DOC(navground, sim, Scenario, add_init));

  py::class_<SimpleScenario, Scenario, std::shared_ptr<SimpleScenario>>(
      m, "SimpleScenario", DOC(navground, sim, SimpleScenario))
      .def(py::init<>(), DOC(navground, sim, SimpleScenario, SimpleScenario));

  py::class_<AntipodalScenario, Scenario, std::shared_ptr<AntipodalScenario>>(
      m, "AntipodalScenario", DOC(navground, sim, AntipodalScenario))
      .def(
          py::init<float, float, float, float, bool>(),
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
          DOC(navground, sim, AntipodalScenario, property_orientation_noise));

  py::class_<CrossScenario, Scenario, std::shared_ptr<CrossScenario>>(
      m, "CrossScenario", DOC(navground, sim, CrossScenario, CrossScenario))
      .def(py::init<float, float, float, bool, float>(),
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

  py::class_<CorridorScenario, Scenario, std::shared_ptr<CorridorScenario>>(
      m, "CorridorScenario", DOC(navground, sim, CorridorScenario))
      .def(py::init<float, float, float, bool>(),
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
                        property_add_safety_to_agent_margin));

  py::class_<CrossTorusScenario, Scenario, std::shared_ptr<CrossTorusScenario>>(
      m, "CrossTorusScenario", DOC(navground, sim, CrossTorusScenario))
      .def(py::init<float, float, bool>(),
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
}
