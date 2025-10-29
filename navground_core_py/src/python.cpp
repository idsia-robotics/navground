#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/stl_bind.h>

#include <vector>

#include "docstrings.h"
#include "navground/core/attribute.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_group.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/behavior_modulations/limit_acceleration.h"
#include "navground/core/behavior_modulations/limit_twist.h"
#include "navground/core/behavior_modulations/motor_pid.h"
#include "navground/core/behavior_modulations/relaxation.h"
#include "navground/core/behaviors/HL.h"
#include "navground/core/behaviors/HRVO.h"
#include "navground/core/behaviors/ORCA.h"
#include "navground/core/behaviors/dummy.h"
#include "navground/core/build_info.h"
#include "navground/core/cached_collision_computation.h"
#include "navground/core/collision_computation.h"
#include "navground/core/common.h"
#include "navground/core/controller.h"
#include "navground/core/kinematics.h"
#include "navground/core/plugins.h"
#include "navground/core/states/gridmap.h"
#include "navground/core/states/sensing.h"
#include "navground/core/types.h"
#include "navground/core/version.h"
#include "navground/core/yaml/core.h"
#include "navground/core/yaml/schema.h"
#include "navground/core/yaml/schema_core.h"
#include "navground/core/yaml/yaml.h"
#include "navground/core_py/behavior_modulation.h"
#include "navground/core_py/buffer.h"
#include "navground/core_py/pickle.h"
#include "navground/core_py/property.h"
#include "navground/core_py/register.h"
#include "navground/core_py/version.h"
#include "navground/core_py/yaml.h"

using namespace navground::core;
namespace py = pybind11;

static navground::core::BuildDependencies build_dependencies_core_py() {
  return {{"core",
           {navground::core::build_info(), navground::core::get_build_info()}}};
}

PYBIND11_MAKE_OPAQUE(std::map<std::string, Buffer>)

template <typename T> static std::string to_string(const T &value) {
  return std::to_string(value);
}

// template <> std::string to_string(const BuildInfo &bi) {
//   return "<version: " + bi.get_version_string() +
//          ", git_commit: " + bi.git_commit + ", date: " + bi.date +
//          ", floating_point_type: " + bi.floating_point_type + ">";
// }

template <> std::string to_string(const Vector2 &value) {
  return "(" + std::to_string(value[0]) + ", " + std::to_string(value[1]) + ")";
}

// template <>
// std::string to_string(const bool &value) {
//   return value ? "True" : "False";
// }

template <> std::string to_string(const Frame &frame) {
  return frame == Frame::relative ? "Frame.relative" : "Frame.absolute";
}

template <> std::string to_string(const Pose2 &value) {
  return "Pose2(" + to_string(value.position) + ", " +
         std::to_string(value.orientation) + ")";
}

template <> std::string to_string(const Twist2 &value) {
  return "Twist2(" + to_string(value.velocity) + ", " +
         std::to_string(value.angular_speed) +
         ", frame=" + to_string(value.frame) + ")";
}

template <> std::string to_string(const Path &value) {
  std::string s = "Path(length=" + to_string(value.length);
  if (value.loop) {
    s += ", loop=True";
  }
  s += ")";
  return s;
}

template <> std::string to_string(const Target &value) {
  std::string r = "Target(";
  bool first = true;
  if (value.position) {
    if (!first)
      r += ", ";
    r += "position=" + to_string(*value.position);
    first = false;
  }
  if (value.orientation) {
    if (!first)
      r += ", ";
    r += "orientation=" + to_string(*value.orientation);
    first = false;
  }
  if (value.direction) {
    if (!first)
      r += ", ";
    r += "direction=" + to_string(*value.direction);
    first = false;
  }
  if (value.angular_direction) {
    if (!first)
      r += ", ";
    r += "angular_direction=" + to_string(*value.angular_direction);
    first = false;
  }
  if (value.speed) {
    if (!first)
      r += ", ";
    r += "speed=" + to_string(*value.speed);
    first = false;
  }
  if (value.angular_speed) {
    if (!first)
      r += ", ";
    r += "angular_speed=" + to_string(*value.angular_speed);
    first = false;
  }
  if (value.position_tolerance) {
    if (!first)
      r += ", ";
    r += "position_tolerance=" + to_string(value.position_tolerance);
    first = false;
  }
  if (value.orientation_tolerance) {
    if (!first)
      r += ", ";
    r += "orientation_tolerance=" + to_string(value.orientation_tolerance);
    first = false;
  }
  if (value.path) {
    if (!first)
      r += ", ";
    r += "path=" + to_string(*value.path);
    first = false;
  }
  r += ")";
  return r;
}

template <> std::string to_string(const Disc &value) {
  return "Disc(" + to_string(value.position) + ", " +
         std::to_string(value.radius) + ")";
}

template <> std::string to_string(const Neighbor &value) {
  return "Neighbor(" + to_string<Disc>(value) + ", " +
         to_string(value.velocity) + ", " + std::to_string(value.id) + ")";
}

template <> std::string to_string(const LineSegment &value) {
  return "LineSegment(" + to_string(value.p1) + ", " + to_string(value.p2) +
         ")";
}

template <> std::string to_string(const Property &value) {
  std::string r = "<Property: " + value.type_name;
  if (value.readonly) {
    r += " (readonly)";
  }
  r += ">";
  return r;
}

struct PyBehaviorModulation : public BehaviorModulation,
                              public PyHasRegister<BehaviorModulation> {
  /* Inherit the constructors */
  using BehaviorModulation::BehaviorModulation;
  using Native = BehaviorModulation;
  using PyHasRegister<BehaviorModulation>::make_type;

  /* Trampolines (need one for each virtual function) */
  void pre(Behavior &behavior, ng_float_t time_step) override {
    PYBIND11_OVERRIDE(void, BehaviorModulation, pre, behavior, time_step);
  }
  Twist2 post(Behavior &behavior, ng_float_t time_step,
              const Twist2 &cmd) override {
    PYBIND11_OVERRIDE(Twist2, BehaviorModulation, post, behavior, time_step,
                      cmd);
  }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE

  // HACK(J): should not happen but as of now, it can be that get_type returns
  // ''
  // const Properties &get_properties() const override {
  //   const std::string t = get_type();
  //   if (type_properties().count(t)) {
  //     return type_properties().at(t);
  //   }
  //   return Native::get_properties();
  // }
};

struct PyBehavior : public Behavior, public PyHasRegister<Behavior> {
public:
  /* Inherit the constructors */
  using Behavior::Behavior;
  using Native = Behavior;
  using PyHasRegister<Behavior>::make_type;

  /* Trampolines (need one for each virtual function) */

  void prepare() override { PYBIND11_OVERRIDE(void, Behavior, prepare); }

  void close() override { PYBIND11_OVERRIDE(void, Behavior, close); }

  Twist2 compute_cmd_internal(ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, compute_cmd_internal, time_step);
  }
  Vector2 desired_velocity_towards_point(const Vector2 &point, ng_float_t speed,
                                         ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Vector2, Behavior, desired_velocity_towards_point, point,
                      speed, time_step);
  }
  Vector2 desired_velocity_towards_velocity(const Vector2 &velocity,
                                            ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Vector2, Behavior, desired_velocity_towards_velocity,
                      velocity, time_step);
  }
  Twist2 twist_towards_velocity(const Vector2 &absolute_velocity) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, twist_towards_velocity,
                      absolute_velocity);
  }
  Twist2 cmd_twist_along_path(Path &path, ng_float_t speed,
                              ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_along_path, path, speed,
                      time_step);
  }
  Twist2 cmd_twist_towards_pose(const Pose2 &pose, ng_float_t speed,
                                ng_float_t angular_speed,
                                ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_pose, pose, speed,
                      angular_speed, time_step);
  }
  Twist2 cmd_twist_towards_point(const Vector2 &point, ng_float_t speed,
                                 ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_point, point, speed,
                      time_step);
  }
  Twist2 cmd_twist_towards_velocity(const Vector2 &velocity,
                                    ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_velocity, velocity,
                      time_step);
  }
  Twist2 cmd_twist_towards_orientation(ng_float_t orientation,
                                       ng_float_t angular_speed,
                                       ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_orientation,
                      orientation, angular_speed, time_step);
  }
  Twist2 cmd_twist_towards_angular_speed(ng_float_t angular_speed,
                                         ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_angular_speed,
                      angular_speed, time_step);
  }
  Twist2 cmd_twist_towards_stopping(ng_float_t time_step) override {
    PYBIND11_OVERRIDE(Twist2, Behavior, cmd_twist_towards_stopping, time_step);
  }

  EnvironmentState *get_environment_state() override {
    PYBIND11_OVERRIDE(EnvironmentState *, Behavior, get_environment_state);
  }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE

  // HACK(J): should not happen but as of now, it can be that get_type returns
  // ''
  // const Properties &get_properties() const override {
  //   const std::string t = get_type();
  //   if (type_properties().count(t)) {
  //     return type_properties().at(t);
  //   }
  //   return Native::get_properties();
  // };

  py::object py_kinematics;
  // std::vector<py::object> py_modulations;
  // py::list py_modulations;

  void set_kinematics_py(const py::object &value) {
    py_kinematics = value;
    set_kinematics(value.cast<std::shared_ptr<Kinematics>>());
  }
};

struct PyKinematics : public Kinematics, public PyHasRegister<Kinematics> {
  /* Inherit the constructors */
  using Kinematics::Kinematics;
  using Native = Kinematics;
  using PyHasRegister<Kinematics>::make_type;

  /* Trampolines (need one for each virtual function) */
  Twist2 feasible(const Twist2 &twist) const override {
    PYBIND11_OVERRIDE_PURE(Twist2, Kinematics, feasible, twist);
  }
  Twist2 feasible_from_current(const Twist2 &twist, const Twist2 &current,
                               ng_float_t time_step) const override {
    PYBIND11_OVERRIDE(Twist2, Kinematics, feasible_from_current, twist, current,
                      time_step);
  }
  bool is_wheeled() const override {
    PYBIND11_OVERRIDE(bool, Kinematics, is_wheeled);
  }
  unsigned dof() const override {
    PYBIND11_OVERRIDE_PURE(unsigned, Kinematics, dof);
  }
  ng_float_t get_max_angular_speed() const override {
    PYBIND11_OVERRIDE(ng_float_t, Kinematics, get_max_angular_speed);
  }
  ng_float_t get_max_speed() const override {
    PYBIND11_OVERRIDE(ng_float_t, Kinematics, get_max_angular_speed);
  }

  OVERRIDE_DECODE
  OVERRIDE_ENCODE

  // HACK(J): should not happen but as of now, it can be that get_type returns
  // ''
  // const Properties &get_properties() const override {
  //   const std::string t = get_type();
  //   if (type_properties().count(t)) {
  //     return type_properties().at(t);
  //   }
  //   return Native::get_properties();
  // };
};

struct PyBehaviorGroup : public BehaviorGroup {
public:
  /* Inherit the constructors */
  using BehaviorGroup::BehaviorGroup;
  /* Trampolines (need one for each virtual function) */
  std::vector<Twist2> compute_cmds(ng_float_t time_step) override {
    PYBIND11_OVERRIDE_PURE(std::vector<Twist2>, BehaviorGroup, compute_cmds,
                           time_step);
  }
};

struct PyBehaviorGroupMember : public BehaviorGroupMember {
public:
  /* Inherit the constructors */
  using BehaviorGroupMember::BehaviorGroupMember;
  using BehaviorGroupMember::Groups;
  /* Trampolines (need one for each virtual function) */

  size_t get_group_hash() const override {
    PYBIND11_OVERRIDE(size_t, BehaviorGroupMember, get_group_hash);
  }

  void prepare() override {
    PYBIND11_OVERRIDE(void, BehaviorGroupMember, prepare);
  }

  void close() override { PYBIND11_OVERRIDE(void, BehaviorGroupMember, close); }

  EnvironmentState *get_environment_state() override {
    PYBIND11_OVERRIDE(EnvironmentState *, BehaviorGroupMember,
                      get_environment_state);
  }

  // OVERRIDE_DECODE
  // OVERRIDE_ENCODE

  std::shared_ptr<BehaviorGroup> make_group() const override { return nullptr; }

  py::dict get_groups_py() const {
    const auto get_groups_fn = py::get_override(this, "get_groups");
    return py::cast<py::dict>(get_groups_fn());
  }

  py::object make_group_py() const {
    const auto make_group_fn = py::get_override(this, "make_group");
    return make_group_fn();
  }

  std::shared_ptr<BehaviorGroup> get_or_create_group(size_t key) override {
    py::dict groups = get_groups_py();
    if (!groups.contains(key)) {
      py::object group = make_group_py();
      groups[py::cast(key)] = group;
    }
    auto py_group = groups[py::cast(key)];
    return py::cast<std::shared_ptr<BehaviorGroup>>(py_group);
  }

  void remove_group(size_t key) override {
    py::dict groups = get_groups_py();
    groups.attr("pop")(py::cast(key));
  }
};

namespace YAML {
template <> py::object load_node_py<PyBehavior>(const Node &node) {
  auto obj = make_type_from_yaml_py<PyBehavior>(node);
  if (!obj.is_none()) {
    for (auto item : node["modulations"]) {
      auto value = load_node_py<PyBehaviorModulation>(item);
      add_modulation_py(obj, value);
    }
    Node n = node;
    n.remove("modulations");
    convert<Behavior>::decode(n, obj.cast<Behavior &>());
  }
  return obj;
}
} // namespace YAML

PYBIND11_MODULE(_navground, m) {
  py::options options;
  // options.disable_function_signatures();

#if PYBIND11_VERSION_MAJOR >= 2 && PYBIND11_VERSION_MINOR >= 10
  options.disable_enum_members_docstring();
#endif

  py::class_<BuildInfo>(m, "BuildInfo", DOC(navground, core, BuildInfo))
      .def_readonly("version", &BuildInfo::version,
                    DOC(navground, core, BuildInfo, version))
      .def_readonly("git", &BuildInfo::git,
                    DOC(navground, core, BuildInfo, git))
      .def_readonly("date", &BuildInfo::date,
                    DOC(navground, core, BuildInfo, date))
      .def_readonly("floating_point_type", &BuildInfo::floating_point_type,
                    DOC(navground, core, BuildInfo, floating_point_type))
      .def_property("version_string", &BuildInfo::get_version_string, nullptr,
                    DOC(navground, core, BuildInfo, property, version_string))
      .def_property("date_string", &BuildInfo::get_date_string, nullptr,
                    DOC(navground, core, BuildInfo, property, date_string))
      .def("to_string_diff", &BuildInfo::to_string_diff, py::arg("other"),
           DOC(navground, core, BuildInfo, to_string_diff))
      .def("__repr__", &BuildInfo::to_string);

  py::class_<DependencyInfo>(m, "DependencyInfo",
                             DOC(navground, core, DependencyInfo))
      .def_readonly("build", &DependencyInfo::build,
                    DOC(navground, core, DependencyInfo, build))
      .def_readonly("run", &DependencyInfo::run,
                    DOC(navground, core, DependencyInfo, run))
      .def("__repr__", &DependencyInfo::to_string);

  py::class_<Property>(m, "Property", DOC(navground, core, Property))
      .def(py::init(&make_property_py_with_type), py::arg("getter"),
           py::arg("setter"), py::arg("default"), py::arg("type_name"),
           py::arg("description") = "", py::arg("schema") = nullptr,
           py::arg("deprecated_names") = std::vector<std::string>{}, R"doc(
Constructs a new instance.

:param getter: The getter
:type getter: :py:class:`typing.Callable[[], T]`
:param setter: An optional setter
:type setter: :py:class:`typing.Callable[[T], None]` | None
:param default: The default value (should be convertible to the property type)
:type default: :py:type:`navground.core.PropertyField`
:param type_name: The property type name
:type type_name: str
:param description: Optional description
:type description: str
:param schema: Optional schema modifier
:type schema: :py:class:`typing.Callable[[dict[str, typing.Any]], None]`
:param deprecated_names: A list of deprecated names for this property
:type deprecated_names: list[str]
           )doc")
      .def_readonly("description", &Property::description,
                    DOC(navground, core, Property, description))
      .def_readonly("deprecated_names", &Property::deprecated_names,
                    DOC(navground, core, Property, deprecated_names))
      .def_readonly("owner_type_name", &Property::owner_type_name,
                    DOC(navground, core, Property, owner_type_name))
      .def_readonly("default_value", &Property::default_value,
                    DOC(navground, core, Property, default_value))
      .def_readonly("type_name", &Property::type_name,
                    DOC(navground, core, Property, type_name))
      .def_readonly("readonly", &Property::readonly,
                    DOC(navground, core, Property, readonly))
      .def("__repr__", &to_string<Property>)
      .def_static("make_prototype", &Property::make_prototype,
                  py::arg("type_name"),
                  DOC(navground, core, Property, make_prototype))
      .def_static("make", &make_property_with_py_property_with_type,
                  py::arg("property"), py::arg("default"), py::arg("type_name"),
                  py::arg("description") = "", py::arg("schema") = nullptr,
                  py::arg("deprecated_names") = std::vector<std::string>{},
                  R"doc(
Constructs a navground property from a Python property.

:param property: The Python property
:type property: :py:class:`property`
:param default: The default value (should be convertible to the property type)
:type default: :py:type:`navground.core.PropertyField`
:param type_name: The property type name
:type type_name: str
:param description: Optional description
:type description: str
:param schema: Optional schema modifier
:type schema: :py:class:`typing.Callable[[dict[str, typing.Any]], None]`
:param deprecated_names: A list of deprecated names for this property
:type deprecated_names: list[str]
           )doc");

  py::class_<HasProperties, std::shared_ptr<HasProperties>>(
      m, "HasProperties", DOC(navground, core, HasProperties))
      .def("get", &HasProperties::get, py::arg("name"),
           DOC(navground, core, HasProperties, get))
      .def("set", &HasProperties::set, py::arg("name"), py::arg("value"),
           DOC(navground, core, HasProperties, set))
      .def("has", &HasProperties::has, py::arg("name"),
           DOC(navground, core, HasProperties, has))
      .def("get_property_type_name", &HasProperties::get_property_type_name,
           py::arg("name"),
           DOC(navground, core, HasProperties, get_property_type_name));
  // .def_property("properties", &HasProperties::get_properties, nullptr,
  //               DOC(navground, core, HasProperties, property_properties));

  declare_register<Behavior>(m, "Behavior");
  declare_register<Kinematics>(m, "Kinematics");
  declare_register<BehaviorModulation>(m, "BehaviorModulation");

  py::enum_<Frame>(m, "Frame", DOC(navground, core, Frame))
      .value("relative", Frame::relative, DOC(navground, core, Frame, relative))
      .value("absolute", Frame::absolute,
             DOC(navground, core, Frame, absolute));

  auto twist = py::class_<Twist2>(m, "Twist2", DOC(navground, core, Twist2));

  py::class_<Pose2>(m, "Pose2", DOC(navground, core, Pose2))
      .def(py::init<Vector2, ng_float_t>(),
           py::arg("position") = Vector2::Zero(), py::arg("orientation") = 0,
           DOC(navground, core, Pose2, Pose2))
      .def_readwrite("position", &Pose2::position,
                     DOC(navground, core, Pose2, position))
      .def_readwrite("orientation", &Pose2::orientation,
                     DOC(navground, core, Pose2, orientation))
      .def("rotate", &Pose2::rotate, py::arg("angle"),
           DOC(navground, core, Pose2, rotate))
      .def("integrate", &Pose2::integrate, py::arg("twist"),
           py::arg("time_step"), DOC(navground, core, Pose2, rotate))
      .def("absolute", &Pose2::absolute, py::arg("reference"),
           DOC(navground, core, Pose2, absolute))
      .def("relative", &Pose2::relative, py::arg("reference"),
           DOC(navground, core, Pose2, relative))
      .def(py::self * py::self)
      .def(py::self / py::self)
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("inverse", &Pose2::inverse, DOC(navground, core, Pose2, inverse))
      .def("transform_pose", &Pose2::transform_pose, py::arg("pose"),
           DOC(navground, core, Pose2, transform_pose))
      .def("transform_point", &Pose2::transform_point, py::arg("point"),
           DOC(navground, core, Pose2, transform_point))
      .def("transform_vector", &Pose2::transform_vector, py::arg("vector"),
           DOC(navground, core, Pose2, transform_vector))
      .def("get_transformation_in_frame", &Pose2::get_transformation_in_frame,
           py::arg("frame"),
           DOC(navground, core, Pose2, get_transformation_in_frame))
      .def("__repr__", &to_string<Pose2>);

  twist
      .def(py::init<Vector2, ng_float_t, Frame>(),
           py::arg("velocity") = Vector2::Zero(), py::arg("angular_speed") = 0,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           DOC(navground, core, Twist2, Twist2))
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def_readwrite("velocity", &Twist2::velocity,
                     DOC(navground, core, Twist2, velocity))
      .def_readwrite("angular_speed", &Twist2::angular_speed,
                     DOC(navground, core, Twist2, angular_speed))
      .def_readwrite("frame", &Twist2::frame,
                     DOC(navground, core, Twist2, frame))
      .def("rotate", &Twist2::rotate, py::arg("angle"),
           DOC(navground, core, Twist2, rotate))
      .def("absolute", &Twist2::absolute, py::arg("reference"),
           DOC(navground, core, Twist2, absolute))
      .def("relative", &Twist2::relative, py::arg("reference"),
           DOC(navground, core, Twist2, relative))
      .def("is_almost_zero", &Twist2::is_almost_zero,
           py::arg("epsilon_speed") = 1e-6,
           py::arg("epsilon_angular_speed") = 1e-6,
           DOC(navground, core, Twist2, is_almost_zero))
      .def("snap_to_zero", &Twist2::snap_to_zero, py::arg("epsilon") = 1e-6,
           DOC(navground, core, Twist2, snap_to_zero))
      .def("interpolate", &Twist2::interpolate, py::arg("target"),
           py::arg("time_step"), py::arg("max_acceleration"),
           py::arg("max_angular_acceleration"))
      .def("__repr__", &to_string<Twist2>);

  py::class_<Path>(m, "Path", DOC(navground, core, Path))
      .def(py::init<const Path::Projection &, const Path::Curve &, ng_float_t,
                    bool>(),
           py::arg("project"), py::arg("curve"), py::arg("length"),
           py::arg("loop") = false, DOC(navground, core, Path, Path))
      .def_readonly("project", &Path::project,
                    DOC(navground, core, Path, project))
      .def_readonly("curve", &Path::curve, DOC(navground, core, Path, curve))
      .def_readonly("length", &Path::length, DOC(navground, core, Path, length))
      .def_readonly("coordinate", &Path::coordinate,
                    DOC(navground, core, Path, coordinate))
      .def_readonly("loop", &Path::loop, DOC(navground, core, Path, loop))
      .def("track", &Path::track, py::arg("position"),
           py::arg("look_ahead") = 0, DOC(navground, core, Path, track))
      .def("get_point", &Path::get_point, py::arg("position"),
           py::arg("look_ahead") = 0, DOC(navground, core, Path, get_point))
      .def("__repr__", &to_string<Path>);

  py::class_<Target>(m, "Target", DOC(navground, core, Target))
      .def(py::init<std::optional<Vector2>, std::optional<Radians>,
                    std::optional<ng_float_t>, std::optional<Vector2>,
                    std::optional<ng_float_t>, std::optional<int>,
                    std::optional<Path>, ng_float_t, ng_float_t>(),
           py::arg("position") = py::none(),
           py::arg("orientation") = py::none(), py::arg("speed") = py::none(),
           py::arg("direction") = py::none(),
           py::arg("angular_speed") = py::none(), py::arg("path") = py::none(),
           py::arg("angular_direction") = py::none(),
           py::arg("position_tolerance") = 0,
           py::arg("orientation_tolerance") = 0,
           DOC(navground, core, Target, Target))
      .def_readwrite("position", &Target::position,
                     DOC(navground, core, Target, position))
      .def_readwrite("orientation", &Target::orientation,
                     DOC(navground, core, Target, orientation))
      .def_readwrite("speed", &Target::speed,
                     DOC(navground, core, Target, speed))
      .def_readwrite("angular_speed", &Target::angular_speed,
                     DOC(navground, core, Target, angular_speed))
      .def_readwrite("direction", &Target::direction,
                     DOC(navground, core, Target, direction))
      .def_readwrite("angular_direction", &Target::angular_direction,
                     DOC(navground, core, Target, angular_direction))
      .def_readwrite("position_tolerance", &Target::position_tolerance,
                     DOC(navground, core, Target, position_tolerance))
      .def_readwrite("orientation_tolerance", &Target::orientation_tolerance,
                     DOC(navground, core, Target, orientation_tolerance))
      .def_readwrite("path", &Target::path, DOC(navground, core, Target, path))
      .def("satisfied",
           py::overload_cast<const Vector2 &>(&Target::satisfied, py::const_),
           py::arg("point"), DOC(navground, core, Target, satisfied))
      .def("satisfied",
           py::overload_cast<ng_float_t>(&Target::satisfied, py::const_),
           py::arg("angle"), DOC(navground, core, Target, satisfied, 2))
      .def("satisfied",
           py::overload_cast<const Pose2 &>(&Target::satisfied, py::const_),
           py::arg("pose"), DOC(navground, core, Target, satisfied, 3))
      .def_property("valid", &Target::valid, nullptr,
                    DOC(navground, core, Target, valid))
      .def_static("Point", &Target::Point, py::arg("point"),
                  py::arg("tolerance") = 0, py::arg("along_path") = py::none(),
                  DOC(navground, core, Target, Point))
      .def_static("Pose", &Target::Pose, py::arg("pose"),
                  py::arg("position_tolerance") = 0,
                  py::arg("orientation_tolerance") = 0,
                  py::arg("along_path") = py::none(),
                  DOC(navground, core, Target, Pose))
      .def_static("Orientation", &Target::Orientation, py::arg("orientation"),
                  py::arg("tolerance") = 0,
                  DOC(navground, core, Target, Orientation))
      .def_static("Velocity", &Target::Velocity, py::arg("velocity"),
                  DOC(navground, core, Target, Velocity))
      .def_static("Direction", &Target::Direction, py::arg("direction"),
                  DOC(navground, core, Target, Direction))
      .def_static("Twist", &Target::Twist, py::arg("twist"),
                  DOC(navground, core, Target, Twist))
      .def_static("Stop", &Target::Stop, DOC(navground, core, Target, Stop))
      .def("__repr__", &to_string<Target>);

  py::class_<EnvironmentState, std::shared_ptr<EnvironmentState>>(
      m, "EnvironmentState", DOC(navground, core, EnvironmentState))
      .def(py::init<>(),
           DOC(navground, core, EnvironmentState, EnvironmentState));

  py::class_<Disc>(m, "Disc", DOC(navground, core, Disc))
      .def(py::init<Vector2, ng_float_t>(), py::arg("position"),
           py::arg("radius"), DOC(navground, core, Disc, Disc))
      .def_readwrite("position", &Disc::position,
                     DOC(navground, core, Disc, position))
      .def_readwrite("radius", &Disc::radius,
                     DOC(navground, core, Disc, radius))
      .def("distance", &Disc::distance, py::arg("other"),
           DOC(navground, core, Disc, distance))
      .def("__repr__", &to_string<Disc>)
      .def_static("schema", &YAML::schema_py<Disc>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_unique_py<Disc>, py::arg("value"),
                  YAML::load_string_py_doc("disc", "Disc").c_str())
      .def("dump", &YAML::dump<Disc>, YAML::dump_doc());

  py::class_<Neighbor, Disc>(m, "Neighbor", DOC(navground, core, Neighbor))
      .def(py::init<Vector2, ng_float_t, Vector2, int>(), py::arg("position"),
           py::arg("radius"), py::arg_v("velocity", Vector2::Zero(), "(0, 0)"),
           py::arg("id") = 0, DOC(navground, core, Neighbor, Neighbor))
      .def_readwrite("velocity", &Neighbor::velocity,
                     DOC(navground, core, Neighbor, velocity))
      .def_readwrite("id", &Neighbor::id, DOC(navground, core, Neighbor, id))
      .def("relative_to", &Neighbor::relative_to, py::arg("reference"),
           DOC(navground, core, Neighbor, relative_to))
      .def(py::self == py::self)
      .def(py::self != py::self)
      .def("__repr__", &to_string<Neighbor>)
      .def_static("schema", &YAML::schema_py<Neighbor>, YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_unique_py<Neighbor>, py::arg("value"),
                  YAML::load_string_py_doc("neighbor", "Neighbor").c_str())
      .def("dump", &YAML::dump<Neighbor>, YAML::dump_doc());

  py::class_<LineSegment>(m, "LineSegment", DOC(navground, core, LineSegment))
      .def(py::init<Vector2, Vector2>(), py::arg("p1"), py::arg("p2"),
           DOC(navground, core, LineSegment, LineSegment))
      .def_readonly("p1", &LineSegment::p1,
                    DOC(navground, core, LineSegment, p1))
      .def_readonly("p2", &LineSegment::p2,
                    DOC(navground, core, LineSegment, p2))
      .def_readonly("e1", &LineSegment::e1,
                    DOC(navground, core, LineSegment, e1))
      .def_readonly("e2", &LineSegment::e2,
                    DOC(navground, core, LineSegment, e2))
      .def_readonly("length", &LineSegment::length,
                    DOC(navground, core, LineSegment, length))
      .def("distance_from_point",
           py::overload_cast<const Vector2 &>(&LineSegment::distance,
                                              py::const_),
           py::arg("point"), DOC(navground, core, LineSegment, distance))
      .def("distance_along", &LineSegment::distance_along, py::arg("point"),
           py::arg("direction"), py::arg("orientation") = 0,
           DOC(navground, core, LineSegment, distance_along))
      .def("distance_from_disc",
           py::overload_cast<const Disc &, bool>(&LineSegment::distance,
                                                 py::const_),
           py::arg("disc"), py::arg("penetration") = false,
           DOC(navground, core, LineSegment, distance, 2))
      .def("__repr__", &to_string<LineSegment>)
      .def_static("schema", &YAML::schema_py<LineSegment>,
                  YAML::schema_py_doc())
      .def_static("load", &YAML::load_string_unique_py<LineSegment>,
                  py::arg("value"),
                  YAML::load_string_py_doc("line", "LineSegment").c_str())
      .def("dump", &YAML::dump<LineSegment>, YAML::dump_doc());

  py::class_<Kinematics, PyKinematics, HasRegister<Kinematics>, HasProperties,
             std::shared_ptr<Kinematics>>
      kinematics(m, "Kinematics", DOC(navground, core, Kinematics));
  kinematics
      .def(py::init<ng_float_t, ng_float_t>(),
           py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
           py::arg_v("max_angular_speed", Kinematics::inf, "float('inf')"))
      .def_property("max_speed", &Kinematics::get_max_speed,
                    &Kinematics::set_max_speed,
                    DOC(navground, core, Kinematics, property_max_speed))
      .def_property(
          "max_angular_speed", &Kinematics::get_max_angular_speed,
          &Kinematics::set_max_angular_speed,
          DOC(navground, core, Kinematics, property_max_angular_speed))
      .def("get_max_speed", &Kinematics::get_max_speed,
           DOC(navground, core, Kinematics, get_max_speed))
      .def("get_max_angular_speed", &Kinematics::get_max_angular_speed,
           DOC(navground, core, Kinematics, get_max_angular_speed))
      .def("is_wheeled", &Kinematics::is_wheeled,
           DOC(navground, core, Kinematics, is_wheeled))
      .def("dof", &Kinematics::dof, DOC(navground, core, Kinematics, dof))
      .def("feasible", &Kinematics::feasible, py::arg("twist"),
           DOC(navground, core, Kinematics, feasible))
      .def("feasible_from_current", &Kinematics::feasible_from_current,
           py::arg("twist"), py::arg("current"), py::arg("time_step"),
           DOC(navground, core, Kinematics, feasible_from_current))
      .def_static("load", &YAML::load_string_py<PyKinematics>, py::arg("value"),
                  YAML::load_string_py_doc("kinematics", "Kinematics").c_str());

  py::class_<OmnidirectionalKinematics, Kinematics,
             std::shared_ptr<OmnidirectionalKinematics>>
      omni(m, "OmnidirectionalKinematics",
           DOC(navground, core, OmnidirectionalKinematics));
  omni.def(py::init<ng_float_t, ng_float_t>(),
           py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
           py::arg_v("max_angular_speed", Kinematics::inf, "float('inf')"),
           DOC(navground, core, OmnidirectionalKinematics,
               OmnidirectionalKinematics));

  py::class_<AheadKinematics, Kinematics, std::shared_ptr<AheadKinematics>>
      ahead(m, "AheadKinematics", DOC(navground, core, AheadKinematics));
  ahead.def(py::init<ng_float_t, ng_float_t>(),
            py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
            py::arg_v("max_angular_speed", Kinematics::inf, "float('inf')"),
            DOC(navground, core, AheadKinematics, AheadKinematics));

  py::class_<WheeledKinematics, Kinematics, std::shared_ptr<WheeledKinematics>>
      wk(m, "WheeledKinematics", DOC(navground, core, WheeledKinematics));
  wk.def("twist", &WheeledKinematics::twist, py::arg("value"),
         DOC(navground, core, WheeledKinematics, twist))
      .def("wheel_speeds", &WheeledKinematics::wheel_speeds, py::arg("value"),
           DOC(navground, core, WheeledKinematics, wheel_speeds))
      .def("feasible_wheel_speeds", &WheeledKinematics::feasible_wheel_speeds,
           py::arg("value"),
           DOC(navground, core, WheeledKinematics, feasible_wheel_speeds));

  py::class_<TwoWheelsDifferentialDriveKinematics, WheeledKinematics,
             std::shared_ptr<TwoWheelsDifferentialDriveKinematics>>
      wk2(m, "TwoWheelsDifferentialDriveKinematics",
          DOC(navground, core, TwoWheelsDifferentialDriveKinematics));
  wk2.def(
         py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, ng_float_t>(),
         py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
         py::arg("wheel_axis") = 1,
         py::arg_v("max_angular_speed", Kinematics::inf, "float('inf')"),
         py::arg_v("max_forward_speed", Kinematics::inf, "float('inf')"),
         py::arg("max_backward_speed") = 0,
         DOC(navground, core, TwoWheelsDifferentialDriveKinematics,
             TwoWheelsDifferentialDriveKinematics))
      .def_property("wheel_axis",
                    &TwoWheelsDifferentialDriveKinematics::get_wheel_axis,
                    &TwoWheelsDifferentialDriveKinematics::set_wheel_axis,
                    DOC(navground, core, TwoWheelsDifferentialDriveKinematics,
                        property_wheel_axis))
      .def_property(
          "max_forward_speed",
          &TwoWheelsDifferentialDriveKinematics::get_max_forward_speed,
          &TwoWheelsDifferentialDriveKinematics::set_max_forward_speed,
          DOC(navground, core, TwoWheelsDifferentialDriveKinematics,
              property_max_forward_speed))
      .def_property(
          "max_backward_speed",
          &TwoWheelsDifferentialDriveKinematics::get_max_backward_speed,
          &TwoWheelsDifferentialDriveKinematics::set_max_backward_speed,
          DOC(navground, core, TwoWheelsDifferentialDriveKinematics,
              property_max_backward_speed))
      .def_property("can_move_forwards",
                    &TwoWheelsDifferentialDriveKinematics::can_move_forwards,
                    nullptr,
                    DOC(navground, core, TwoWheelsDifferentialDriveKinematics,
                        property_can_move_forwards))
      .def_property("can_move_backwards",
                    &TwoWheelsDifferentialDriveKinematics::can_move_backwards,
                    nullptr,
                    DOC(navground, core, TwoWheelsDifferentialDriveKinematics,
                        property_can_move_backwards));

  py::class_<FourWheelsOmniDriveKinematics, WheeledKinematics,
             std::shared_ptr<FourWheelsOmniDriveKinematics>>
      wk4(m, "FourWheelsOmniDriveKinematics",
          DOC(navground, core, FourWheelsOmniDriveKinematics));
  wk4.def(py::init<ng_float_t, ng_float_t>(),
          py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
          py::arg("wheel_axis") = 1,
          DOC(navground, core, FourWheelsOmniDriveKinematics,
              FourWheelsOmniDriveKinematics))
      .def_property("wheel_axis",
                    &FourWheelsOmniDriveKinematics::get_wheel_axis,
                    &FourWheelsOmniDriveKinematics::set_wheel_axis,
                    DOC(navground, core, FourWheelsOmniDriveKinematics,
                        property_wheel_axis));

  py::class_<BicycleKinematics, Kinematics, std::shared_ptr<BicycleKinematics>>
      bk(m, "BicycleKinematics", DOC(navground, core, BicycleKinematics));
  bk.def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, ng_float_t,
                  bool>(),
         py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
         py::arg_v("max_backward_speed", Kinematics::inf, "float('inf')"),
         py::arg("axis") = 1, py::arg("max_steering_angle") = 1,
         py::arg("k") = 0, py::arg("use_velocity_norm") = false,
         DOC(navground, core, BicycleKinematics, BicycleKinematics))
      .def_property(
          "max_steering_angle", &BicycleKinematics::get_max_steering_angle,
          &BicycleKinematics::set_max_steering_angle,
          DOC(navground, core, BicycleKinematics, property_max_steering_angle))
      .def_property(
          "min_steering_radius", &BicycleKinematics::get_min_steering_radius,
          nullptr,
          DOC(navground, core, BicycleKinematics, property_min_steering_radius))
      .def_property(
          "max_backward_speed", &BicycleKinematics::get_max_backward_speed,
          &BicycleKinematics::set_max_backward_speed,
          DOC(navground, core, BicycleKinematics, property_max_backward_speed))
      .def_property("axis", &BicycleKinematics::get_axis,
                    &BicycleKinematics::set_axis,
                    DOC(navground, core, BicycleKinematics, property_axis))
      .def_property("k", &BicycleKinematics::get_k, &BicycleKinematics::set_k,
                    DOC(navground, core, BicycleKinematics, property_k))
      .def_property(
          "use_velocity_norm", &BicycleKinematics::get_use_velocity_norm,
          &BicycleKinematics::set_use_velocity_norm,
          DOC(navground, core, BicycleKinematics, property_use_velocity_norm))
      .def("feasible_steering_angle",
           &BicycleKinematics::feasible_steering_angle,
           DOC(navground, core, BicycleKinematics, feasible_steering_angle))
      .def("twist_from_steering", &BicycleKinematics::twist_from_steering,
           DOC(navground, core, BicycleKinematics, twist_from_steering));

  py::class_<DynamicTwoWheelsDifferentialDriveKinematics,
             TwoWheelsDifferentialDriveKinematics, WheeledKinematics,
             std::shared_ptr<DynamicTwoWheelsDifferentialDriveKinematics>>
      dwk2(m, "DynamicTwoWheelsDifferentialDriveKinematics",
           DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics));
  dwk2.def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t, ng_float_t,
                    ng_float_t, ng_float_t>(),
           py::arg_v("max_speed", Kinematics::inf, "float('inf')"),
           py::arg("wheel_axis") = 1,
           py::arg_v("max_angular_speed", Kinematics::inf, "float('inf')"),
           py::arg_v("max_forward_speed", Kinematics::inf, "float('inf')"),
           py::arg("max_backward_speed") = 0,
           py::arg_v("max_acceleration", Kinematics::inf, "float('inf')"),
           py::arg("moi") = 1,
           DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
               DynamicTwoWheelsDifferentialDriveKinematics))
      .def_property(
          "max_acceleration",
          &DynamicTwoWheelsDifferentialDriveKinematics::get_max_acceleration,
          &DynamicTwoWheelsDifferentialDriveKinematics::set_max_acceleration,
          DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
              property_max_acceleration))
      .def_property("max_angular_acceleration",
                    &DynamicTwoWheelsDifferentialDriveKinematics::
                        get_max_angular_acceleration,
                    &DynamicTwoWheelsDifferentialDriveKinematics::
                        set_max_angular_acceleration,
                    DOC(navground, core,
                        DynamicTwoWheelsDifferentialDriveKinematics,
                        property_max_angular_acceleration))
      .def_property(
          "moi", &DynamicTwoWheelsDifferentialDriveKinematics::get_moi,
          &DynamicTwoWheelsDifferentialDriveKinematics::set_moi,
          DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
              property_moi))
      .def("wheel_torques",
           &DynamicTwoWheelsDifferentialDriveKinematics::wheel_torques,
           py::arg("value"), py::arg("current"), py::arg("time_step"),
           DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
               wheel_torques))
      .def("twist_from_wheel_torques",
           &DynamicTwoWheelsDifferentialDriveKinematics::
               twist_from_wheel_torques,
           py::arg("values"), py::arg("current"), py::arg("time_step"),
           DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
               twist_from_wheel_torques))
      .def_property(
          "max_wheel_torque",
          &DynamicTwoWheelsDifferentialDriveKinematics::get_max_wheel_torque,
          nullptr,
          DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
              property_max_wheel_torque));
  // .def_property(
  //     "reduce_torques",
  //     &DynamicTwoWheelsDifferentialDriveKinematics::get_reduce_torques,
  //     &DynamicTwoWheelsDifferentialDriveKinematics::set_reduce_torques,
  //     DOC(navground, core, DynamicTwoWheelsDifferentialDriveKinematics,
  //         property_reduce_torques));

  py::class_<SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::Modulation>>(
      m, "SocialMarginModulation",
      DOC(navground, core, SocialMargin_Modulation))
      .def(
          "__call__",
          [](const SocialMargin::Modulation *mod, ng_float_t margin,
             std::optional<ng_float_t> distance) {
            if (distance) {
              return (*mod)(margin, *distance);
            }
            return (*mod)(margin);
          },
          py::arg("margin"), py::arg("distance") = py::none(),
          DOC(navground, core, SocialMargin_Modulation, operator_call));

  py::class_<SocialMargin::ZeroModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::ZeroModulation>>(
      m, "SocialMarginZeroModulation",
      DOC(navground, core, SocialMargin_ZeroModulation))
      .def(py::init<>(),
           DOC(navground, core, SocialMargin_Modulation, Modulation));
  py::class_<SocialMargin::ConstantModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::ConstantModulation>>(
      m, "SocialMarginConstantModulation",
      DOC(navground, core, SocialMargin_ConstantModulation))
      .def(py::init<>(),
           DOC(navground, core, SocialMargin_Modulation, Modulation));
  py::class_<SocialMargin::LinearModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::LinearModulation>>(
      m, "SocialMarginLinearModulation",
      DOC(navground, core, SocialMargin_LinearModulation))
      .def(py::init<ng_float_t>(), py::arg("upper_distance"),
           DOC(navground, core, SocialMargin_LinearModulation,
               LinearModulation));
  py::class_<SocialMargin::QuadraticModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::QuadraticModulation>>(
      m, "SocialMarginQuadraticModulation",
      DOC(navground, core, SocialMargin_QuadraticModulation))
      .def(py::init<ng_float_t>(), py::arg("upper_distance"),
           DOC(navground, core, SocialMargin_QuadraticModulation,
               QuadraticModulation));
  py::class_<SocialMargin::LogisticModulation, SocialMargin::Modulation,
             std::shared_ptr<SocialMargin::LogisticModulation>>(
      m, "SocialMarginLogisticModulation",
      DOC(navground, core, SocialMargin_LogisticModulation))
      .def(py::init<>(),
           DOC(navground, core, SocialMargin_Modulation, Modulation));

  py::class_<SocialMargin, std::shared_ptr<SocialMargin>> social_margin(
      m, "SocialMargin", DOC(navground, core, SocialMargin));
  social_margin
      .def(py::init<ng_float_t>(), py::arg("value") = 0,
           DOC(navground, core, SocialMargin, SocialMargin))
      .def_property("modulation", &SocialMargin::get_modulation,
                    &SocialMargin::set_modulation,
                    DOC(navground, core, SocialMargin, property_modulation))
      .def(
          "get",
          [](SocialMargin *sm, std::optional<unsigned> type,
             std::optional<ng_float_t> distance) {
            if (!type) {
              return sm->get();
            }
            if (!distance) {
              return sm->get(*type);
            }
            return sm->get(*type, *distance);
          },
          py::arg("type") = py::none(), py::arg("distance") = py::none(),
          DOC(navground, core, SocialMargin, get))
      .def(
          "set",
          [](SocialMargin *sm, ng_float_t value, std::optional<unsigned> type) {
            if (!type) {
              return sm->set(value);
            }
            return sm->set(*type, value);
          },
          py::arg("value"), py::arg("type") = py::none(),
          DOC(navground, core, SocialMargin, set))
      .def_property("max_value", &SocialMargin::get_max_value, nullptr,
                    DOC(navground, core, SocialMargin, property_max_value))
      .def_static("schema", &YAML::schema_py<SocialMargin>,
                  YAML::schema_py_doc());

  py::class_<BehaviorModulation, PyBehaviorModulation,
             HasRegister<BehaviorModulation>, HasProperties,
             std::shared_ptr<BehaviorModulation>>
      behavior_modulation(m, "BehaviorModulation",
                          DOC(navground, core, BehaviorModulation));

  py::class_<Behavior, PyBehavior, HasRegister<Behavior>, HasProperties,
             std::shared_ptr<Behavior>>
      behavior(m, "Behavior", py::dynamic_attr(),
               DOC(navground, core, Behavior));

  behavior_modulation
      .def(py::init<>(),
           DOC(navground, core, BehaviorModulation, BehaviorModulation))
      .def("pre", &BehaviorModulation::pre, py::arg("behavior"),
           py::arg("time_step"), DOC(navground, core, BehaviorModulation, pre))
      .def("post", &BehaviorModulation::post, py::arg("behavior"),
           py::arg("time_step"), py::arg("cmd"),
           DOC(navground, core, BehaviorModulation, post))
      // .def_property(
      //     "type", [](BehaviorModulation *obj) { return obj->get_type(); },
      //     nullptr, DOC(navground, core, HasRegister, property_type))
      .def_property("enabled", &BehaviorModulation::get_enabled,
                    &BehaviorModulation::set_enabled,
                    DOC(navground, core, BehaviorModulation, property_enabled))
      .def_static(
          "load", &YAML::load_string_py<PyBehaviorModulation>, py::arg("value"),
          YAML::load_string_py_doc("modulation", "BehaviorModulation").c_str());

  py::class_<RelaxationModulation, BehaviorModulation,
             std::shared_ptr<RelaxationModulation>>
      relaxation(m, "RelaxationModulation",
                 DOC(navground, core, RelaxationModulation));

  relaxation
      .def(py::init<ng_float_t>(), py::arg("tau") = 0.125,
           DOC(navground, core, RelaxationModulation, RelaxationModulation))
      .def_property("tau", &RelaxationModulation::get_tau,
                    &RelaxationModulation::set_tau,
                    DOC(navground, core, RelaxationModulation, property_tau));

  py::class_<LimitAccelerationModulation, BehaviorModulation,
             std::shared_ptr<LimitAccelerationModulation>>
      limit_acceleration(m, "LimitAccelerationModulation",
                         DOC(navground, core, LimitAccelerationModulation));

  limit_acceleration
      .def(py::init<ng_float_t, ng_float_t>(),
           py::arg_v("max_acceleration",
                     std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           py::arg_v("max_angular_acceleration",
                     std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           DOC(navground, core, LimitAccelerationModulation,
               LimitAccelerationModulation))
      .def_property("max_acceleration",
                    &LimitAccelerationModulation::get_max_acceleration,
                    &LimitAccelerationModulation::set_max_acceleration,
                    DOC(navground, core, LimitAccelerationModulation,
                        property_max_acceleration))
      .def_property("max_angular_acceleration",
                    &LimitAccelerationModulation::get_max_angular_acceleration,
                    &LimitAccelerationModulation::set_max_angular_acceleration,
                    DOC(navground, core, LimitAccelerationModulation,
                        property_max_angular_acceleration));

  py::class_<LimitTwistModulation, BehaviorModulation,
             std::shared_ptr<LimitTwistModulation>>
      limit_twist(m, "LimitTwistModulation",
                  DOC(navground, core, LimitTwistModulation));

  limit_twist
      .def(py::init<ng_float_t, ng_float_t, ng_float_t, ng_float_t,
                    ng_float_t>(),
           py::arg_v("forward", std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           py::arg_v("backward", std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           py::arg_v("leftward", std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           py::arg_v("rightward", std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           py::arg_v("angular", std::numeric_limits<ng_float_t>::infinity(),
                     "float('inf')"),
           DOC(navground, core, LimitTwistModulation, LimitTwistModulation))
      .def_property("max_forward_speed",
                    &LimitTwistModulation::get_max_forward_speed,
                    &LimitTwistModulation::set_max_forward_speed,
                    DOC(navground, core, LimitTwistModulation,
                        property_max_forward_speed))
      .def_property("max_backward_speed",
                    &LimitTwistModulation::get_max_backward_speed,
                    &LimitTwistModulation::set_max_backward_speed,
                    DOC(navground, core, LimitTwistModulation,
                        property_max_backward_speed))
      .def_property("max_leftward_speed",
                    &LimitTwistModulation::get_max_leftward_speed,
                    &LimitTwistModulation::set_max_leftward_speed,
                    DOC(navground, core, LimitTwistModulation,
                        property_max_leftward_speed))
      .def_property("max_rightward_speed",
                    &LimitTwistModulation::get_max_rightward_speed,
                    &LimitTwistModulation::set_max_rightward_speed,
                    DOC(navground, core, LimitTwistModulation,
                        property_max_rightward_speed))
      .def_property("max_angular_speed",
                    &LimitTwistModulation::get_max_angular_speed,
                    &LimitTwistModulation::set_max_angular_speed,
                    DOC(navground, core, LimitTwistModulation, property,
                        max_angular_speed));

  py::class_<MotorPIDModulation, BehaviorModulation,
             std::shared_ptr<MotorPIDModulation>>
      motor_pid(m, "MotorPIDModulation",
                DOC(navground, core, MotorPIDModulation));

  motor_pid
      .def(py::init<ng_float_t, ng_float_t, ng_float_t>(), py::arg("k_p") = 1,
           py::arg("k_i") = 0, py::arg("k_d") = 0,
           DOC(navground, core, MotorPIDModulation, MotorPIDModulation))
      .def_property("k_p", &MotorPIDModulation::get_k_p,
                    &MotorPIDModulation::set_k_p,
                    DOC(navground, core, MotorPIDModulation, property_k_p))
      .def_property("k_i", &MotorPIDModulation::get_k_i,
                    &MotorPIDModulation::set_k_i,
                    DOC(navground, core, MotorPIDModulation, property_k_i))
      .def_property("k_d", &MotorPIDModulation::get_k_d,
                    &MotorPIDModulation::set_k_d,
                    DOC(navground, core, MotorPIDModulation, property_k_d));

  py::enum_<Behavior::Heading>(behavior, "Heading",
                               DOC(navground, core, Behavior_Heading))
      .value("idle", Behavior::Heading::idle,
             DOC(navground, core, Behavior_Heading, idle))
      .value("target_point", Behavior::Heading::target_point,
             DOC(navground, core, Behavior_Heading, target_point))
      .value("target_angle", Behavior::Heading::target_angle,
             DOC(navground, core, Behavior_Heading, target_angle))
      .value("target_angular_speed", Behavior::Heading::target_angular_speed,
             DOC(navground, core, Behavior_Heading, target_angular_speed))
      .value("velocity", Behavior::Heading::velocity,
             DOC(navground, core, Behavior_Heading, velocity));

  behavior
      .def(py::init<std::shared_ptr<Kinematics>, ng_float_t>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0,
           DOC(navground, core, Behavior, Behavior))
      .def("clear_modulations", &clear_modulations_py,
           DOC(navground, core, Behavior, clear_modulations))
      .def("add_modulation", &add_modulation_py, py::arg("value"),
           DOC(navground, core, Behavior, add_modulation))
      .def("remove_modulation", &remove_modulation_py, py::arg("value"),
           DOC(navground, core, Behavior, remove_modulation))
      .def_property("modulations",
                    py::overload_cast<>(&Behavior::get_modulations, py::const_),
                    nullptr,
                    DOC(navground, core, Behavior, property_modulations))
      .def_property(
          "kinematics", &Behavior::get_kinematics,
          py::cpp_function(&Behavior::set_kinematics, py::keep_alive<1, 2>()),
          DOC(navground, core, Behavior, property_kinematics))
      .def_property("radius", &Behavior::get_radius, &Behavior::set_radius,
                    DOC(navground, core, Behavior, property_radius))
      .def_property("max_speed", &Behavior::get_max_speed,
                    &Behavior::set_max_speed,
                    DOC(navground, core, Behavior, property_max_speed))
      .def_property("max_angular_speed", &Behavior::get_max_angular_speed,
                    &Behavior::set_max_angular_speed,
                    DOC(navground, core, Behavior, property_max_angular_speed))
      .def_property("optimal_speed", &Behavior::get_optimal_speed,
                    &Behavior::set_optimal_speed,
                    DOC(navground, core, Behavior, property_optimal_speed))
      .def_property(
          "optimal_angular_speed", &Behavior::get_optimal_angular_speed,
          &Behavior::set_optimal_angular_speed,
          DOC(navground, core, Behavior, property_optimal_angular_speed))
      .def_property("rotation_tau", &Behavior::get_rotation_tau,
                    &Behavior::set_rotation_tau,
                    DOC(navground, core, Behavior, property_rotation_tau))
      .def_property("safety_margin", &Behavior::get_safety_margin,
                    &Behavior::set_safety_margin,
                    DOC(navground, core, Behavior, property_safety_margin))
      .def_property("horizon", &Behavior::get_horizon, &Behavior::set_horizon,
                    DOC(navground, core, Behavior, property_horizon))
      .def_property("path_tau", &Behavior::get_path_tau,
                    &Behavior::set_path_tau,
                    DOC(navground, core, Behavior, property_path_tau))
      .def_property("path_look_ahead", &Behavior::get_path_look_ahead,
                    &Behavior::set_path_look_ahead,
                    DOC(navground, core, Behavior, property_path_look_ahead))
      .def_property("pose", &Behavior::get_pose, &Behavior::set_pose,
                    DOC(navground, core, Behavior, property_pose))
      // .def_property("pose_ref", &Behavior::get_pose_ref,
      //               DOC(navground, core, Behavior, property_pose))
      .def_property("position", &Behavior::get_position,
                    &Behavior::set_position,
                    DOC(navground, core, Behavior, property_position))
      .def_property("orientation", &Behavior::get_orientation,
                    &Behavior::set_orientation,
                    DOC(navground, core, Behavior, property_orientation))
      .def_property(
          "assume_cmd_is_actuated", &Behavior::get_assume_cmd_is_actuated,
          &Behavior::set_assume_cmd_is_actuated,
          DOC(navground, core, Behavior, property_assume_cmd_is_actuated))

      .def_readonly("social_margin", &Behavior::social_margin,
                    DOC(navground, core, Behavior, social_margin))
      .def_property(
          "twist", [](const Behavior &self) { return self.get_twist(); },
          &Behavior::set_twist, DOC(navground, core, Behavior, property_twist))
      .def("get_twist", &Behavior::get_twist,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           DOC(navground, core, Behavior, get_twist))
      .def_property(
          "velocity", [](const Behavior &self) { return self.get_velocity(); },
          [](Behavior &self, const Vector2 v) { return self.set_velocity(v); },
          DOC(navground, core, Behavior, property_velocity))
      .def("get_velocity", &Behavior::get_velocity,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           DOC(navground, core, Behavior, get_velocity))
      .def("set_velocity", &Behavior::set_velocity, py::arg("velocity"),
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           DOC(navground, core, Behavior, set_velocity))
      .def_property("angular_speed", &Behavior::get_angular_speed,
                    &Behavior::set_angular_speed,
                    DOC(navground, core, Behavior, property_angular_speed))
      .def_property("speed", &Behavior::get_speed, nullptr,
                    DOC(navground, core, Behavior, property_speed))
      .def_property("wheel_speeds", &Behavior::get_wheel_speeds,
                    &Behavior::set_wheel_speeds,
                    DOC(navground, core, Behavior, property_wheel_speeds))
      .def_property(
          "actuated_twist",
          [](const Behavior &self) { return self.get_actuated_twist(); },
          &Behavior::set_actuated_twist,
          DOC(navground, core, Behavior, property_actuated_twist))
      // .def_property("actuated_twist_ref", &Behavior::get_actuated_twist_ref,
      //               nullptr,
      //               DOC(navground, core, Behavior,
      //               property_actuated_twist_ref))
      .def("get_actuated_twist", &Behavior::get_actuated_twist,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           DOC(navground, core, Behavior, get_actuated_twist))
      .def_property(
          "actuated_wheel_speeds", &Behavior::get_actuated_wheel_speeds,
          nullptr,
          DOC(navground, core, Behavior, property_actuated_wheel_speeds))
      .def("actuate",
           py::overload_cast<const Twist2 &, ng_float_t, bool>(
               &Behavior::actuate),
           py::arg("twist"), py::arg("time_step"),
           py::arg("enforce_feasibility") = false,
           DOC(navground, core, Behavior, actuate))
      .def("actuate", py::overload_cast<ng_float_t>(&Behavior::actuate),
           py::arg("time_step"), DOC(navground, core, Behavior, actuate, 2))
      .def_property("heading_behavior", &Behavior::get_heading_behavior,
                    &Behavior::set_heading_behavior,
                    DOC(navground, core, Behavior, property_heading_behavior))
      .def_property("has_target", &Behavior::has_target, nullptr,
                    DOC(navground, core, Behavior, property_has_target))
      .def_property("target", &Behavior::get_target, &Behavior::set_target,
                    DOC(navground, core, Behavior, property_target))
      .def_property("target_ref", &Behavior::get_target_ref, nullptr,
                    DOC(navground, core, Behavior, property_target_ref))
      .def("check_if_target_satisfied", &Behavior::check_if_target_satisfied,
           DOC(navground, core, Behavior, check_if_target_satisfied))
      .def("prepare", &Behavior::prepare,
           DOC(navground, core, Behavior, prepare))
      .def("close", &Behavior::close, DOC(navground, core, Behavior, close))
      .def("compute_cmd", &Behavior::compute_cmd, py::arg("time_step"),
           py::arg("frame") = py::none(),
           py::arg("enforce_feasibility") = false,
           DOC(navground, core, Behavior, compute_cmd))
      .def(
          "compute_cmd_internal",
          [](PyBehavior &behavior, ng_float_t time_step) {
            return behavior.compute_cmd_internal(time_step);
          },
          py::arg("time_step"),
          DOC(navground, core, Behavior, compute_cmd_internal))
      .def(
          "desired_velocity_towards_point",
          [](PyBehavior &behavior, const Vector2 &point, ng_float_t speed,
             ng_float_t time_step) {
            return behavior.desired_velocity_towards_point(point, speed,
                                                           time_step);
          },
          py::arg("point"), py::arg("speed"), py::arg("time_step"),
          DOC(navground, core, Behavior, desired_velocity_towards_point))
      .def(
          "desired_velocity_towards_velocity",
          [](PyBehavior &behavior, const Vector2 &velocity,
             ng_float_t time_step) {
            return behavior.desired_velocity_towards_velocity(velocity,
                                                              time_step);
          },
          py::arg("velocity"), py::arg("time_step"),
          DOC(navground, core, Behavior, desired_velocity_towards_velocity))
      .def(
          "twist_towards_velocity",
          [](PyBehavior &behavior, const Vector2 &velocity) {
            return behavior.twist_towards_velocity(velocity);
          },
          py::arg("velocity"),
          DOC(navground, core, Behavior, twist_towards_velocity))
      .def(
          "cmd_twist_along_path",
          [](PyBehavior &behavior, Path &path, ng_float_t speed,
             ng_float_t time_step) {
            return behavior.cmd_twist_along_path(path, speed, time_step);
          },
          py::arg("path"), py::arg("speed"), py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_along_path))
      .def(
          "cmd_twist_towards_pose",
          [](PyBehavior &behavior, const Pose2 &pose, ng_float_t speed,
             ng_float_t angular_speed, ng_float_t time_step) {
            return behavior.cmd_twist_towards_pose(pose, speed, angular_speed,
                                                   time_step);
          },
          py::arg("pose"), py::arg("speed"), py::arg("angular_speed"),
          py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_towards_pose))
      .def(
          "cmd_twist_towards_point",
          [](PyBehavior &behavior, const Vector2 &point, ng_float_t speed,
             ng_float_t time_step) {
            return behavior.cmd_twist_towards_point(point, speed, time_step);
          },
          py::arg("point"), py::arg("speed"), py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_towards_point))
      .def(
          "cmd_twist_towards_velocity",
          [](PyBehavior &behavior, const Vector2 &velocity,
             ng_float_t time_step) {
            return behavior.cmd_twist_towards_velocity(velocity, time_step);
          },
          py::arg("velocity"), py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_towards_velocity))
      .def(
          "cmd_twist_towards_orientation",
          [](PyBehavior &behavior, ng_float_t orientation,
             ng_float_t angular_speed, ng_float_t time_step) {
            return behavior.cmd_twist_towards_orientation(
                orientation, angular_speed, time_step);
          },
          py::arg("orientation"), py::arg("angular_speed"),
          py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_towards_orientation))
      .def(
          "cmd_twist_towards_angular_speed",
          [](PyBehavior &behavior, ng_float_t angular_speed,
             ng_float_t time_step) {
            return behavior.cmd_twist_towards_angular_speed(angular_speed,
                                                            time_step);
          },
          py::arg("angular_speed"), py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_towards_angular_speed))
      .def(
          "cmd_twist_towards_stopping",
          [](PyBehavior &behavior, ng_float_t time_step) {
            return behavior.cmd_twist_towards_stopping(time_step);
          },
          py::arg("time_step"),
          DOC(navground, core, Behavior, cmd_twist_towards_stopping))
      .def_property("desired_velocity", &Behavior::get_desired_velocity,
                    nullptr,
                    DOC(navground, core, Behavior, property_desired_velocity))
      // .def_property(
      //     "type", [](Behavior *obj) { return obj->get_type(); }, nullptr,
      //     DOC(navground, core, HasRegister, property_type))
      .def("to_frame", &Behavior::to_frame, py::arg("value"), py::arg("frame"),
           DOC(navground, core, Behavior, to_frame))
      .def("feasible_speed", &Behavior::feasible_speed, py::arg("value"),
           DOC(navground, core, Behavior, feasible_speed))
      .def("feasible_angular_speed", &Behavior::feasible_angular_speed,
           py::arg("value"),
           DOC(navground, core, Behavior, feasible_angular_speed))
      .def("feasible_twist", &Behavior::feasible_twist, py::arg("value"),
           DOC(navground, core, Behavior, feasible_twist))
      .def("feasible_twist_from_current",
           &Behavior::feasible_twist_from_current, py::arg("value"),
           py::arg("time_step"),
           DOC(navground, core, Behavior, feasible_twist_from_current))
      .def("estimate_time_until_target_satisfied",
           &Behavior::estimate_time_until_target_satisfied,
           DOC(navground, core, Behavior, estimate_time_until_target_satisfied))
      .def_property("environment_state",
                    py::cpp_function(&Behavior::get_environment_state,
                                     py::return_value_policy::reference),
                    nullptr,
                    DOC(navground, core, Behavior, property_environment_state))
      .def("get_environment_state", &Behavior::get_environment_state,
           py::return_value_policy::reference,
           DOC(navground, core, Behavior, get_environment_state))
      .def("wheel_speeds_from_twist", &Behavior::wheel_speeds_from_twist,
           py::arg("value"),
           DOC(navground, core, Behavior, wheel_speeds_from_twist))
      .def("twist_from_wheel_speeds", &Behavior::twist_from_wheel_speeds,
           py::arg("value"),
           DOC(navground, core, Behavior, twist_from_wheel_speeds))
      .def("set_state_from", &Behavior::set_state_from, py::arg("other"),
           DOC(navground, core, Behavior, set_state_from))
      .def_property("efficacy", &Behavior::get_efficacy, nullptr,
                    DOC(navground, core, Behavior, property_efficacy))
      .def_property("is_stuck", &Behavior::is_stuck, nullptr,
                    DOC(navground, core, Behavior, property_is_stuck))
      .def("get_target_pose", &Behavior::get_target_pose,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_pose))
      .def("get_target_position", &Behavior::get_target_position,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_position))
      .def("get_target_orientation", &Behavior::get_target_orientation,
           py::arg("frame"), py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_orientation))
      .def("get_target_direction", &Behavior::get_target_direction,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_direction))
      .def("get_target_angular_direction",
           &Behavior::get_target_angular_direction,
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_angular_direction))
      .def("get_target_distance", &Behavior::get_target_distance,
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_distance))
      .def("get_target_angular_distance",
           &Behavior::get_target_angular_distance,
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_angular_distance))
      .def("get_target_twist", &Behavior::get_target_twist,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_velocity))
      .def("get_target_velocity", &Behavior::get_target_velocity,
           py::arg_v("frame", Frame::absolute,
                     "navground.core._navground.Frame.absolute"),
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_velocity))
      .def("get_target_angular_velocity",
           &Behavior::get_target_angular_velocity,
           py::arg("ignore_tolerance") = false,
           DOC(navground, core, Behavior, get_target_angular_velocity))
      .def("get_target_speed", &Behavior::get_target_speed,
           DOC(navground, core, Behavior, get_target_speed))
      .def("get_target_angular_speed", &Behavior::get_target_angular_speed,
           DOC(navground, core, Behavior, get_target_angular_speed))
      .def("is_stopped", &Behavior::is_stopped, py::arg("epsilon_speed") = 1e-6,
           py::arg("epsilon_angular_speed") = 1e-6,
           DOC(navground, core, Behavior, is_stopped))
      .def_static("load", &YAML::load_string_py<PyBehavior>, py::arg("value"),
                  YAML::load_string_py_doc("behavior", "Behavior").c_str());

  m.def(
      "behavior_has_geometric_state",
      [](Behavior *obj) {
        return (dynamic_cast<GeometricState *>(obj->get_environment_state())) !=
               nullptr;
      },
      py::arg("behavior"));

  py::class_<GeometricState, EnvironmentState, std::shared_ptr<GeometricState>>(
      m, "GeometricState", DOC(navground, core, GeometricState))
      .def(py::init<>(), DOC(navground, core, GeometricState, GeometricState))
      .def_property("neighbors", &GeometricState::get_neighbors,
                    &GeometricState::set_neighbors,
                    DOC(navground, core, GeometricState, property_neighbors))
      .def_property(
          "static_obstacles", &GeometricState::get_static_obstacles,
          &GeometricState::set_static_obstacles,
          DOC(navground, core, GeometricState, property_static_obstacles))
      .def_property(
          "line_obstacles", &GeometricState::get_line_obstacles,
          &GeometricState::set_line_obstacles,
          DOC(navground, core, GeometricState, property_line_obstacles));

  py::class_<BufferDescription>(m, "BufferDescription",
                                DOC(navground, core, BufferDescription))
      .def(py::init([](const BufferShape &shape, const py::object &_dtype,
                       double low, double high, bool categorical) {
             const py::dtype dtype = make_dtype(_dtype);
             const std::string fmt = type_from_dtype(dtype);
             return new BufferDescription(shape, fmt, low, high, categorical);
           }),
           py::arg("shape") = BufferShape{}, py::arg("dtype") = "float",
           py::arg("low") = std::numeric_limits<double>::min(),
           py::arg("high") = std::numeric_limits<double>::max(),
           py::arg("categorical") = false,
           DOC(navground, core, BufferDescription, BufferDescription))
      .def_property(
          "shape",
          [](const BufferDescription &value) {
            return py::tuple(py::cast(value.shape));
          },
          nullptr, DOC(navground, core, BufferDescription, shape))
      .def_property(
          "type",
          [](const BufferDescription &value) { return py::dtype(value.type); },
          nullptr, DOC(navground, core, BufferDescription, type))
      .def_property("strides", &BufferDescription::get_strides, nullptr,
                    DOC(navground, core, BufferDescription, property_strides))
      .def_readonly("low", &BufferDescription::low,
                    DOC(navground, core, BufferDescription, low))
      .def_readonly("high", &BufferDescription::high,
                    DOC(navground, core, BufferDescription, high))
      .def_readonly("categorical", &BufferDescription::categorical,
                    DOC(navground, core, BufferDescription, categorical))
      .def("__repr__",
           [](const BufferDescription &value) -> py::str {
             py::str r("BufferDescription(shape=");
             r += py::str(py::tuple(py::cast(value.shape)));
             r += py::str(", type=") + py::dtype(value.type).attr("__repr__")();
             r += py::str(", low=") + py::str(py::cast(value.low));
             r += py::str(", high=") + py::str(py::cast(value.high));
             r += py::str(", categorical=") +
                  py::str(py::cast(value.categorical)) + py::str(")");
             return r;
           })
      .def(py::pickle(
          [](const BufferDescription &value) {
            return py::make_tuple(py::tuple(py::cast(value.shape)), value.type,
                                  value.low, value.high, value.categorical);
          },
          [](py::tuple v) {
            const py::dtype dtype = make_dtype(v[1]);
            const std::string fmt = type_from_dtype(dtype);
            BufferDescription desc(
                py::cast<BufferShape>(v[0]), fmt, py::cast<double>(v[2]),
                py::cast<double>(v[3]), py::cast<bool>(v[4]));
            return desc;
          }));

  py::class_<Buffer>(m, "Buffer", DOC(navground, core, Buffer))
#if 0
      .def(py::init<const BufferDescription &, BufferType>(),
           py::arg("description"), py::arg("value"),
           DOC(navground, core, Buffer, Buffer))
      .def(py::init<const BufferDescription &, const BufferData &>(),
           py::arg("description"), py::arg("data"),
           DOC(navground, core, Buffer, Buffer, 2))
      .def(py::init<const BufferDescription &>(), py::arg("description"),
           DOC(navground, core, Buffer, Buffer, 3))
      .def(py::init<const BufferData &>(),
           py::arg("data") = std::valarray<ng_float_t>(0),
           DOC(navground, core, Buffer, Buffer, 4))
      .def(py::init([](const py::buffer &value) {
             Buffer buffer;
             set_buffer_from_buffer(buffer, value, true);
             return buffer;
           }),
           R"doc(Constructs an unbounded buffer with the provided data)doc")
#endif
      .def(py::init([](const std::optional<BufferDescription> &description =
                           std::nullopt,
                       const py::object &data = py::none()) {
             return make_buffer(description, data);
           }),
           py::arg("description") = std::nullopt, py::arg("data") = py::none(),
           R"doc(
Constructs a new instance.

:param description: The description
:type description: :py:class:`navground.core.BufferDescription` | None
:param data: Optional data to initialize the buffer. If provided, 
             it will overwrite data initialized from the description.
             Else the buffer will be initialized with zeros.
             Accepts any type supporting the buffer protocol.
:type data: object
)doc")
      .def_property("size", &Buffer::size, nullptr,
                    DOC(navground, core, Buffer, property_size))
      .def_property(
          "type",
          [](const Buffer &buffer) { return dtype_from_buffer(buffer); },
          [](Buffer &buffer, const py::object &value) {
            const auto dtype = make_dtype(value);
            buffer.set_type(type_from_dtype(dtype), true);
          },
          "The buffer type")
      .def_property(
          "data",
          [](const Buffer &buffer) { return get_array_from_buffer(buffer); },
          [](Buffer &buffer, const py::buffer &value) {
            set_buffer_from_buffer(buffer, value, true);
          },
          "The data stored in the buffer")
      .def_property(
          "description", &Buffer::get_description,
          [](Buffer &buffer, const BufferDescription &value) {
            buffer.set_description(value, true);
          },
          DOC(navground, core, Buffer, property_description))
      .def_property("low", &Buffer::get_low, &Buffer::set_low,
                    DOC(navground, core, Buffer, property_low))
      .def_property("high", &Buffer::get_high, &Buffer::set_high,
                    DOC(navground, core, Buffer, property_high))
      .def_property("categorical", &Buffer::get_categorical,
                    &Buffer::set_categorical,
                    DOC(navground, core, Buffer, property_categorical))
      .def_property(
          "shape",
          [](const Buffer &buffer) {
            return py::tuple(py::cast(buffer.get_shape()));
          },
          [](Buffer &buffer, const BufferShape &value) {
            buffer.set_shape(value, true);
          },
          DOC(navground, core, Buffer, property_shape))
      .def("set_description", &Buffer::set_description, py::arg("value"),
           py::arg("force") = false,
           DOC(navground, core, Buffer, set_description))
      .def("__repr__",
           [](const Buffer &value) -> py::str {
             py::str r("Buffer(description=");
             const auto obj = py::cast(value);
             r += obj.attr("description").attr("__repr__")();
             r += py::str(", data=") + obj.attr("data").attr("__repr__")() +
                  py::str(")");
             return r;
           })
      .def(py::pickle(
          [](const Buffer &value) {
            return py::make_tuple(value.get_description(),
                                  get_array_from_buffer(value));
          },
          [](py::tuple v) {
            Buffer buffer(py::cast<BufferDescription>(v[0]));
            set_buffer_from_buffer(buffer, py::cast<py::buffer>(v[1]), true);
            return buffer;
          }));

  py::bind_map<std::map<std::string, Buffer>>(
      m, "BufferMap", "A dictionary of type Dict[str, Buffer]");

  py::class_<SensingState, EnvironmentState, std::shared_ptr<SensingState>>(
      m, "SensingState", DOC(navground, core, SensingState))
      .def(py::init<>(), DOC(navground, core, SensingState, SensingState))
#if 0
      .def("init_buffer",
           py::overload_cast<const std::string &, const BufferDescription &,
                             BufferType>(&SensingState::init_buffer),
           py::arg("key"), py::arg("description"), py::arg("value"),
           py::return_value_policy::automatic_reference,
           DOC(navground, core, SensingState, init_buffer))
      .def("init_buffer",
           py::overload_cast<const std::string &, const BufferDescription &>(
               &SensingState::init_buffer),
           py::arg("key"), py::arg("description"),
           py::return_value_policy::automatic_reference,
           DOC(navground, core, SensingState, init_buffer, 2))
      .def("init_buffer",
           py::overload_cast<const std::string &, const BufferData &>(
               &SensingState::init_buffer),
           py::arg("key"), py::arg("data"),
           py::return_value_policy::automatic_reference,
           DOC(navground, core, SensingState, init_buffer, 3))
#endif
      .def(
          "init_buffer",
          [](SensingState &state, const std::string &key,
             const std::optional<BufferDescription> &description = std::nullopt,
             const py::object &data = py::none()) -> Buffer * {
            if (state.get_buffers().count(key)) {
              return nullptr;
            }
            auto buffer = make_buffer(description, data);
            state.set_buffer(key, std::move(buffer));
            return state.get_buffer(key);
          },
          py::arg("key"), py::arg("description") = std::nullopt,
          py::arg("data") = py::none(),
          py::return_value_policy::automatic_reference,
          R"doc(
Initializes a buffer.

:param key: The key
:type key: str
:param description: The description
:type description: :py:class:`navground.core.BufferDescription` | None
:param data: Optional data to initialize the buffer. If provided, 
             it will overwrite data initialized from the description.
             Else the buffer will be initialized with zeros.
             Accept any type supporting the buffer protocol.
:type data: object
:return: The buffer if successfully initialized else a null pointer
:rtype: :py:class:`navground.core.Buffer` | None
)doc")
      .def("get_buffer", &SensingState::get_buffer, py::arg("key"),
           py::return_value_policy::automatic_reference,
           DOC(navground, core, SensingState, get_buffer))
      .def("set_buffer", &SensingState::set_buffer, py::arg("key"),
           py::arg("buffer"), DOC(navground, core, SensingState, set_buffer))
      .def_property("buffers", &SensingState::get_buffers, nullptr,
                    DOC(navground, core, SensingState, property_buffers))
      .def(py::pickle(
          [](const SensingState &value) {
            py::dict rs;
            for (const auto &[key, buffer] : value.get_buffers()) {
              rs[py::str(key)] = buffer;
            }
            return py::make_tuple(rs);
          },
          [](py::tuple v) {
            SensingState state;
            for (const auto &[key, value] : py::cast<py::dict>(v[0])) {
              state.set_buffer(py::cast<std::string>(key),
                               py::cast<Buffer>(value));
            };
            return state;
          }));

  py::class_<GridMap, std::shared_ptr<GridMap>>(m, "GridMap",
                                                DOC(navground, core, GridMap))
      .def(py::init<unsigned, unsigned, double, const Vector2 &>(),
           py::arg("width"), py::arg("height"), py::arg("resolution"),
           py::arg("origin") = Vector2::Zero(),
           DOC(navground, core, GridMap, GridMap))
      .def_property("origin", &GridMap::get_origin, &GridMap::set_origin,
                    DOC(navground, core, GridMap, property_origin))
      .def_property("center", &GridMap::get_center, &GridMap::set_center,
                    DOC(navground, core, GridMap, property_center))
      .def_property("resolution", &GridMap::get_resolution,
                    &GridMap::set_resolution,
                    DOC(navground, core, GridMap, property_resolution))
      .def_property("width", &GridMap::get_width, &GridMap::set_width,
                    DOC(navground, core, GridMap, property_width))
      .def_property("height", &GridMap::get_height, &GridMap::set_height,
                    DOC(navground, core, GridMap, property_height))
      .def_property("top_right", &GridMap::get_top_right, nullptr,
                    DOC(navground, core, GridMap, property_top_right))
      .def_property("bottom_left", &GridMap::get_bottom_left, nullptr,
                    DOC(navground, core, GridMap, property_bottom_left))
      // Non const map
      .def_property("map", py::overload_cast<>(&GridMap::get_map), nullptr,
                    DOC(navground, core, GridMap, property_map))
      .def("get_position_of_cell", &GridMap::get_position_of_cell,
           py::arg("cell"), DOC(navground, core, GridMap, get_position_of_cell))
      .def("get_possible_cell_at_position",
           &GridMap::get_possible_cell_at_position, py::arg("position"),
           DOC(navground, core, GridMap, get_possible_cell_at_position))
      .def("contains_point", &GridMap::contains_point, py::arg("point"),
           DOC(navground, core, GridMap, contains_point))
      .def("move_center", &GridMap::move_center, py::arg("position"),
           py::arg("value") = 128, py::arg("snap") = true,
           DOC(navground, core, GridMap, move_center))
      .def("move_origin", &GridMap::move_origin, py::arg("position"),
           py::arg("value") = 128, py::arg("snap") = true,
           DOC(navground, core, GridMap, move_origin))
      .def("set_value", &GridMap::set_value, py::arg("value"),
           DOC(navground, core, GridMap, set_value))
      .def("set_value_of_cell", &GridMap::set_value_of_cell, py::arg("cell"),
           py::arg("value"), DOC(navground, core, GridMap, set_value_of_cell))
      .def("set_value_at_point", &GridMap::set_value_at_point, py::arg("point"),
           py::arg("value"), DOC(navground, core, GridMap, set_value_at_point))
      .def("set_value_in_rectangle", &GridMap::set_value_in_rectangle,
           py::arg("bottom_left"), py::arg("width"), py::arg("height"),
           py::arg("value"),
           DOC(navground, core, GridMap, set_value_in_rectangle))
      .def("set_value_in_disc", &GridMap::set_value_in_disc, py::arg("center"),
           py::arg("radius"), py::arg("value"),
           DOC(navground, core, GridMap, set_value_in_disc))
      .def("set_value_on_line", &GridMap::set_value_on_line, py::arg("p1"),
           py::arg("p2"), py::arg("value"),
           DOC(navground, core, GridMap, set_value_on_line))
      .def("set_value_between_cells", &GridMap::set_value_between_cells,
           py::arg("c1"), py::arg("c2"), py::arg("value"),
           DOC(navground, core, GridMap, set_value_between_cells))
      .def("get_cell_at_position", &GridMap::get_cell_at_position,
           py::arg("position"), py::arg("clamp"),
           DOC(navground, core, GridMap, get_cell_at_position))
      .def("raytrace_between_cells", &GridMap::raytrace_between_cells,
           py::arg("at"), py::arg("c1"), py::arg("c2"),
           py::arg("max_length") = std::numeric_limits<unsigned int>::max(),
           py::arg("min_length") = 0,
           DOC(navground, core, GridMap, raytrace_between_cells));

  py::class_<HLBehavior, Behavior, std::shared_ptr<HLBehavior>> hl(
      m, "HLBehavior", DOC(navground, core, HLBehavior));
  hl.def(py::init<std::shared_ptr<Kinematics>, ng_float_t>(),
         py::arg("kinematics") = py::none(), py::arg("radius") = 0,
         DOC(navground, core, HLBehavior, HLBehavior))
      .def_property("eta", &HLBehavior::get_eta, &HLBehavior::set_eta,
                    DOC(navground, core, HLBehavior, property_eta))
      .def_property("tau", &HLBehavior::get_tau, &HLBehavior::set_tau,
                    DOC(navground, core, HLBehavior, property_tau))
      .def_property("aperture", &HLBehavior::get_aperture,
                    &HLBehavior::set_aperture,
                    DOC(navground, core, HLBehavior, property_aperture))
      .def_property("resolution", &HLBehavior::get_resolution,
                    &HLBehavior::set_resolution,
                    DOC(navground, core, HLBehavior, property_resolution))
      .def_property(
          "angular_resolution", &HLBehavior::get_angular_resolution, nullptr,
          DOC(navground, core, HLBehavior, property_angular_resolution))
      .def_property("epsilon", &HLBehavior::get_epsilon,
                    &HLBehavior::set_epsilon,
                    DOC(navground, core, HLBehavior, property_epsilon))
      .def_property("barrier_angle", &HLBehavior::get_barrier_angle,
                    &HLBehavior::set_barrier_angle,
                    DOC(navground, core, HLBehavior, property_barrier_angle))
      .def("get_collision_distance", &HLBehavior::get_collision_distance,
           py::arg("assuming_static") = false, py::arg("speed") = std::nullopt,
           DOC(navground, core, HLBehavior, get_collision_distance));

  py::class_<ORCABehavior::Line>(m, "ORCALine")
      .def_readonly("point", &ORCABehavior::Line::point,
                    DOC(navground, core, ORCABehavior, Line, point))
      .def_readonly("direction", &ORCABehavior::Line::direction,
                    DOC(navground, core, ORCABehavior, Line, direction));

  py::class_<ORCABehavior, Behavior, std::shared_ptr<ORCABehavior>> orca(
      m, "ORCABehavior", DOC(navground, core, ORCABehavior));
  orca.def(py::init<std::shared_ptr<Kinematics>, ng_float_t>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0,
           DOC(navground, core, ORCABehavior, ORCABehavior))
      .def_property("lines", &ORCABehavior::get_lines, nullptr,
                    DOC(navground, core, ORCABehavior, get_lines))
      .def_property(
          "max_number_of_neighbors", &ORCABehavior::get_max_number_of_neighbors,
          &ORCABehavior::set_max_number_of_neighbors,
          DOC(navground, core, ORCABehavior, property_max_number_of_neighbors))
      .def_property("treat_obstacles_as_agents",
                    &ORCABehavior::get_treat_obstacles_as_agents,
                    &ORCABehavior::set_treat_obstacles_as_agents,
                    DOC(navground, core, ORCABehavior,
                        property_treat_obstacles_as_agents))
      .def_property("time_horizon", &ORCABehavior::get_time_horizon,
                    &ORCABehavior::set_time_horizon,
                    DOC(navground, core, ORCABehavior, property_time_horizon))
      .def_property(
          "static_time_horizon", &ORCABehavior::get_static_time_horizon,
          &ORCABehavior::set_static_time_horizon,
          DOC(navground, core, ORCABehavior, property_static_time_horizon))
      .def_property(
          "is_using_effective_center", &ORCABehavior::is_using_effective_center,
          &ORCABehavior::should_use_effective_center,
          DOC(navground, core, ORCABehavior, is_using_effective_center));

  py::class_<HRVOBehavior, Behavior, std::shared_ptr<HRVOBehavior>> hrvo(
      m, "HRVOBehavior", DOC(navground, core, HRVOBehavior));
  hrvo.def(py::init<std::shared_ptr<Kinematics>, ng_float_t>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0,
           DOC(navground, core, HRVOBehavior, HRVOBehavior))
      .def_property(
          "max_number_of_neighbors", &HRVOBehavior::get_max_number_of_neighbors,
          &HRVOBehavior::set_max_number_of_neighbors,
          DOC(navground, core, HRVOBehavior, property_max_number_of_neighbors))
      .def_property(
          "uncertainty_offset", &HRVOBehavior::get_uncertainty_offset,
          &HRVOBehavior::set_uncertainty_offset,
          DOC(navground, core, HRVOBehavior, property_uncertainty_offset));

  py::class_<DummyBehavior, Behavior, std::shared_ptr<DummyBehavior>> dummy(
      m, "DummyBehavior", DOC(navground, core, DummyBehavior));
  dummy
      .def(py::init<std::shared_ptr<Kinematics>, ng_float_t>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0,
           DOC(navground, core, Behavior, Behavior))
      .def("set_environment_state", &DummyBehavior::set_environment_state,
           py::arg("state"),
           DOC(navground, core, DummyBehavior, set_environment_state))
      .def_property("environment_state_type",
                    &DummyBehavior::get_environment_state_type,
                    &DummyBehavior::set_environment_state_type,
                    DOC(navground, core, DummyBehavior, property,
                        environment_state_type));

  py::class_<Action, std::shared_ptr<Action>> action(
      m, "Action", DOC(navground, core, Action));

  py::enum_<Action::State>(action, "State", DOC(navground, core, Action_State))
      .value("idle", Action::State::idle,
             DOC(navground, core, Action_State, idle))
      .value("running", Action::State::running,
             DOC(navground, core, Action_State, running))
      .value("failure", Action::State::failure,
             DOC(navground, core, Action_State, failure))
      .value("success", Action::State::success,
             DOC(navground, core, Action_State, success));

  action
      .def_readonly("state", &Action::state,
                    DOC(navground, core, Action, state))
      .def_property("done", &Action::done, nullptr,
                    DOC(navground, core, Action, done))
      .def_property("running", &Action::running, nullptr,
                    DOC(navground, core, Action, running))
      .def("abort", &Action::abort, DOC(navground, core, Action, abort))
      .def_readwrite("running_cb", &Action::running_cb,
                     DOC(navground, core, Action, running_cb))
      .def_readwrite("done_cb", &Action::done_cb,
                     DOC(navground, core, Action, done_cb));

  py::class_<Controller>(m, "Controller", DOC(navground, core, Controller))
      .def(py::init<std::shared_ptr<Behavior>>(), py::arg("behavior") = nullptr,
           DOC(navground, core, Controller, Controller))
      .def_property("state", &Controller::get_state, nullptr,
                    DOC(navground, core, Controller, property_state))
      .def_property("idle", &Controller::idle, nullptr,
                    DOC(navground, core, Controller, idle))
      .def_property("is_still", &Controller::is_still, nullptr,
                    DOC(navground, core, Controller, is_still))
      .def_property("behavior", &Controller::get_behavior,
                    &Controller::set_behavior,
                    DOC(navground, core, Controller, property_behavior))
      .def_property("speed_tolerance", &Controller::get_speed_tolerance,
                    &Controller::set_speed_tolerance,
                    DOC(navground, core, Controller, property_speed_tolerance))
      .def_property(
          "angular_speed_tolerance", &Controller::get_angular_speed_tolerance,
          &Controller::set_angular_speed_tolerance,
          DOC(navground, core, Controller, property_angular_speed_tolerance))
      .def_property("cmd_frame", &Controller::get_cmd_frame,
                    &Controller::set_cmd_frame,
                    DOC(navground, core, Controller, property_cmd_frame))
      .def("follow_manual_cmd", &Controller::follow_manual_cmd, py::arg("cmd"),
           DOC(navground, core, Controller, follow_manual_cmd))
      .def("follow_path", &Controller::follow_path, py::arg("path"),
           py::arg("tolerance"), DOC(navground, core, Controller, follow_path))
      .def("go_to_position", &Controller::go_to_position, py::arg("position"),
           py::arg("tolerance"), py::arg("along_path") = py::none(),
           DOC(navground, core, Controller, go_to_position))
      .def("go_to_pose", &Controller::go_to_pose, py::arg("pose"),
           py::arg("position_tolerance"), py::arg("orientation_tolerance"),
           py::arg("along_path") = py::none(),
           DOC(navground, core, Controller, go_to_pose))
      .def("follow_point", &Controller::follow_point, py::arg("point"),
           DOC(navground, core, Controller, follow_point))
      .def("follow_pose", &Controller::follow_pose, py::arg("pose"),
           DOC(navground, core, Controller, follow_pose))
      .def("follow_direction", &Controller::follow_direction,
           py::arg("direction"),
           DOC(navground, core, Controller, follow_direction))
      .def("follow_velocity", &Controller::follow_velocity, py::arg("velocity"),
           DOC(navground, core, Controller, follow_velocity))
      .def("follow_twist", &Controller::follow_twist, py::arg("twist"),
           DOC(navground, core, Controller, follow_twist))
      .def("update", &Controller::update, py::arg("time_step"),
           DOC(navground, core, Controller, update))
      .def("set_cmd_cb", &Controller::set_cmd_cb, py::arg("callback"),
           DOC(navground, core, Controller, set_cmd_cb))
      .def("stop", &Controller::stop, DOC(navground, core, Controller, stop));

  py::class_<CollisionComputation>(m, "CollisionComputation",
                                   DOC(navground, core, CollisionComputation))
      .def(py::init<>(),
           DOC(navground, core, CollisionComputation, CollisionComputation))
      .def(
          "setup",
          py::overload_cast<Pose2, ng_float_t, const std::vector<LineSegment> &,
                            const std::vector<Disc> &,
                            const std::vector<Neighbor> &>(
              &CollisionComputation::setup),
          py::arg_v("pose", Pose2(),
                    "navground.core._navground.Pose2((0, 0), 0)"),
          py::arg("margin") = 0,
          py::arg("line_segments") = std::vector<LineSegment>(),
          py::arg("static_discs") = std::vector<Disc>(),
          py::arg("dynamic_discs") = std::vector<Neighbor>(),
          DOC(navground, core, CollisionComputation, setup, 2))

      .def("static_free_distance",
           py::overload_cast<ng_float_t, ng_float_t, bool>(
               &CollisionComputation::static_free_distance),
           py::arg("angle"), py::arg("max_distance"),
           py::arg("include_neighbors") = true,
           DOC(navground, core, CollisionComputation, static_free_distance))

      .def("dynamic_free_distance",
           &CollisionComputation::dynamic_free_distance, py::arg("angle"),
           py::arg("max_distance"), py::arg("speed"),
           DOC(navground, core, CollisionComputation, dynamic_free_distance))

      .def("get_angles_for_sector",
           &CollisionComputation::get_angles_for_sector, py::arg("from_angle"),
           py::arg("length"), py::arg("resolution"),
           DOC(navground, core, CollisionComputation, get_angles_for_sector))
      .def("get_free_distance_for_sector",
           &CollisionComputation::get_free_distance_for_sector,
           py::arg("from_angle"), py::arg("length"), py::arg("resolution"),
           py::arg("max_distance"), py::arg("dynamic"), py::arg("speed") = 0,
           DOC(navground, core, CollisionComputation,
               get_free_distance_for_sector))
      .def("get_contour_for_sector",
           &CollisionComputation::get_free_distance_for_sector,
           py::arg("from_angle"), py::arg("length"), py::arg("resolution"),
           py::arg("max_distance"), py::arg("dynamic"), py::arg("speed") = 0,
           DOC(navground, core, CollisionComputation, get_contour_for_sector));

  py::class_<CachedCollisionComputation, CollisionComputation>(
      m, "CachedCollisionComputation",
      DOC(navground, core, CachedCollisionComputation))
      .def(py::init<>(), DOC(navground, core, CachedCollisionComputation,
                             CachedCollisionComputation))
      .def_property(
          "resolution", &CachedCollisionComputation::get_resolution,
          &CachedCollisionComputation::set_resolution,
          DOC(navground, core, CachedCollisionComputation, property_resolution))
      .def_property(
          "min_angle", &CachedCollisionComputation::get_min_angle,
          &CachedCollisionComputation::set_min_angle,
          DOC(navground, core, CachedCollisionComputation, property_min_angle))
      .def_property(
          "length", &CachedCollisionComputation::get_length,
          &CachedCollisionComputation::set_length,
          DOC(navground, core, CachedCollisionComputation, property_length))
      .def_property("max_distance",
                    &CachedCollisionComputation::get_max_distance,
                    &CachedCollisionComputation::set_max_distance,
                    DOC(navground, core, CachedCollisionComputation,
                        property_max_distance))
      .def_property(
          "speed", &CachedCollisionComputation::get_speed,
          &CachedCollisionComputation::set_speed,
          DOC(navground, core, CachedCollisionComputation, property_speed))
      .def("get_free_distance", &CachedCollisionComputation::get_free_distance,
           py::arg("dynamic"),
           DOC(navground, core, CachedCollisionComputation, get_free_distance));

  py::class_<HasAttributes, std::shared_ptr<HasAttributes>>(
      m, "HasAttributes", DOC(navground, core, HasAttributes))
      .def("get", &HasAttributes::get, py::arg("name"),
           DOC(navground, core, HasAttributes, get))
      .def("has", &HasAttributes::has, py::arg("name"),
           DOC(navground, core, HasAttributes, has))
      .def("set", &HasAttributes::set, py::arg("name"), py::arg("value"),
           DOC(navground, core, HasAttributes, set))
      .def("clear", &HasAttributes::clear,
           DOC(navground, core, HasAttributes, clear))
      .def_property("attributes", &HasAttributes::get_attributes,
                    &HasAttributes::set_attributes,
                    DOC(navground, core, HasAttributes, property, attributes));

  py::class_<BehaviorGroup, PyBehaviorGroup, std::shared_ptr<BehaviorGroup>>(
      m, "BehaviorGroup", DOC(navground, core, BehaviorGroup))
      .def(py::init<>())
      .def(
          "compute_cmds",
          [](PyBehaviorGroup &group, ng_float_t time_step) {
            return group.compute_cmds(time_step);
          },
          py::arg("time_step"),
          DOC(navground, core, BehaviorGroup, compute_cmds))
      .def_property("members", &BehaviorGroup::get_members, nullptr,
                    DOC(navground, core, BehaviorGroup, property_members))
      .def_property("size", &BehaviorGroup::size, nullptr,
                    DOC(navground, core, BehaviorGroup, property_size));

  py::class_<BehaviorGroupMember, PyBehaviorGroupMember, Behavior,
             std::shared_ptr<BehaviorGroupMember>>(
      m, "BehaviorGroupMember", DOC(navground, core, BehaviorGroupMember))
      .def(py::init<std::shared_ptr<Kinematics>, ng_float_t>(),
           py::arg("kinematics") = py::none(), py::arg("radius") = 0)
      .def(
          "make_group",
          [](const PyBehaviorGroupMember &group) {
            return group.make_group_py();
          },
          DOC(navground, core, BehaviorGroupMember, make_group))
      .def(
          "get_groups",
          [](const PyBehaviorGroupMember &group) {
            return group.get_groups_py();
          },
          DOC(navground, core, BehaviorGroupMember, get_groups))
      .def(
          "get_group_hash",
          [](const PyBehaviorGroupMember &group) {
            return group.get_group_hash();
          },
          DOC(navground, core, BehaviorGroupMember, get_group_hash))
      .def_property("group", &BehaviorGroupMember::get_group, nullptr,
                    DOC(navground, core, BehaviorGroup, property_group));

  m.def("load_plugins", &load_plugins, py::arg("plugins") = py::set(),
        py::arg("directories") = py::dict(), py::arg("include_default") = true,
        DOC(navground, core, load_plugins));

  m.def("get_loaded_plugins", &get_loaded_plugins,
        DOC(navground, core, get_loaded_plugins));

  // add [partial] pickle support
  pickle_via_yaml<PyBehaviorModulation>(behavior_modulation);
  pickle_via_yaml<PyBehavior>(behavior);
  pickle_via_yaml<PyBehavior>(hl);
  pickle_via_yaml<PyBehavior>(orca);
  pickle_via_yaml<PyBehavior>(hrvo);
  pickle_via_yaml<PyBehavior>(dummy);
  pickle_via_yaml<PyKinematics>(kinematics);
  pickle_via_yaml<PyKinematics>(omni);
  pickle_via_yaml<PyKinematics>(wk);
  pickle_via_yaml<PyKinematics>(ahead);
  pickle_via_yaml<PyKinematics>(wk2);
  pickle_via_yaml<PyKinematics>(wk4);
  pickle_via_yaml<PyKinematics>(dwk2);
  pickle_via_yaml<PyKinematics>(bk);
  pickle_via_yaml<PyBehaviorModulation>(relaxation);
  pickle_via_yaml<PyBehaviorModulation>(limit_acceleration);
  pickle_via_yaml<PyBehaviorModulation>(limit_twist);
  pickle_via_yaml<PyBehaviorModulation>(motor_pid);
  pickle_via_yaml_native<SocialMargin>(social_margin);

  m.def("to_absolute", &to_absolute, py::arg("value"), py::arg("reference"),
        DOC(navground, core, to_absolute));
  m.def("to_relative", &to_relative, py::arg("value"), py::arg("reference"),
        DOC(navground, core, to_relative));
  m.def("to_absolute_point", &to_absolute_point, py::arg("value"),
        py::arg("reference"), DOC(navground, core, to_absolute_point));
  m.def("to_relative_point", &to_relative_point, py::arg("value"),
        py::arg("reference"), DOC(navground, core, to_relative_point));
  m.def("normalize_angle", &normalize_angle, py::arg("angle"),
        DOC(navground, core, normalize_angle));
  m.def("orientation_of", &orientation_of, py::arg("vector"),
        DOC(navground, core, orientation_of));
  m.def("unit", &unit, py::arg("angle"), DOC(navground, core, unit));
  m.def("rotate", &rotate, py::arg("vector"), py::arg("angle"),
        DOC(navground, core, rotate));
  m.def("clamp_norm", &clamp_norm, py::arg("vector"), py::arg("max_length"),
        DOC(navground, core, clamp_norm));

  m.def(
      "bundle_schema", []() { return YAML::to_py(bundle_schema()); },
      R"doc(
Returns the bundle json-schema

:return: json-schema
:rtype: :py:type:`dict[str, typing.Any]`
)doc");

  m.def("get_build_info", &navground::core_py::build_info,
        DOC(navground, core, get_build_info));
  m.def("get_build_dependencies", &build_dependencies_core_py,
        "Gets the build dependencies",
        DOC(navground, core, get_build_dependencies));
  m.def("get_plugins_dependencies", &get_plugins_dependencies,
        DOC(navground, core, get_plugins_dependencies));

  m.def("make_properties", make_properties_py, py::arg("cls"),
        py::arg("owner") = "");

  m.def(
      "convert",
      [](const Property::Field &value, const std::string &scalar_type_name,
         bool is_list) -> std::optional<Property::Field> {
        const auto fn = get_converter(scalar_type_name, is_list);
        if (fn) {
          return std::nullopt;
        }
        return (*fn)(value);
      },
      py::arg("value"), py::arg("scalar_type_name"), py::arg("is_list"), "");

  m.def("get_scalar_type_name", &get_scalar_type_name, py::arg("type_name"));

  m.def("get_type_name_with_scalar", &get_type_name_with_scalar,
        py::arg("scalar_type_name"), py::arg("is_list"));

  m.def(
      "uses_doubles",
      []() { return std::is_same<ng_float_t, float>::value == false; },
      "Returns whether navground has been compiled to use floats or doubles");
}
