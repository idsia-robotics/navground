#ifndef NAVGROUND_CORE_PY_REGISTER_H
#define NAVGROUND_CORE_PY_REGISTER_H

#include <pybind11/pybind11.h>

#include "navground/core/property.h"
#include "navground/core/register.h"

namespace py = pybind11;

using navground::core::HasProperties;
using navground::core::HasRegister;
using navground::core::Properties;
using navground::core::Property;

#define OVERRIDE_DECODE                                                        \
  py::function py_decode() const override {                                    \
    return py::get_override(this, "decode");                                   \
  }
#define OVERRIDE_ENCODE                                                        \
  py::function py_encode() const override {                                    \
    return py::get_override(this, "encode");                                   \
  }

template <typename T>
struct PyHasRegister : public virtual navground::core::HasRegister<T> {
  /* Inherit the constructors */
  using navground::core::HasRegister<T>::HasRegister;
  using Factory = py::object;
  using navground::core::HasRegister<T>::get_type;
  using navground::core::HasRegister<T>::type_properties;
  using C = py::object;

  // TODO(Jerome): may be unsafe ...
  // pybind11 does guard the GIL when calling python functions
  void decode(const YAML::Node &node) override {
    auto fn = py_decode();
    if (fn) {
      YAML::Emitter out;
      out << node;
      const auto value = std::string(out.c_str());
      fn(value);
    }
  }

  void encode(YAML::Node &node) const override {
    auto fn = py_encode();
    if (fn) {
      const py::object py_value = fn();
      const std::string value = py_value.cast<std::string>();
      if (value.size()) {
        const YAML::Node data = YAML::Load(value);
        for (const auto &kv : data) {
          node[kv.first] = kv.second;
        }
      }
    }
  };

  virtual py::function py_decode() const { return py::function(); }

  virtual py::function py_encode() const { return py::function(); }

  inline static std::map<std::string, Factory> factory = {};

  static void register_type_py(const std::string &name, const py::object &cls) {
    // if (factory.count(name)) {
    //   std::cerr << "Type " << name << " already registered" << std::endl;
    // }
    factory[name] = cls;
    type_properties()[name] = Properties{};
  }

  static std::vector<std::string> types() {
    std::vector rs = HasRegister<T>::types();
    for (const auto &[k, v] : factory)
      rs.push_back(k);
    return rs;
  }

  static py::object make_type(const std::string &name) {
    if (factory.count(name)) {
      return factory[name].attr("__call__")();
    }
    try {
      return py::cast(HasRegister<T>::make_type(name));
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      return py::none();
    }
  }

  static bool has_type(const std::string &name) {
    return factory.count(name) || HasRegister<T>::has_type(name);
  }

  static void add_properties_py(const std::string &type,
                                const Properties &properties) {
    for (const auto &[k, v] : properties) {
      type_properties()[type][k] = v;
    }
  }

  static void
  add_property_py(const std::string &type, const std::string &name,
                  const py::object &py_property,
                  const Property::Field &default_value,
                  const std::string &description = "",
                  const std::vector<std::string> &deprecated_names = {}) {
    std::string type_name = std::visit(
        [](auto &&arg) {
          using V = std::decay_t<decltype(arg)>;
          return std::string(get_type_name<V>());
        },
        default_value);
    Property p;
    py::object py_getter = py_property.attr("fget");
    if (!py_getter.is_none()) {
      p.getter = [py_getter](const HasProperties *obj) {
        auto py_value = py_getter.attr("__call__")(py::cast(obj));
        return py_value.cast<Property::Field>();
      };
    } else {
      p.getter = nullptr;
    }
    py::object py_setter = py_property.attr("fset");
    if (!py_setter.is_none()) {
      p.setter = [py_setter](HasProperties *obj, const Property::Field &value) {
        py_setter.attr("__call__")(py::cast(obj), py::cast(value));
      };
    } else {
      p.setter = nullptr;
    }
    p.readonly = py_setter.is_none();
    p.default_value = default_value;
    p.type_name = type_name;
    p.description = description;
    p.owner_type_name = type;
    p.deprecated_names = deprecated_names;
    type_properties()[type][name] = std::move(p);
  }

  std::string get_type() const override {
    try {
      auto value = py::cast(this).attr("_type");
      return value.template cast<std::string>();
    } catch (const std::exception &e) {
      return "";
    }
  }
};

template <typename T>
void declare_register(py::module &m, const std::string &typestr) {
  using Register = HasRegister<T>;
  using PyRegister = PyHasRegister<T>;
  std::string pyclass_name = typestr + std::string("Register");
  py::class_<Register, PyRegister, std::shared_ptr<Register>>(
      m, pyclass_name.c_str())
      .def_static("_add_property", &PyRegister::add_property_py,
                  py::arg("type"), py::arg("name"), py::arg("property"),
                  py::arg("default_value"), py::arg("description") = "",
                  py::arg("deprecated_names") = std::vector<std::string>())
      .def_property_readonly_static(
          "types", [](py::object /* self */) { return PyRegister::types(); })
      .def_property_readonly_static(
          "type_properties",
          [](py::object /* self */) { return PyRegister::type_properties(); })
      .def_static("make_type", &PyRegister::make_type, py::arg("name"), R"doc(
Create an object of a sub-class selected by name.

:param type:
    The associated type name.

:return:
    An object of a registered sub-class or ``None`` in case the desired name is not found.)doc")
      .def_static("has_type", &PyRegister::has_type, py::arg("name"), R"doc(
Check whether a type name has been registered.

:param type:
    The associated type name.

:return:
    True if the type name has been registered.)doc")
      .def_static("_register_type", &PyRegister::register_type_py,
                  py::arg("name"), py::arg("cls"))
      .def_static("_add_properties", &PyRegister::add_properties_py,
                  py::arg("type"), py::arg("properties"));
}

#endif // NAVGROUND_CORE_PY_REGISTER_H
