#ifndef NAVGROUND_CORE_PY_REGISTER_H
#define NAVGROUND_CORE_PY_REGISTER_H

#include <pybind11/pybind11.h>
#include <pybind11/pytypes.h>

#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core_py/property.h"
#include "navground/core_py/yaml.h"

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
struct PyHasRegister : virtual public navground::core::HasRegister<T> {
  /* Inherit the constructors */
  using navground::core::HasRegister<T>::HasRegister;
  using Factory = py::object;
  // using navground::core::HasRegister<T>::get_type;
  using navground::core::HasRegister<T>::type_properties;
  using navground::core::HasRegister<T>::type_schema;
  using C = py::object;
  // using navground::core::HasRegister<T>::get_properties;

  // TODO(Jerome): may be unsafe ...
  // pybind11 does guard the GIL when calling python functions
#if 0
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

#else
  void decode(const YAML::Node &node) override {
    auto fn = py_decode();
    if (fn) {
      fn(YAML::to_py(node));
    }
  }

  void encode(YAML::Node &node) const override {
    auto fn = py_encode();
    if (fn) {
      const py::object obj = fn(YAML::to_py(node));
      if (!obj.is_none()) {
        node = YAML::from_py(obj);
      }
    }
  };
#endif

  virtual py::function py_decode() const { return py::function(); }

  virtual py::function py_encode() const { return py::function(); }

  inline static std::map<std::string, Factory> factory = {};
  inline static std::map<std::string, py::object> schema = {};

  static void register_type_py(const std::string &name, const py::object &cls) {
    // if (factory.count(name)) {
    //   std::cerr << "Type " << name << " already registered" << std::endl;
    // }
    factory[name] = cls;
    cls.attr("_type") = name;
    type_properties()[name] = Properties{};
  }

  // static void set_schema_py(const std::string &name,
  //                           const py::function &schema_fn) {
  //   schema[name] = schema_fn;
  //   type_schema()[name] = [schema_fn](YAML::Node &node) {
  //     auto obj = YAML::to_py(node);
  //     schema_fn(obj);
  //     node = YAML::from_py(obj);
  //   };
  // }

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
      const auto obj = HasRegister<T>::make_type(name);
      return py::cast(obj);
    } catch (const std::exception &e) {
      std::cerr << e.what() << std::endl;
      return py::none();
    }
  }

  template<typename S>
  static py::object make_subtype(const std::string &name) {
    if (factory.count(name)) {
      return factory[name].attr("__call__")();
    }
    try {
      const auto obj = std::dynamic_pointer_cast<S>(HasRegister<T>::make_type(name));

      return py::cast(obj);
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

  static void register_properties_py(const py::object &cls) {
    if (!py::hasattr(cls, "_type")) {
      std::cerr << "Python class not registered yet" << std::endl;
      return;
    }
    std::string owner = cls.attr("__module__").cast<std::string>() + "." +
                        cls.attr("__name__").cast<std::string>();
    std::string type = cls.attr("_type").cast<std::string>();
    const auto py_property = py::module_::import("builtins").attr("property");
    const auto isinstance = py::module_::import("builtins").attr("isinstance");
    for (auto &item : cls.attr("__dict__").cast<py::dict>()) {
      const py::object v = py::cast<py::object>(item.second);
      if (isinstance(v, py_property).cast<bool>() && py::hasattr(v, "fget")) {
        const auto fget = v.attr("fget");
        if (py::hasattr(fget, "__default_value__")) {
          auto default_value =
              fget.attr("__default_value__").cast<Property::Field>();
          // If a list is empty, Python as no way to cast it to the correct
          // type. Therefore, we read the type from a extra `__scalar_value__`
          // field.
          if (is_empty_vector(default_value) &&
              py::hasattr(fget, "__scalar_value__")) {
            const auto scalar_value =
                fget.attr("__scalar_value__").cast<Property::Scalar>();
            default_value = std::visit(
                [](auto &&arg) -> Property::Field {
                  using S = std::decay_t<decltype(arg)>;
                  return std::vector<S>();
                },
                scalar_value);
          }
          const auto desc = fget.attr("__desc__").cast<std::string>();
          const auto deprecated_names = fget.attr("__deprecated_names__")
                                            .cast<std::vector<std::string>>();

          const auto schema =
              py::hasattr(v, "fget")
                  ? fget.attr("__schema__").cast<std::optional<py::function>>()
                  : std::nullopt;
          // py::print("Adding Property", item.first, "to type", type, "from
          // class", owner);
          add_property_py(type, owner, item.first.cast<std::string>(), v,
                          default_value, desc, schema, deprecated_names);
        }
      }
    }
  }

  static void register_schema_py(const py::object &cls) {
    if (!py::hasattr(cls, "_type")) {
      std::cerr << "Python class not registered yet" << std::endl;
      return;
    }
    std::string type = cls.attr("_type").cast<std::string>();
    const auto py_staticmethod =
        py::module_::import("builtins").attr("staticmethod");
    const auto isinstance = py::module_::import("builtins").attr("isinstance");
    for (auto &item : cls.attr("__dict__").cast<py::dict>()) {
      const py::object v = py::cast<py::object>(item.second);
      if (isinstance(v, py_staticmethod).cast<bool>() &&
          py::hasattr(v, "__is_schema__")) {
        // py::print("Setting schema for type", type);
        // set_schema_py(type, v);
        schema[type] = v;
        type_schema()[type] = YAML::from_schema_py(v);
      }
    }
  }

  static void
  add_property_py(const std::string &type, const std::string &owner,
                  const std::string &name, const py::object &py_property,
                  const Property::Field &default_value,
                  const std::string &description = "",
                  const std::optional<py::function> &schema = std::nullopt,
                  const std::vector<std::string> &deprecated_names = {}) {
    Property p = make_property_with_py_property(
        py_property, default_value, description, schema, deprecated_names);
    p.owner_type_name = owner;
    type_properties()[type][name] = std::move(p);
  }

  std::string get_type() const override {
    const auto type = HasRegister<T>::get_type();
    if (!type.empty()) {
      return type;
    }
    try {
      auto value = py::cast(this).attr("_type");
      return value.template cast<std::string>();
    } catch (const std::exception &) {
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
      .def_property_readonly_static(
          "__init_subclass__",
          [](py::object &cls) {
            return py::cpp_function([cls](const py::kwargs &kwargs) {
              if (kwargs.contains("name")) {
                const std::string name = py::cast<std::string>(kwargs["name"]);
                // py::print("will register", cls, "with name", name);
                PyRegister::register_type_py(name, cls);
                PyRegister::register_properties_py(cls);
                PyRegister::register_schema_py(cls);
              }
            });
          })
      // .def_static("_add_property", &PyRegister::add_property_py,
      //             py::arg("type"), py::arg("name"), py::arg("property"),
      //             py::arg("default_value"), py::arg("description") = "",
      //             py::arg("deprecated_names") = std::vector<std::string>())
      .def_property(
          "type", [](PyRegister *obj) { return obj->get_type(); }, nullptr,
          "The name associated to the type of an object"
          // DOC(navground, core, HasRegister, property_type)
          )
      .def_property(
          "properties", [](PyRegister *obj) { return obj->get_properties(); },
          nullptr, "The registered properties"
          // DOC(navground, core, HasRegister, property_properties)
          )
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
      .def_static("schema", &YAML::component_schema_py<T>,
                  py::arg("reference_register_schema") = true,
                  py::arg("type") = std::nullopt,
                  YAML::component_schema_py_doc())
      .def_static("register_schema", &YAML::register_schema_py<T>,
                  YAML::register_schema_py_doc())
      .def("dump", &YAML::dump<T>, YAML::dump_doc());
  // .def_static("_register_type", &PyRegister::register_type_py,
  //             py::arg("name"), py::arg("cls"))
  // .def_static("_register_schema", &PyRegister::register_schema_py,
  //             py::arg("name"), py::arg("schema"))
  // .def_static("_add_properties", &PyRegister::add_properties_py,
  //             py::arg("type"), py::arg("properties"));
}

#endif // NAVGROUND_CORE_PY_REGISTER_H
