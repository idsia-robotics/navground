#ifndef NAVGROUND_CORE_PY_COLLECTION_H
#define NAVGROUND_CORE_PY_COLLECTION_H

/**
 * When we create a Python object from C++,
 * for instance evaluating a function like in:
 *
 * 		auto py_object = py_cls.attr("__call__")();
 * 		auto cpp_object = py_object.cast<std::shared_ptr<CppClass>>();
 *
 * The python object get de-allocated exiting the scope, unless we retain a
 * reference to it. To do that, we can use the functions
 * in this files, which attaches the python object to another one, extending
 * therefore its lifetime, in a private python list attribute `__py_<name>`.
 */

#include <pybind11/pybind11.h>

#include <memory>
#include <string>

namespace py = pybind11;

inline bool has_py_collection(const py::object &obj, const std::string &name) {
  py::object key = py::str("__py_" + name);
  return py::hasattr(obj, key);
}

inline py::list get_py_collection(const py::object &obj,
                                  const std::string &name) {
  py::object key = py::str("__py_" + name);
  if (!py::hasattr(obj, key)) {
    obj.attr(key) = py::list();
  }
  return static_cast<py::list>(py::getattr(obj, key));
}

inline void add_py_item(const py::object &obj, const py::object value,
                        const std::string &name) {
  get_py_collection(obj, name).append(value);
}

inline void remove_py_item(const py::object &obj, const py::object value,
                           const std::string &name) {
  try {
    get_py_collection(obj, name).attr("remove")(value);
  } catch (std::exception &) {
  }
}

inline bool has_py_item(const py::object &obj, const py::object value,
                        const std::string &name) {
  return get_py_collection(obj, name).attr("count")(value).cast<int>() > 0;
}

inline void clear_collection_py(const py::object &obj,
                                const std::string &name) {
  if (has_py_collection(obj, name)) {
    get_py_collection(obj, name).attr("clear")();
  }
}

inline bool has_py_map(const py::object &obj, const std::string &name) {
  return has_py_collection(obj, name);
}

inline py::dict get_py_map(const py::object &obj, const std::string &name) {
  py::object key = py::str("__py_" + name);
  if (!py::hasattr(obj, key)) {
    obj.attr(key) = py::dict();
  }
  return static_cast<py::dict>(py::getattr(obj, key));
}

inline void set_py_key(const py::object &obj, const std::string &key,
                       const py::object &value, const std::string &name) {
  get_py_map(obj, name)[key.c_str()] = value;
}

inline void remove_py_key(const py::object &obj, const std::string &key,
                          const std::string &name) {
  auto dict = get_py_map(obj, name);
  if (dict.contains(key)) {
    dict.attr("pop")(key);
  }
}

inline void clear_py_map(const py::object &obj, const std::string &name) {
  if (has_py_map(obj, name)) {
    get_py_map(obj, name).attr("clear")();
  }
}

#endif // NAVGROUND_CORE_PY_COLLECTION_H