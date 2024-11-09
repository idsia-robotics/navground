#ifndef NAVGROUND_CORE_PY_BUFFER_H
#define NAVGROUND_CORE_PY_BUFFER_H

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

#include "navground/core/buffer.h"

namespace py = pybind11;
using namespace navground::core;

inline std::string type_from_dtype(const py::dtype &value) {
  return std::string(1, value.kind()) + std::to_string(value.alignment());
}

inline py::dtype make_dtype(const py::object &value) {
  py::module_ np = py::module_::import("numpy");
  return np.attr("dtype")(value);
}

inline py::dtype dtype_from_buffer(const Buffer &buffer) {
  py::module_ np = py::module_::import("numpy");
  return np.attr("dtype")(buffer.get_description().type);
}

inline py::array get_array_from_buffer(const Buffer &buffer) {
  const auto &shape = buffer.get_description().shape;
  const auto &strides = buffer.get_description().get_strides();
  const py::dtype dtype(buffer.get_description().type);
  return py::array(dtype, shape, strides, buffer.get_ptr());
}

inline BufferShape shape_from_array(const py::array &value) {
  return value.request().shape;
}

inline std::string type_from_array(const py::array &value) {
  return type_from_dtype(value.dtype());
}

inline BufferShape shape_from_buffer(const py::buffer &value) {
  return value.request().shape;
}

inline std::string type_from_buffer(const py::buffer &value) {
  return type_from_dtype(make_dtype(py::cast(value.request().format)));
}

inline bool set_buffer_from_buffer(Buffer &buffer, const py::buffer &value,
                                   bool force = false) {
  return buffer.set_ptr(value.request().ptr, shape_from_buffer(value),
                        type_from_buffer(value), force);
}

inline Buffer
make_buffer(const std::optional<BufferDescription> &description = std::nullopt,
            const py::object &data = py::none()) {
  Buffer buffer;
  if (description) {
    buffer.set_description(*description, true);
  }
  if (!data.is_none()) {
    try {
      auto bdata = py::cast<py::buffer>(data);
      set_buffer_from_buffer(buffer, bdata, true);
    } catch (const std::exception &) {
      throw pybind11::type_error(
          "Data has invalid type that does not support the buffer protocol.");
    }
  }
  return buffer;
}

#endif // NAVGROUND_CORE_PY_BUFFER_H