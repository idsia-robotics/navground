#include "navground/sim/dataset.h"

#include <highfive/H5DataSpace.hpp>
#include <string>

#include "navground/core/utilities.h"

using namespace navground::sim;

void Dataset::save(const std::string &key, HighFive::Group &group) const {
  std::visit(
      [this, &group, &key](auto &&arg) {
        using T =
            typename std::remove_reference<decltype(arg)>::type::value_type;
        group.createDataSet<T>(key, HighFive::DataSpace(get_shape()))
            .write_raw(arg.data());
      },
      _data);
}

size_t Dataset::get_size(const Dataset::Data &data) {
  return std::visit([](auto &&arg) { return arg.size(); }, data);
}

size_t Dataset::get_shape_size(const Dataset::Shape &shape) {
  return std::accumulate(std::begin(shape), std::end(shape), 1,
                         std::multiplies<>{});
}

void Dataset::set_item_shape(const Shape &value) {
  _item_shape = value;
  _item_size = static_cast<unsigned>(get_shape_size(value));
}

bool Dataset::is_valid() const { return size() % _item_size == 0; }

Dataset::Shape Dataset::get_shape() const {
  Dataset::Shape shape = {_item_size ? size() / _item_size : 0};
  shape.insert(std::end(shape), std::begin(_item_shape), std::end(_item_shape));
  return shape;
}

void Dataset::reset() {
  std::visit([](auto &&arg) { return arg.clear(); }, _data);
}

void Dataset::append(const Dataset::Data &data) {
  std::visit(
      [this](auto &&arg) {
        using T =
            typename std::remove_reference<decltype(arg)>::type::value_type;
        append<T>(arg);
      },
      data);
}

void Dataset::append(const core::Buffer &buffer) {
  std::visit(
      [this](auto &&arg) {
        using T =
            typename std::remove_reference<decltype(arg)>::type::value_type;
        append<T>(arg);
      },
      buffer.get_data_container());
}

void Dataset::push(const Dataset::Scalar &data) {
  std::visit(
      [this](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        push<T>(arg);
      },
      data);
}

// std::shared_ptr<Dataset> Dataset::make(const core::Buffer &buffer) {
//   Dataset::Shape shape;
//   const auto &bshape = buffer.get_shape();
//   std::copy(bshape.begin(), bshape.end(), std::back_inserter(shape));
//   auto ds = std::make_shared<Dataset>(shape);
//   std::visit(
//       [&ds](auto &&arg) {
//         using T =
//             typename std::remove_reference<decltype(arg)>::type::value_type;
//         ds->set_dtype<T>();
//       },
//       buffer.get_data_container());
//   return ds;
// }

void Dataset::config_to_hold_buffer(const navground::core::Buffer &buffer) {
  Dataset::Shape shape;
  const auto &bshape = buffer.get_shape();
  std::copy(bshape.begin(), bshape.end(), std::back_inserter(shape));
  set_item_shape(shape);
  std::visit(
      [this](auto &&arg) {
        using T =
            typename std::remove_reference<decltype(arg)>::type::value_type;
        set_dtype<T>();
      },
      buffer.get_data_container());
}