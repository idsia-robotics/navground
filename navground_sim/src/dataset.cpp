  #include "navground/sim/dataset.h"

#include <highfive/H5DataSpace.hpp>
#include <string>

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
  _item_size = get_shape_size(value);
}

bool Dataset::is_valid() const { return size() % _item_size == 0; }

Dataset::Shape Dataset::get_shape() const {
  Dataset::Shape shape = {size() / _item_size};
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

void Dataset::push(const Dataset::Scalar &data) {
  std::visit(
      [this](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        push<T>(arg);
      },
      data);
}
