#include "navground/sim/probe.h"

#include <highfive/H5DataSpace.hpp>
#include <string>

using namespace navground::sim;

void Probe::save(const std::string &key, HighFive::Group &group) const {
  std::visit(
      [this, &group, &key](auto &&arg) {
        using T =
            typename std::remove_reference<decltype(arg)>::type::value_type;
        group.createDataSet<T>(key, HighFive::DataSpace(shape()))
            .write_raw(arg.data());
      },
      _data);
}

size_t Probe::size() const {
  return std::visit([](auto &&arg) { return arg.size(); }, _data);
}
