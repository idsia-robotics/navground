#ifndef NAVGROUND_CORE_STATES_GRIDMAP_H
#define NAVGROUND_CORE_STATES_GRIDMAP_H

#include "navground/core/common.h"
#include "navground/core/export.h"
#include "navground/core/types.h"
#include <functional>
#include <limits>
#include <optional>
// #include <valarray>

#include <Eigen/Dense>

namespace navground::core {

struct NAVGROUND_CORE_EXPORT GridMap {
public:
  using Map = Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using ExternalMap = Eigen::Map<Map, Eigen::RowMajor>;
  using Cell = Eigen::Vector2<int>;
  using ActionType = std::function<void(const Cell &)>;

  GridMap(unsigned width, unsigned height, double resolution,
          const Vector2 &origin = Vector2::Zero())
      : _map(height, width), _e_map(nullptr, 0, 0), _width(width),
        _height(height), _resolution(resolution), _origin(origin) {}

  GridMap(uint8_t *data, unsigned width, unsigned height, ng_float_t resolution,
          const Vector2 &origin = Vector2::Zero())
      : _e_map(data, height, width), _width(width), _height(height),
        _resolution(resolution), _origin(origin) {}

  Vector2 get_origin() const { return _origin; };
  void set_origin(const Vector2 &value) { _origin = value; };

  Vector2 get_top_right() const {
    return _origin + Vector2(_width, _height) * _resolution;
  }

  Vector2 get_bottom_left() const { return _origin; }

  Vector2 get_center() const {
    return _origin + Vector2(_width, _height) * _resolution * 0.5;
  };

  void set_center(const Vector2 &value) {
    _origin = value - Vector2(_width, _height) * _resolution * 0.5;
  };

  ng_float_t get_resolution() const { return _resolution; };
  void set_resolution(ng_float_t value) {
    if (value > 0) {
      _resolution = value;
    }
  };

  unsigned get_width() const { return _width; };
  void set_width(unsigned &value) {
    if (!_e_map.size() && value > 0) {
      _map.resize(value, _map.rows());
      _width = value;
    }
  };

  unsigned get_height() const { return _height; };
  void set_height(unsigned &value) {
    if (!_e_map.size() && value > 0) {
      _map.resize(_map.cols(), value);
      _height = value;
    }
  };

  const Eigen::Ref<const Map> get_map() const {
    if (_e_map.size()) {
      return _e_map;
    }
    return _map;
  }

  Eigen::Ref<Map> get_map() {
    if (_e_map.size()) {
      return _e_map;
    }
    return _map;
  }

  Vector2 get_position_of_cell(const Cell &cell) const;
  std::optional<Cell> get_cell_at_position(const Vector2 &position) const;
  Cell get_cell_at_position_no_bounds(const Vector2 &position) const;
  Cell get_clamped_cell_at_position(const Vector2 &position) const;
  bool contains_point(const Vector2 &point) const;

  void move_center(const Vector2 &position);
  void move_origin(const Vector2 &position);

  // unsigned int cell_distance(ng_float_t world_distance);
  // void reset_to_unknown();

  void set_value(uint8_t value);
  void set_value_of_cell(const Cell &cell, uint8_t value);
  void set_value_at_point(const Vector2 &point, uint8_t value);
  void set_value_in_rectangle(const Vector2 &bottom_left, ng_float_t width,
                              ng_float_t height, uint8_t value);
  void set_value_in_disc(const Vector2 &center, ng_float_t radius,
                         uint8_t value);
  void set_value_on_line(const Vector2 &p1, const Vector2 &p2, uint8_t value);

  void set_value_between_cells(const Cell &c1, const Cell &c2, uint8_t value);

  // void set_footprint_as_freespace(ng_float_t radius);
  // void raycast_freespace(const std::valarray<ng_float_t> &ranges,
  //                        ng_float_t begin, ng_float_t delta);
  // void add_obstacles(const std::valarray<ng_float_t> &ranges, ng_float_t
  // begin,
  //                    ng_float_t delta);

  void raytrace_between_cells(
      ActionType at, Cell c1, Cell c2,
      unsigned int max_length = std::numeric_limits<unsigned int>::max(),
      unsigned int min_length = 0);

private:
  void bresenham2D(ActionType at, unsigned abs_da, unsigned abs_db, int error,
                   GridMap::Cell da, GridMap::Cell db, GridMap::Cell cell,
                   unsigned length);
  void move(const Cell &delta);
  Map _map;
  ExternalMap _e_map;
  ng_float_t _width;
  ng_float_t _height;
  ng_float_t _resolution;
  Vector2 _origin;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_STATES_GRIDMAP_H
