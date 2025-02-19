#include "navground/core/states/gridmap.h"

#include <cmath>
#include <iostream>
#include <limits>

namespace navground::core {

static int sign(int x) { return x > 0 ? 1.0 : -1.0; }

void GridMap::raytrace_between_cells(ActionType at, GridMap::Cell c0,
                                     GridMap::Cell c1, unsigned int max_length,
                                     unsigned int min_length) {
  // std::cerr << "raytrace_line " << c0 << " -- " << c1 << std::endl;
  GridMap::Cell delta = c1 - c0;

  // we need to chose how much to scale our dominant dimension,
  // based on the maximum length of the line
  const auto dist = delta.norm();
  if (static_cast<unsigned>(dist) < min_length) {
    return;
  }

  if (static_cast<unsigned>(dist) > min_length) {
    // Adjust starting point and offset to start from min_length distance
    // TODO(JEROME): use floats to avoid overflows
    c0 = c0 + delta * min_length / dist;
    delta = c1 - c0;
  }

  const GridMap::Cell abs_delta{std::abs(delta[0]), std::abs(delta[1])};

  const GridMap::Cell dx{sign(delta[0]), 0};
  const GridMap::Cell dy{0, sign(delta[1])};
  const ng_float_t scale =
      (dist == 0) ? 1 : std::min<ng_float_t>(1, max_length / dist);
  // if x is dominant
  if (abs_delta[0] >= abs_delta[1]) {
    const int error = abs_delta[0] / 2;
    const unsigned length = scale * abs_delta[0];
    bresenham2D(at, abs_delta[0], abs_delta[1], error, dx, dy, c0, length);
    return;
  }

  // otherwise y is dominant
  const int error = abs_delta[1] / 2;
  const unsigned length = scale * abs_delta[1];

  bresenham2D(at, abs_delta[1], abs_delta[0], error, dy, dx, c0, length);
}

void GridMap::bresenham2D(ActionType at, unsigned abs_da, unsigned abs_db,
                          int error, GridMap::Cell da, GridMap::Cell db,
                          GridMap::Cell cell, unsigned length) {
  // std::cerr << "bresenham2D: " << abs_da << " " << abs_db << " " << error;
  // std::cerr << " " << da << " " << db << " " << cell << " " << length <<
  // std::endl;
  for (unsigned i = 0; i < length; ++i) {
    at(cell);
    cell += da;
    error += abs_db;
    if (static_cast<unsigned>(error) >= abs_da) {
      cell += db;
      error -= abs_da;
    }
  }
  at(cell);
}

Vector2 GridMap::get_position_of_cell(const GridMap::Cell &cell) const {
  return _origin + (cell.cast<ng_float_t>() + Vector2{0.5, 0.5}) * _resolution;
}

std::optional<GridMap::Cell>
GridMap::get_cell_at_position(const Vector2 &position) const {
  if (position[0] < _origin[0] || position[1] < _origin[1]) {
    return std::nullopt;
  }
  int mx = static_cast<int>((position[0] - _origin[0]) / _resolution);
  int my = static_cast<int>((position[1] - _origin[1]) / _resolution);
  if (mx < _width && my < _height) {
    return GridMap::Cell{mx, my};
  }
  return std::nullopt;
}

GridMap::Cell
GridMap::get_cell_at_position_no_bounds(const Vector2 &position) const {
  return ((position - _origin) / _resolution).cast<int>();
}

GridMap::Cell
GridMap::get_clamped_cell_at_position(const Vector2 &position) const {
  GridMap::Cell cell = get_cell_at_position_no_bounds(position);
  cell[0] = std::clamp<int>(cell[0], 0, _width);
  cell[1] = std::clamp<int>(cell[1], 0, _height);
  return cell;
}

// unsigned GridMap::cell_distance(ng_float_t world_dist) {
//   const auto dist =
//       std::max<ng_float_t>(0, std::ceil(world_dist / _resolution));
//   return static_cast<unsigned>(dist);
// }

// GridMap GridMap::get_window_around(Vector2 position, unsigned int width,
//                                    unsigned int height) {
//   GridMap window(width, height);
//   window._map = _map.block(i, j, width, height);
// }

// void GridMap::reset_to_unknown() { set_value(128); }

void GridMap::move_center(const Vector2 &position) {
  const GridMap::Cell delta =
      ((position - get_center()) / _resolution).array().rint().cast<int>();
  move(delta);
  set_center(position);
}

void GridMap::move_origin(const Vector2 &position) {
  const GridMap::Cell delta =
      ((position - get_origin()) / _resolution).array().rint().cast<int>();
  move(delta);
  set_origin(position);
}

void GridMap::move(const GridMap::Cell &delta) {
  if (delta == GridMap::Cell::Zero())
    return;
  const int w = _width - abs(delta[0]);
  const int h = _height - abs(delta[1]);
  // std::cerr << "w " << w << ", h " << h << std::endl;
  if (w > 0 && h > 0 && w <= _width && h <= _height) {
    Eigen::Ref<Map> m = get_map();
    if (delta[0] < 0 && delta[1] < 0) {
      // std::cerr << "A" << std::endl;
      m.bottomRightCorner(h, w) = m.topLeftCorner(h, w).eval();
      m.topRows(-delta[1]) = 128;
      m.leftCols(-delta[0]) = 128;
    } else if (delta[0] < 0) {
      // std::cerr << "B" << std::endl;
      m.topRightCorner(h, w) = m.bottomLeftCorner(h, w).eval();
      m.bottomRows(delta[1]) = 128;
      m.leftCols(-delta[0]) = 128;
    } else if (delta[1] < 0) {
      // std::cerr << "C" << std::endl;
      m.bottomLeftCorner(h, w) = m.topRightCorner(h, w).eval();
      m.topRows(-delta[1]) = 128;
      m.rightCols(delta[0]) = 128;
    } else {
      // std::cerr << "D" << std::endl;
      m.topLeftCorner(h, w) = m.bottomRightCorner(h, w).eval();
      m.bottomRows(delta[1]) = 128;
      m.rightCols(delta[0]) = 128;
    }
  }
}

bool GridMap::contains_point(const Vector2 &point) const {
  if (((point - _origin).array() < 0).any())
    return false;
  if (((get_top_right() - point).array() < 0).any())
    return false;
  return true;
}

#if 0
void GridMap::set_footprint_as_freespace(ng_float_t radius) {

  const auto a = get_cell_at_position(Vector2{-radius, -radius});
  // const auto b = get_cell_at_position(Vector2{radius, radius});
  const unsigned r = static_cast<unsigned>(std::ceil(2 * radius / _resolution));
  // const auto d = *b - *a;
  // std::cerr << "FOOTPRINT" << std::endl;
  // std::cerr << *a << std::endl;
  // std::cerr << d << std::endl;
  get_map().block((*a)[0], (*a)[1], r, r).setConstant(255);
}
#endif

void GridMap::set_value_in_rectangle(const Vector2 &bottom_left,
                                     ng_float_t width, ng_float_t height,
                                     uint8_t value) {
  auto map = get_map();
  const auto c0 = get_clamped_cell_at_position(bottom_left);
  const auto c1 =
      get_clamped_cell_at_position(bottom_left + Vector2{width, height});
  const auto d = c1 - c0;
  map.block(c0[1], c0[0], d[1], d[0]) = value;
}

void GridMap::set_value_in_disc(const Vector2 &center, ng_float_t radius,
                                uint8_t value) {
  const auto a = get_cell_at_position(center);
  if (!a)
    return;
  const auto center_cell = *a;
  const ng_float_t r = radius / _resolution;
  const int c = static_cast<int>(std::ceil(r));
  auto map = get_map();
  for (int i = -c; i <= c; ++i) {
    const int j = static_cast<int>(std::round(std::sqrt(r * r - i * i)));
    if (j == 0 && i != 0) {
      continue;
    }
    // std::cerr << a[0] - j << " " << a[1] + i << " " << 2 * j + 1 << " " << 1
    //           << std::endl;
    map.block(center_cell[1] + i, center_cell[0] - j, 1, 2 * j + 1)
        .setConstant(value);
  }
}

void GridMap::set_value_of_cell(const GridMap::Cell &cell, uint8_t value) {
  get_map()(cell[1], cell[0]) = value;
}

void GridMap::set_value_at_point(const Vector2 &point, uint8_t value) {
  const auto cell = get_cell_at_position(point);
  if (cell) {
    set_value_of_cell(*cell, value);
  }
}

void GridMap::set_value(uint8_t value) { get_map() = value; }

void GridMap::set_value_on_line(const Vector2 &p1, const Vector2 &p2,
                                uint8_t value) {
  const auto c1 = get_cell_at_position(p1);
  const auto c2 = get_cell_at_position(p2);
  if (c1 && c2) {
    set_value_between_cells(*c1, *c2, value);
  }
}

void GridMap::set_value_between_cells(const GridMap::Cell &c1,
                                      const GridMap::Cell &c2, uint8_t value) {
  raytrace_between_cells(
      [this, value](const GridMap::Cell &c) { set_value_of_cell(c, value); },
      c1, c2);
}

#if 0
void GridMap::raycast_freespace(const std::valarray<ng_float_t> &ranges,
                                ng_float_t begin_angle,
                                ng_float_t delta_angle) {
  // get the map coordinates of the origin of the sensor
  const Vector2 x0 = Vector2::Zero();
  const Vector2 a = _origin;
  const Vector2 b = a + Vector2{_width, _height} * _resolution;
  const auto c0 = get_cell_at_position(x0);
  const size_t n = ranges.size();
  Eigen::Map<Array> rs(const_cast<ng_float_t *>(&ranges[0]), n);
  const Array angles =
      Array::LinSpaced(n, begin_angle, begin_angle + delta_angle * (n - 1));
  // std::cerr << "ANGLES: " << angles << std::endl;;
  const Array xs = angles.cos() * rs;
  const Array ys = angles.sin() * rs;
  // const ng_float_t max_range = 10;
  // const ng_float_t min_range = 0.01;
  // std::cerr << n << std::endl;
  for (size_t i = 0; i < n; i++) {

    // now we also need to make sure that the enpoint we're raytracing
    // to isn't off the costmap and scale if necessary
    Vector2 x1{xs[i], ys[i]};
    const auto delta = x1 - x0;

    // std::cerr << "raycast_freespace " << x0 << " -- " << x1 << std::endl;

    for (int i = 0; i < 2; ++i) {
      // the minimum value to raytrace from is the origin
      if (x1[i] < a[i]) {
        const auto t = (a[i] - x0[i]) / delta[i];
        x1[i] = a[i];
        x1[1 - i] = x0[1 - i] + delta[1 - i] * t;
      }
      // the maximum value to raytrace to is the end of the map
      if (x1[i] > b[i]) {
        const auto t = (b[i] - x0[i]) / delta[i];
        x1[i] = b[i] - .001;
        x1[1 - i] = x0[1 - i] + delta[1 - i] * t;
      }
    }

    // std::cerr << "=> raycast_freespace " << x0 << " -- " << x1 << std::endl;

    const auto c1 = get_cell_at_position(x1);
    if (!c1) {
      // std::cerr << "ERROR\n";
      continue;
    }

    raytrace_line([this](const GridMap::Cell &cell) { set_value(cell, 255); },
                  *c0, *c1);
  }
}

void GridMap::add_obstacles(const std::valarray<ng_float_t> &ranges,
                            ng_float_t begin_angle, ng_float_t delta_angle) {

  const size_t n = ranges.size();
  Eigen::Map<Array> rs(const_cast<ng_float_t *>(&ranges[0]), n);
  const Array angles =
      Array::LinSpaced(n, begin_angle, begin_angle + delta_angle * (n - 1));
  const Array xs = angles.cos() * rs;
  const Array ys = angles.sin() * rs;
  const ng_float_t max_range = 10;
  const ng_float_t min_range = 0.01;
  // std::cerr << n << std::endl;
  for (size_t i = 0; i < n; i++) {
    // std::cerr << rs[i] << std::endl;
    // if the point is far enough away... we won't consider it
    if (rs[i] >= max_range) {
      continue;
    }

    // if the point is too close, do not conisder it
    if (rs[i] < min_range) {
      continue;
    }

    auto cell = get_cell_at_position({xs[i], ys[i]});
    if (!cell) {
      continue;
    }
    // std::cerr << *cell << std::endl;
    set_value(*cell, 0);
  }
}

#endif

} // namespace navground::core
