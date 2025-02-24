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

/**
 * @brief      A grid map discretizes a portion of space,
 * holding values in an Eigen matrix.
 *
 *
 *
 */
struct NAVGROUND_CORE_EXPORT GridMap {
public:
  /**
   * An Eigen array, holding the grid map values.
   */
  using Map =
      Eigen::Array<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  /**
   * An Eigen map, holding the grid map values.
   */
  using ExternalMap = Eigen::Map<Map, Eigen::RowMajor>;
  /**
   * The indices of a cell
   */
  using Cell = Eigen::Vector2<int>;
  /**
   * An action to be applied to a cell during raytracing.
   */
  using ActionType = std::function<void(const Cell &)>;

  /**
   * @brief      Construct a grid map, storing values in an internal \ref Map.
   *
   * @param[in]  width       The width of the map in number of cells
   * @param[in]  height      The height of the map in number of cells
   * @param[in]  resolution  The resolution: size of one cell in meters.
   * @param[in]  origin      The position of the left-bottom corner
   *                         of the map in meters.
   *
   */
  GridMap(unsigned width, unsigned height, double resolution,
          const Vector2 &origin = Vector2::Zero())
      : _map(height, width), _e_map(nullptr, 0, 0), _width(width),
        _height(height), _resolution(resolution), _origin(origin) {}

  /**
   * @brief      Construct a grid map, linking to external values.
   *
   * @param[in]  data        A valid pointer to at least at least width x height
   * values.
   * @param[in]  width       The width of the map in number of cells
   * @param[in]  height      The height of the map in number of cells
   * @param[in]  resolution  The resolution: size of one cell in meters.
   * @param[in]  origin      The position of the left-bottom corner
   *                         of the map in meters.
   *
   */
  GridMap(uint8_t *data, unsigned width, unsigned height, ng_float_t resolution,
          const Vector2 &origin = Vector2::Zero())
      : _e_map(data, height, width), _width(width), _height(height),
        _resolution(resolution), _origin(origin) {}

  /**
   * @brief      Gets the position of the  bottom-left corner
   *             of the map in meters.
   *
   * @return     The left-bottom position.
   */
  Vector2 get_origin() const { return _origin; };
  /**
   * @brief      Sets the the position of the bottom-left corner
   *             of the map in meters.
   *
   * @param[in]  value  The desired value
   */
  void set_origin(const Vector2 &value) { _origin = value; };

  /**
   * @brief      Gets the position of the top-right corner
   *             of the map in meters.
   *
   * @return     The top-right position.
   */
  Vector2 get_top_right() const {
    return _origin + Vector2(_width, _height) * _resolution;
  }

  /**
   * @brief      Gets the position of the  bottom-left corner
   *             of the map in meters.
   *
   * @return     The left-bottom position.
   */
  Vector2 get_bottom_left() const { return _origin; }

  /**
   * @brief      Gets the position of the center of the map in meters.
   *
   * @return     The center.
   */
  Vector2 get_center() const {
    return _origin + Vector2(_width, _height) * _resolution * 0.5;
  }

  /**
   * @brief      Sets the position of the center of the map in meters.
   *
   * @param[in]  value  The desired value
   */
  void set_center(const Vector2 &value) {
    _origin = value - Vector2(_width, _height) * _resolution * 0.5;
  }

  /**
   * @brief      Gets the size of one cell in meters.
   *
   * @return     The resolution.
   */
  ng_float_t get_resolution() const { return _resolution; }
  /**
   * @brief      Sets the size of one cell in meters.
   *
   * @param[in]  value  The desired positive value
   */
  void set_resolution(ng_float_t value) {
    if (value > 0) {
      _resolution = value;
    }
  }
  /**
   * @brief      Gets the number of columns.
   *
   * @return     The width.
   */
  unsigned get_width() const { return _width; }
  /**
   * @brief      Sets the number of columns.
   *
   * Will resize the map if needed.
   *
   * @param      value  A positive value
   */
  void set_width(unsigned &value) {
    if (!_e_map.size() && value > 0) {
      _map.resize(value, _map.rows());
      _width = value;
    }
  }
  /**
   * @brief      Gets the number of rows.
   *
   * @return     The height.
   */
  unsigned get_height() const { return _height; };
  /**
   * @brief      Sets the number of rows.
   *
   * Will resize the map if needed.
   *
   * @param      value  A positive value
   */
  void set_height(unsigned &value) {
    if (!_e_map.size() && value > 0) {
      _map.resize(_map.cols(), value);
      _height = value;
    }
  };

  /**
   * @brief      Returns a constant reference to the map.
   *
   * @return     The map.
   */
  const Eigen::Ref<const Map> get_map() const {
    if (_e_map.size()) {
      return _e_map;
    }
    return _map;
  }
  /**
   * @brief      Returns a reference to the map.
   *
   * @return     The map.
   */
  Eigen::Ref<Map> get_map() {
    if (_e_map.size()) {
      return _e_map;
    }
    return _map;
  }

  /**
   * @brief      Gets the position of a the center of a cell
   *
   * Does not check whether the cell is contained or not in the map.
   *
   * @param[in]  cell  The cell indices
   *
   * @return     The position in meters
   */
  Vector2 get_position_of_cell(const Cell &cell) const;
  /**
   * @brief      Gets the indices corresponding
   *             to a cell at a given position.
   *
   * @param[in]  position  The position
   *
   * @return     The cell or null if the cell lies outside of the map.
   */
  std::optional<Cell>
  get_possible_cell_at_position(const Vector2 &position) const;
  /**
   * @brief      Gets the indices corresponding
   *             to a cell at a given position.
   *
   * When the cell lies outside of the map, it either returns unbounded indices
   * (``clamp=false``) or clamp them (``clamp=false``).
   *
   * @param[in]  position  The position
   * @param[in]  clamp  Whether to clamp
   *
   * @return     The cell.
   */
  Cell get_cell_at_position(const Vector2 &position, bool clamp) const;
  /**
   * @brief      Checks whether the map contains a point.
   *
   * @param[in]  point  The point
   *
   * @return     True if the point is covered by the map.
   */
  bool contains_point(const Vector2 &point) const;

  /**
   * @brief      Moves the center of the map.
   *
   * When snap is enabled, it constraints moves by
   * multiple of the cell size.
   *
   * @param[in]  position  The desired center position
   * @param[in]  value     The cell value set in regions not
   *                       previously covered by the map.
   * @param[in]  snap      Whether to snap movements to the grid.
   */
  void move_center(const Vector2 &position, uint8_t value = 128,
                   bool snap = true);
  /**
   * @brief      Moves the origin of the map.
   *
   * When snap is enabled, it constraints moves by
   * multiple of the cell size.
   *
   * @param[in]  position  The desired origin (bottom-left corner) position
   * @param[in]  value     The cell value set in regions not
   *                       previously covered by the map.
   * @param[in]  snap      Whether to snap movements to the grid.
   */
  void move_origin(const Vector2 &position, uint8_t value = 128,
                   bool snap = true);

  // unsigned int cell_distance(ng_float_t world_distance);
  // void reset_to_unknown();

  /**
   * @brief      Sets the value of all cells.
   *
   * @param[in]  value  The value
   */
  void set_value(uint8_t value);
  /**
   * @brief      Sets the value of a single cell.
   *
   * @param[in]  cell   The cell
   * @param[in]  value  The value
   */
  void set_value_of_cell(const Cell &cell, uint8_t value);
  /**
   * @brief      Sets the value of the cell at a location.
   *
   * @param[in]  point  The cell location in meters
   * @param[in]  value  The value
   */
  void set_value_at_point(const Vector2 &point, uint8_t value);
  /**
   * @brief      Sets the value of all cells in a spatial rectangle
   *
   * @param[in]  bottom_left  The bottom-left corner in meters
   * @param[in]  width        The rectangle width in meters
   * @param[in]  height       The rectangle height in meters
   * @param[in]  value        The value
   */
  void set_value_in_rectangle(const Vector2 &bottom_left, ng_float_t width,
                              ng_float_t height, uint8_t value);
  /**
   * @brief      Sets the value of all cells in a disc
   *
   * @param[in]  center  The center of the disc in meters
   * @param[in]  radius  The radius of the disc in meters
   * @param[in]  value   The value
   */
  void set_value_in_disc(const Vector2 &center, ng_float_t radius,
                         uint8_t value);
  /**
   * @brief      Sets the value of all cells along a line between to points.
   *
   * @param[in]  p1     The line vertex in meters
   * @param[in]  p2     The other line vertex in meters
   * @param[in]  value  The value
   */
  void set_value_on_line(const Vector2 &p1, const Vector2 &p2, uint8_t value);

  /**
   * @brief      Sets the value of all cells along a line between two cells
   *
   * @param[in]  c1     The cell at one vertex
   * @param[in]  c2     The cell at the other vertex
   * @param[in]  value  The value
   */
  void set_value_between_cells(const Cell &c1, const Cell &c2, uint8_t value);

  // void set_footprint_as_freespace(ng_float_t radius);
  // void raycast_freespace(const std::valarray<ng_float_t> &ranges,
  //                        ng_float_t begin, ng_float_t delta);
  // void add_obstacles(const std::valarray<ng_float_t> &ranges, ng_float_t
  // begin,
  //                    ng_float_t delta);

  /**
   * @brief      Applies a function for each cell
   * along a line between two cells using Bresenham's line algorithms.
   *
   * If the line has less than ``min_length`` cells, it will be ignored.
   * If the line has more than ``max_length`` cells, it will be clipped.
   *
   * @param[in]  at          The function
   * @param[in]  c1          The cell at one vertex
   * @param[in]  c2          The cell at the other vertex
   * @param[in]  max_length  The maximum number of cells
   * @param[in]  min_length  The minimum number of cells
   */
  void raytrace_between_cells(
      ActionType at, Cell c1, Cell c2,
      unsigned int max_length = std::numeric_limits<unsigned int>::max(),
      unsigned int min_length = 0);

private:
  void bresenham2D(ActionType at, unsigned abs_da, unsigned abs_db, int error,
                   GridMap::Cell da, GridMap::Cell db, GridMap::Cell cell,
                   unsigned length);
  void move(const Cell &delta, uint8_t value = 128);
  Map _map;
  ExternalMap _e_map;
  ng_float_t _width;
  ng_float_t _height;
  ng_float_t _resolution;
  Vector2 _origin;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_STATES_GRIDMAP_H
